from __future__ import annotations

import argparse
import json
import logging
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

from check_hardware import ping_host


DEFAULT_XARM_IP = "192.168.0.203"
DEFAULT_HOME_JOINTS_DEG = (0.0, 0.0, 0.0, 0.0, 90.0, -45.0)

# Candidate center poses for keyboard traversal around a balanced, non-singular zone.
CANDIDATE_CENTERS = {
    "center_a": (0.0, -20.0, -35.0, 0.0, 100.0, -45.0),
    "center_b": (0.0, -17.0, -28.0, 0.0, 75.0, -45.0),
    "center_c": (0.0, -26.0, -44.0, 0.0, 106.0, -45.0),
    "center_d": (0.0, -18.0, -52.0, 0.0, 110.0, -45.0),
}

# xArm 850 joint limits (degrees), used for margin scoring.
JOINT_LIMITS_DEG = (
    (-360.0, 360.0),  # J1
    (-132.0, 132.0),  # J2
    (-242.0, 3.5),    # J3
    (-360.0, 360.0),  # J4
    (-124.0, 124.0),  # J5
    (-360.0, 360.0),  # J6
)


@dataclass
class CandidateResult:
    name: str
    center_pose_deg: list[float]
    ok: bool
    reason: str
    segment_times_sec: list[float]
    mean_segment_time_sec: float
    std_segment_time_sec: float
    speed_raw: float
    smoothness_raw: float
    margin_raw: float
    total_score: float


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def _pose_with_j1_offset(base_pose: Sequence[float], delta_j1_deg: float) -> list[float]:
    pose = list(base_pose)
    pose[0] = pose[0] + delta_j1_deg
    return pose


def _joint_margin_score(angles_deg: Sequence[float]) -> float:
    margins: list[float] = []
    for angle, (lower, upper) in zip(angles_deg, JOINT_LIMITS_DEG):
        if upper <= lower:
            margins.append(0.0)
            continue
        distance_to_edge = min(angle - lower, upper - angle)
        normalized = max(0.0, min(1.0, (2.0 * distance_to_edge) / (upper - lower)))
        margins.append(normalized)
    return min(margins) if margins else 0.0


def _candidate_margin(center_pose_deg: Sequence[float], sweep_delta_j1_deg: float) -> float:
    left = _pose_with_j1_offset(center_pose_deg, -abs(sweep_delta_j1_deg))
    right = _pose_with_j1_offset(center_pose_deg, abs(sweep_delta_j1_deg))
    return min(
        _joint_margin_score(center_pose_deg),
        _joint_margin_score(left),
        _joint_margin_score(right),
    )


def _run_sweep_benchmark(
    arm: "XArmDriver",
    center_pose_deg: Sequence[float],
    sweep_delta_j1_deg: float,
    cycles: int,
    speed: float,
    acceleration: float,
) -> tuple[bool, str, list[float]]:
    left_pose = _pose_with_j1_offset(center_pose_deg, -abs(sweep_delta_j1_deg))
    right_pose = _pose_with_j1_offset(center_pose_deg, abs(sweep_delta_j1_deg))

    # Warmup pass to stabilize first-move effects.
    for warmup_pose in (center_pose_deg, left_pose, right_pose, center_pose_deg):
        if not arm.move_joints(warmup_pose, speed=speed, acceleration=acceleration, wait=True):
            return False, "warmup move failed", []

    segment_times: list[float] = []
    for _ in range(cycles):
        for target_pose in (left_pose, right_pose, center_pose_deg):
            t0 = time.perf_counter()
            ok = arm.move_joints(target_pose, speed=speed, acceleration=acceleration, wait=True)
            elapsed = time.perf_counter() - t0
            if not ok:
                return False, "sweep move failed", segment_times
            segment_times.append(elapsed)

    return True, "ok", segment_times


def _score_results(results: list[CandidateResult]) -> None:
    valid = [r for r in results if r.ok]
    if not valid:
        return

    max_speed = max(r.speed_raw for r in valid) or 1.0
    max_smooth = max(r.smoothness_raw for r in valid) or 1.0
    max_margin = max(r.margin_raw for r in valid) or 1.0

    for r in valid:
        speed_norm = r.speed_raw / max_speed
        smooth_norm = r.smoothness_raw / max_smooth
        margin_norm = r.margin_raw / max_margin
        r.total_score = 0.50 * speed_norm + 0.35 * smooth_norm + 0.15 * margin_norm


def _print_results(results: list[CandidateResult]) -> None:
    print("")
    print("=== Calibration Results ===")
    for r in sorted(results, key=lambda x: x.total_score, reverse=True):
        status = "OK" if r.ok else "FAIL"
        print(
            f"{r.name:9s} | {status:4s} | score={r.total_score:.3f} | "
            f"mean={r.mean_segment_time_sec:.3f}s | std={r.std_segment_time_sec:.3f}s | "
            f"margin={r.margin_raw:.3f} | reason={r.reason}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate xArm play-center pose for horizontal keyboard motion speed and smoothness."
    )
    parser.add_argument("--xarm-ip", default=DEFAULT_XARM_IP, help=f"xArm IP (default: {DEFAULT_XARM_IP})")
    parser.add_argument(
        "--home-joints",
        default=",".join(str(v) for v in DEFAULT_HOME_JOINTS_DEG),
        help="Comma-separated 6 joint angles for safe home pose",
    )
    parser.add_argument("--sweep-delta-j1", type=float, default=10.0, help="J1 +/- sweep delta in degrees")
    parser.add_argument("--cycles", type=int, default=3, help="Left-right-center cycles per candidate")
    parser.add_argument("--speed", type=float, default=60.0, help="Joint speed for calibration moves")
    parser.add_argument("--acc", type=float, default=600.0, help="Joint acceleration for calibration moves")
    parser.add_argument(
        "--output-json",
        default="play_center_calibration_results.json",
        help="Output JSON report path",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually move hardware. Without this flag, script exits after precheck.",
    )
    parser.add_argument(
        "--move-to-best",
        action="store_true",
        help="After calibration, move arm to the selected best center pose.",
    )
    args = parser.parse_args()

    configure_logging()
    logger = logging.getLogger("play_center_calibration")

    try:
        home_joints = [float(v.strip()) for v in args.home_joints.split(",") if v.strip()]
        if len(home_joints) != 6:
            raise ValueError
    except ValueError:
        logger.error("--home-joints must contain exactly 6 comma-separated values.")
        return 2

    ping_ok, ping_output = ping_host(args.xarm_ip, timeout_seconds=1, count=2)
    if not ping_ok:
        logger.error("xArm ping failed for %s. Output: %s", args.xarm_ip, ping_output.replace("\n", " | "))
        return 1

    if not args.execute:
        logger.info("Precheck passed. Re-run with --execute to move hardware and run calibration.")
        return 0

    from drivers import XArmConfig, XArmDriver

    arm = XArmDriver(XArmConfig(robot_ip=args.xarm_ip, speed=args.speed, acceleration=args.acc))
    if not arm.connect():
        logger.error("Failed to connect to xArm.")
        return 1

    results: list[CandidateResult] = []
    best_result: CandidateResult | None = None
    try:
        logger.info("Moving to home pose before calibration: %s", home_joints)
        if not arm.move_joints(home_joints, speed=args.speed, acceleration=args.acc, wait=True):
            logger.error("Failed to reach home pose.")
            return 1

        for name, pose in CANDIDATE_CENTERS.items():
            logger.info("Evaluating candidate %s: %s", name, list(pose))

            margin = _candidate_margin(pose, args.sweep_delta_j1)
            ok, reason, segment_times = _run_sweep_benchmark(
                arm=arm,
                center_pose_deg=pose,
                sweep_delta_j1_deg=args.sweep_delta_j1,
                cycles=args.cycles,
                speed=args.speed,
                acceleration=args.acc,
            )

            if ok and segment_times:
                mean_t = statistics.mean(segment_times)
                std_t = statistics.pstdev(segment_times) if len(segment_times) > 1 else 0.0
                speed_raw = 1.0 / max(mean_t, 1e-6)
                cv = std_t / max(mean_t, 1e-6)
                smooth_raw = 1.0 / max(cv, 1e-6)
            else:
                mean_t = math.inf
                std_t = math.inf
                speed_raw = 0.0
                smooth_raw = 0.0

            results.append(
                CandidateResult(
                    name=name,
                    center_pose_deg=list(pose),
                    ok=ok,
                    reason=reason,
                    segment_times_sec=segment_times,
                    mean_segment_time_sec=mean_t,
                    std_segment_time_sec=std_t,
                    speed_raw=speed_raw,
                    smoothness_raw=smooth_raw,
                    margin_raw=margin,
                    total_score=0.0,
                )
            )

        _score_results(results)
        _print_results(results)

        valid = [r for r in results if r.ok]
        if not valid:
            logger.error("No valid candidates completed calibration.")
            return 1

        best_result = max(valid, key=lambda r: r.total_score)
        logger.info("Selected best center pose: %s -> %s", best_result.name, best_result.center_pose_deg)

        out_path = Path(args.output_json).expanduser().resolve()
        out_path.write_text(
            json.dumps(
                {
                    "xarm_ip": args.xarm_ip,
                    "sweep_delta_j1_deg": args.sweep_delta_j1,
                    "cycles": args.cycles,
                    "speed": args.speed,
                    "acc": args.acc,
                    "best_candidate": asdict(best_result),
                    "all_candidates": [asdict(r) for r in results],
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        logger.info("Wrote calibration report: %s", out_path)

        if args.move_to_best and best_result is not None:
            logger.info("Moving to selected best center pose.")
            if not arm.move_joints(best_result.center_pose_deg, speed=args.speed, acceleration=args.acc, wait=True):
                logger.error("Failed to move to best center pose.")
                return 1
        else:
            logger.info("Returning to home pose.")
            arm.move_joints(home_joints, speed=args.speed, acceleration=args.acc, wait=True)

        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    sys.exit(main())
