from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from pathlib import Path

from check_hardware import DEFAULT_KEYWORDS, find_rs485_ports, ping_host


DEFAULT_XARM_IP = "192.168.0.203"
DEFAULT_HAND_PORT = "/dev/ttyUSB0"
DEFAULT_CENTER_JOINTS_DEG = (0.0, -17.0, -28.0, 0.0, 75.0, -45.0)
DEFAULT_HOME_JOINTS_DEG = (0.0, 0.0, 0.0, 0.0, 90.0, -45.0)
DEFAULT_FINGER_INDICES = (3, 2, 1, 0)  # index -> middle -> ring -> pinky (thumb excluded)


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def _parse_joint_targets(text: str) -> list[float]:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != 6:
        raise ValueError(f"Expected 6 comma-separated joint values, got {len(parts)}")
    return [float(p) for p in parts]


def _parse_finger_indices(text: str) -> list[int]:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != 4:
        raise ValueError("Expected exactly 4 comma-separated finger motor indices (no thumb).")
    values = [int(p) for p in parts]
    if any(v < 0 or v > 5 for v in values):
        raise ValueError("Finger indices must be between 0 and 5.")
    return values


def _resolve_hand_port(requested_port: str) -> str:
    if requested_port != "auto":
        return requested_port

    matches, usb_ports = find_rs485_ports(DEFAULT_KEYWORDS)
    if matches:
        return matches[0].device
    if usb_ports:
        return usb_ports[0].device
    return DEFAULT_HAND_PORT


def _load_center_from_report(default_center: list[float]) -> list[float]:
    report_path = Path("play_center_calibration_results.json")
    if not report_path.exists():
        return default_center

    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        best = report.get("best_candidate", {})
        center = best.get("center_pose_deg")
        if isinstance(center, list) and len(center) == 6:
            return [float(v) for v in center]
    except Exception:
        pass
    return default_center


def _run_precheck(xarm_ip: str, hand_port: str, logger: logging.Logger) -> bool:
    matches, _ = find_rs485_ports(DEFAULT_KEYWORDS)
    ping_ok, ping_output = ping_host(xarm_ip, timeout_seconds=1, count=2)

    device_present = any(getattr(port, "device", "") == hand_port for port in matches) or os.path.exists(hand_port)
    device_access = os.access(hand_port, os.R_OK | os.W_OK) if os.path.exists(hand_port) else False
    hand_ok = device_present and device_access

    logger.info("Precheck xArm ping: %s", "PASS" if ping_ok else "FAIL")
    logger.info("Precheck hand port: %s (%s)", hand_port, "PASS" if hand_ok else "FAIL")
    if not ping_ok:
        logger.warning("xArm ping output: %s", ping_output.replace("\n", " | "))
    if device_present and not device_access:
        logger.warning("Hand serial device exists but current user lacks read/write permissions: %s", hand_port)
    return ping_ok and hand_ok


def _compute_strike_value(open_value: int, close_reference: int, down_fraction: float) -> int:
    down_fraction = max(0.0, min(1.0, down_fraction))
    value = open_value + (close_reference - open_value) * down_fraction
    return int(round(value))


def _force_full_open(hand: object, open_value: int, hold_seconds: float, finger_indices: list[int], logger: logging.Logger) -> bool:
    settle = max(0.20, hold_seconds)
    logger.info("Force full-open step 1: all motors -> %s", open_value)
    if not hand.set_all_motors(open_value, hold_seconds=settle):
        return False

    logger.info("Force full-open step 2: touch each target finger -> %s", open_value)
    per_finger_hold = max(0.04, hold_seconds * 0.5)
    for finger_idx in finger_indices:
        if not hand.move_single_motor(finger_idx, open_value, hold_seconds=per_finger_hold):
            return False

    logger.info("Force full-open step 3: all motors -> %s", open_value)
    return hand.set_all_motors(open_value, hold_seconds=settle)


def run_continuous_practice(
    xarm_ip: str,
    hand_port: str,
    hand_slave_id: int,
    center_joints: list[float],
    home_joints: list[float],
    finger_indices: list[int],
    open_value: int,
    close_reference: int,
    down_fraction: float,
    press_hold: float,
    release_hold: float,
    arm_speed: float,
    arm_acc: float,
    return_home: bool,
    max_loops: int,
) -> bool:
    from drivers import InspireHandConfig, InspireHandDriver, XArmConfig, XArmDriver

    logger = logging.getLogger("continuous_practice")
    strike_value = _compute_strike_value(open_value, close_reference, down_fraction)
    logger.info("Computed strike value: %s", strike_value)

    arm = XArmDriver(XArmConfig(robot_ip=xarm_ip, speed=arm_speed, acceleration=arm_acc))
    hand = InspireHandDriver(InspireHandConfig(port=hand_port, slave_id=hand_slave_id))

    arm_ok = False
    hand_ok = False
    completed = False
    interrupted = False
    loop_idx = 0
    try:
        logger.info("Connecting xArm at %s", xarm_ip)
        arm_ok = arm.connect()
        if not arm_ok:
            logger.error("xArm connection failed.")
            return False

        logger.info("Moving xArm to play center pose: %s", center_joints)
        if not arm.move_joints(center_joints, speed=arm_speed, acceleration=arm_acc, wait=True):
            logger.error("Failed to move xArm to center pose.")
            return False

        logger.info("Connecting Inspire hand on %s (slave %s)", hand_port, hand_slave_id)
        hand_ok = hand.connect()
        if not hand_ok:
            logger.error("Inspire hand connection failed.")
            return False

        logger.info("Setting hand to default fully open position: %s", open_value)
        if not _force_full_open(
            hand=hand,
            open_value=open_value,
            hold_seconds=max(press_hold, release_hold),
            finger_indices=finger_indices,
            logger=logger,
        ):
            logger.error("Failed initial full-open command.")
            return False

        logger.info("Starting continuous practice. Press Ctrl+C to stop.")
        while True:
            if max_loops > 0 and loop_idx >= max_loops:
                logger.info("Reached max loops (%s). Stopping.", max_loops)
                break
            loop_idx += 1
            logger.info("Practice loop %s", loop_idx)
            for finger_idx in finger_indices:
                if not hand.move_single_motor(finger_idx, strike_value, hold_seconds=press_hold):
                    logger.error("Press failed for finger motor %s", finger_idx)
                    return False
                if not hand.move_single_motor(finger_idx, open_value, hold_seconds=release_hold):
                    logger.error("Release failed for finger motor %s", finger_idx)
                    return False

        completed = True
        return True
    except KeyboardInterrupt:
        logger.info("Ctrl+C received. Stopping practice cleanly.")
        interrupted = True
        completed = True
        return True
    finally:
        if hand_ok:
            logger.info("Returning hand to default fully open position: %s", open_value)
            _force_full_open(
                hand=hand,
                open_value=open_value,
                hold_seconds=max(press_hold, release_hold),
                finger_indices=finger_indices,
                logger=logger,
            )
            hand.disconnect()
        if arm_ok:
            if interrupted:
                logger.info("Ctrl+C cleanup: returning xArm to home pose: %s", home_joints)
                arm.move_joints(home_joints, speed=arm_speed, acceleration=arm_acc, wait=True)
            elif return_home:
                logger.info("Returning xArm to home pose: %s", home_joints)
                arm.move_joints(home_joints, speed=arm_speed, acceleration=arm_acc, wait=True)
            elif not completed:
                logger.warning("Practice ended unexpectedly; returning xArm home for safety.")
                arm.move_joints(home_joints, speed=arm_speed, acceleration=arm_acc, wait=True)
            arm.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Continuously run 4-finger note simulation until Ctrl+C, then return hand to default position."
        )
    )
    parser.add_argument("--xarm-ip", default=DEFAULT_XARM_IP, help=f"xArm IP (default: {DEFAULT_XARM_IP})")
    parser.add_argument(
        "--hand-port",
        default="auto",
        help=f"Hand serial port (default: auto; fallback {DEFAULT_HAND_PORT})",
    )
    parser.add_argument("--hand-slave-id", type=int, default=1, help="Modbus slave ID for Inspire hand")
    parser.add_argument(
        "--center-joints",
        default=",".join(str(v) for v in DEFAULT_CENTER_JOINTS_DEG),
        help="Comma-separated 6 joint angles for play center",
    )
    parser.add_argument(
        "--home-joints",
        default=",".join(str(v) for v in DEFAULT_HOME_JOINTS_DEG),
        help="Comma-separated 6 joint angles for home pose",
    )
    parser.add_argument(
        "--finger-indices",
        default=",".join(str(v) for v in DEFAULT_FINGER_INDICES),
        help="Exactly 4 motor indices to test (no thumb), e.g. 3,2,1,0",
    )
    parser.add_argument("--hand-open", type=int, default=1000, help="Fully open hand command value")
    parser.add_argument("--hand-close-ref", type=int, default=120, help="Reference closed command value")
    parser.add_argument(
        "--down-fraction",
        type=float,
        default=0.40,
        help="Press depth as fraction from open toward close reference (default: 0.40)",
    )
    parser.add_argument("--press-hold", type=float, default=0.05, help="Hold time at press depth (seconds)")
    parser.add_argument("--release-hold", type=float, default=0.05, help="Hold time after release (seconds)")
    parser.add_argument("--arm-speed", type=float, default=80.0, help="xArm joint move speed")
    parser.add_argument("--arm-acc", type=float, default=800.0, help="xArm joint move acceleration")
    parser.add_argument(
        "--use-calibration-center",
        action="store_true",
        help="Use center pose from play_center_calibration_results.json if available",
    )
    parser.add_argument(
        "--skip-precheck",
        action="store_true",
        help="Skip connectivity precheck (ping + serial access)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Run even if precheck fails",
    )
    parser.add_argument(
        "--return-home",
        action="store_true",
        help="Also return xArm to home pose after normal completion (Ctrl+C always returns home)",
    )
    parser.add_argument(
        "--max-loops",
        type=int,
        default=0,
        help="For testing only: stop automatically after this many loops (0 = infinite)",
    )
    args = parser.parse_args()

    configure_logging()
    logger = logging.getLogger("continuous_practice")

    try:
        center_joints = _parse_joint_targets(args.center_joints)
        home_joints = _parse_joint_targets(args.home_joints)
        finger_indices = _parse_finger_indices(args.finger_indices)
    except ValueError as exc:
        logger.error("%s", exc)
        return 2

    if args.use_calibration_center:
        center_joints = _load_center_from_report(center_joints)
        logger.info("Using center pose from calibration report: %s", center_joints)

    hand_port = _resolve_hand_port(args.hand_port)
    logger.info("Resolved hand port: %s", hand_port)

    precheck_ok = True
    if not args.skip_precheck:
        precheck_ok = _run_precheck(args.xarm_ip, hand_port, logger)
        if not precheck_ok and not args.force:
            logger.error("Precheck failed. Aborting practice.")
            logger.error("Use --force to run anyway, or fix connectivity and retry.")
            return 1

    ok = run_continuous_practice(
        xarm_ip=args.xarm_ip,
        hand_port=hand_port,
        hand_slave_id=args.hand_slave_id,
        center_joints=center_joints,
        home_joints=home_joints,
        finger_indices=finger_indices,
        open_value=args.hand_open,
        close_reference=args.hand_close_ref,
        down_fraction=args.down_fraction,
        press_hold=args.press_hold,
        release_hold=args.release_hold,
        arm_speed=args.arm_speed,
        arm_acc=args.arm_acc,
        return_home=args.return_home,
        max_loops=args.max_loops,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
