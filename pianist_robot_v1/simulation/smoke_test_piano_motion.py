from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any

import yaml
from isaaclab.app import AppLauncher


def _set_joint_target_deg(stage: Any, joint_path: str, target_deg: float) -> None:
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        raise RuntimeError(f"Joint path not found: {joint_path}")
    attr = prim.GetAttribute("drive:angular:physics:targetPosition")
    if not attr.IsValid():
        raise RuntimeError(f"Joint has no drive targetPosition attribute: {joint_path}")
    attr.Set(float(target_deg))


def _interp(open_deg: float, close_deg: float, fraction: float) -> float:
    return open_deg + (close_deg - open_deg) * fraction


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke test: move arm to play center and cycle four fingers."
    )
    parser.add_argument(
        "--stage",
        default="simulation/scenes/pianist_lab.usda",
        help="Scene USD/USDA path.",
    )
    parser.add_argument(
        "--joint-map",
        default="simulation/joint_map.yaml",
        help="Joint map YAML path from generate_joint_map.py",
    )
    parser.add_argument("--loops", type=int, default=2, help="Number of finger-sequence loops.")
    parser.add_argument(
        "--press-fraction",
        type=float,
        default=None,
        help="Finger press fraction [0..1]. Default from joint_map.yaml.",
    )
    parser.add_argument("--settle-frames", type=int, default=60, help="Frames after arm move.")
    parser.add_argument("--press-frames", type=int, default=12, help="Frames to hold press.")
    parser.add_argument("--release-frames", type=int, default=12, help="Frames to hold release.")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    stage_path = Path(args.stage).expanduser().resolve()
    joint_map_path = Path(args.joint_map).expanduser().resolve()

    if not joint_map_path.exists():
        raise FileNotFoundError(f"Joint map not found: {joint_map_path}")
    config = yaml.safe_load(joint_map_path.read_text(encoding="utf-8"))

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import omni.timeline
    import omni.usd

    ctx = omni.usd.get_context()
    opened = ctx.open_stage(str(stage_path))
    stage = ctx.get_stage()
    if not opened or stage is None:
        raise RuntimeError(f"Failed to open stage: {stage_path}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    def step_frames(count: int) -> None:
        for _ in range(max(0, count)):
            simulation_app.update()

    arm_order = config["arm"]["joint_order"]
    play_center = config["arm"]["play_center_deg"]
    arm_joints_cfg = config["arm"]["joints"]
    hand_fingers = config["hand"]["fingers"]
    play_finger_order = config["hand"]["play_finger_order"]
    press_fraction = (
        float(args.press_fraction)
        if args.press_fraction is not None
        else float(config["hand"]["press_fraction_default"])
    )
    press_fraction = max(0.0, min(1.0, press_fraction))

    print(f"Loaded stage: {stage_path}", flush=True)
    print(f"Using joint map: {joint_map_path}", flush=True)
    print(f"Loops: {args.loops}, press_fraction: {press_fraction:.2f}", flush=True)

    # 1) Move arm to play-center pose.
    for idx, joint_name in enumerate(arm_order):
        joint_path = arm_joints_cfg[joint_name]["path"]
        _set_joint_target_deg(stage, joint_path, play_center[idx])
    print(f"Arm moved to play center: {play_center}", flush=True)
    step_frames(args.settle_frames)

    # 2) Ensure hand starts fully open (all fingers, including thumb).
    for finger_cfg in hand_fingers.values():
        for joint_path, open_deg in zip(finger_cfg["joint_paths"], finger_cfg["open_deg"]):
            _set_joint_target_deg(stage, joint_path, open_deg)
    print("Hand set to fully open.", flush=True)
    step_frames(args.release_frames)

    # 3) Index -> Middle -> Ring -> Pinky sequence.
    for loop_idx in range(args.loops):
        print(f"Loop {loop_idx + 1}/{args.loops}", flush=True)
        for finger_name in play_finger_order:
            finger_cfg = hand_fingers[finger_name]
            joint_paths = finger_cfg["joint_paths"]
            open_positions = finger_cfg["open_deg"]
            closed_positions = finger_cfg["closed_deg"]
            press_positions = [
                _interp(o, c, press_fraction) for o, c in zip(open_positions, closed_positions)
            ]

            for joint_path, target in zip(joint_paths, press_positions):
                _set_joint_target_deg(stage, joint_path, target)
            step_frames(args.press_frames)

            for joint_path, target in zip(joint_paths, open_positions):
                _set_joint_target_deg(stage, joint_path, target)
            step_frames(args.release_frames)

    # End with fully open hand for repeatability.
    for finger_cfg in hand_fingers.values():
        for joint_path, open_deg in zip(finger_cfg["joint_paths"], finger_cfg["open_deg"]):
            _set_joint_target_deg(stage, joint_path, open_deg)
    step_frames(args.release_frames)

    print("Smoke test complete.", flush=True)
    timeline.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
