from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any

import yaml
from isaaclab.app import AppLauncher


def _joint_name_from_path(path: str) -> str:
    return path.rsplit("/", 1)[-1]


def _safe_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    return float(value)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Introspect USD revolute joints and generate canonical joint_map.yaml."
    )
    parser.add_argument(
        "--stage",
        default="simulation/scenes/pianist_lab.usda",
        help="Path to assembled scene USD/USDA.",
    )
    parser.add_argument(
        "--output",
        default="simulation/joint_map.yaml",
        help="Output YAML path.",
    )
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    stage_path = Path(args.stage).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import omni.usd
    from pxr import UsdPhysics

    ctx = omni.usd.get_context()
    opened = ctx.open_stage(str(stage_path))
    stage = ctx.get_stage()
    if not opened or stage is None:
        raise RuntimeError(f"Failed to open stage: {stage_path}")

    arm_joints: list[dict[str, Any]] = []
    hand_joints: list[dict[str, Any]] = []
    articulation_roots: list[str] = []

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(str(prim.GetPath()))

        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue

        path = str(prim.GetPath())
        joint_name = _joint_name_from_path(path)
        joint = UsdPhysics.RevoluteJoint(prim)
        lower = _safe_float(joint.GetLowerLimitAttr().Get(), 0.0)
        upper = _safe_float(joint.GetUpperLimitAttr().Get(), 0.0)
        entry = {
            "name": joint_name,
            "path": path,
            "lower_deg": lower,
            "upper_deg": upper,
        }
        if "/arm/joints/" in path:
            arm_joints.append(entry)
        elif "/hand/joints/" in path:
            hand_joints.append(entry)

    if not arm_joints:
        raise RuntimeError("No arm joints found. Check stage path and assembly.")
    if not hand_joints:
        raise RuntimeError("No hand joints found. Check stage path and assembly.")

    arm_root = arm_joints[0]["path"].split("/arm/joints/")[0]

    arm_order = [f"joint{i}" for i in range(1, 7)]
    arm_joint_by_name = {j["name"]: j for j in arm_joints}
    ordered_arm = [arm_joint_by_name[name] for name in arm_order if name in arm_joint_by_name]

    hand_joint_by_name = {j["name"]: j for j in hand_joints}
    finger_layout = {
        "index": ["index_proximal_joint", "index_intermediate_joint"],
        "middle": ["middle_proximal_joint", "middle_intermediate_joint"],
        "ring": ["ring_proximal_joint", "ring_intermediate_joint"],
        "pinky": ["pinky_proximal_joint", "pinky_intermediate_joint"],
        "thumb": [
            "thumb_proximal_yaw_joint",
            "thumb_proximal_pitch_joint",
            "thumb_intermediate_joint",
            "thumb_distal_joint",
        ],
    }

    hand_fingers: dict[str, Any] = {}
    for finger, names in finger_layout.items():
        joints = [hand_joint_by_name[n] for n in names if n in hand_joint_by_name]
        hand_fingers[finger] = {
            "joint_paths": [j["path"] for j in joints],
            "open_deg": [j["lower_deg"] for j in joints],
            "closed_deg": [j["upper_deg"] for j in joints],
        }

    data = {
        "version": 1,
        "stage_path": str(stage_path),
        "robot_root": arm_root,
        "articulation_roots": articulation_roots,
        "arm": {
            "joint_order": arm_order,
            "play_center_deg": [0.0, -17.0, -28.0, 0.0, 75.0, -45.0],
            "joints": {
                j["name"]: {
                    "path": j["path"],
                    "lower_deg": j["lower_deg"],
                    "upper_deg": j["upper_deg"],
                }
                for j in ordered_arm
            },
        },
        "hand": {
            "play_finger_order": ["index", "middle", "ring", "pinky"],
            "fingers": hand_fingers,
            "press_fraction_default": 0.40,
        },
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    print(f"Generated joint map: {output_path}")
    print(f"Arm joints discovered: {len(ordered_arm)}")
    print(f"Hand revolute joints discovered: {len(hand_joints)}")

    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
