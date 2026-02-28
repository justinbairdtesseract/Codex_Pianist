from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.usd
import omni.timeline
from pxr import Gf


DEFAULT_CENTER_DEG = np.array([-12.9, -18.1, -25.5, -15.1, 73.2, -45.0], dtype=np.float32)
ARM_JOINT_PATHS = [
    "/World/PianistRobot/arm/joints/joint1",
    "/World/PianistRobot/arm/joints/joint2",
    "/World/PianistRobot/arm/joints/joint3",
    "/World/PianistRobot/arm/joints/joint4",
    "/World/PianistRobot/arm/joints/joint5",
    "/World/PianistRobot/arm/joints/joint6",
]
ARM_LINK_PATHS = [
    "/World/PianistRobot/arm/link1",
    "/World/PianistRobot/arm/link2",
    "/World/PianistRobot/arm/link3",
    "/World/PianistRobot/arm/link4",
    "/World/PianistRobot/arm/link5",
    "/World/PianistRobot/arm/link6",
    "/World/PianistRobot/arm/link_eef",
]
ARM_CHAIN = [
    ((0.0, 0.0, 0.364), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (1.5708, -1.5708, 0.0), (0.0, 0.0, 1.0)),
    ((0.39, 0.0, 0.0), (-3.1416, 0.0, -1.5708), (0.0, 0.0, 1.0)),
    ((0.15, 0.426, 0.0), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, -0.09, 0.0), (1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None),
]
NON_ARM_PHYSICS_PREFIXES = [
    "/World/PianistRobot/arm/link_eef/adapter",
    "/World/PianistRobot/arm/link_eef/hand",
    "/World/PianistRobot/joint_arm_to_adapter",
    "/World/PianistRobot/joint_adapter_to_hand",
]


def _parse_center_deg() -> np.ndarray:
    raw = os.environ.get("PIANIST_ARM_CENTER_DEG", "")
    if not raw:
        return DEFAULT_CENTER_DEG.copy()
    parts = [item.strip() for item in raw.split(",") if item.strip()]
    if len(parts) != 6:
        print(
            f"[WARN] Expected 6 comma-separated arm angles in PIANIST_ARM_CENTER_DEG, got: {raw}",
            flush=True,
        )
        return DEFAULT_CENTER_DEG.copy()
    return np.array([float(item) for item in parts], dtype=np.float32)


def _parse_settle_frames() -> int:
    raw = os.environ.get("PIANIST_ARM_SETTLE_FRAMES", "120").strip()
    try:
        return max(1, int(raw))
    except ValueError:
        print(f"[WARN] Invalid PIANIST_ARM_SETTLE_FRAMES={raw!r}; using 120.", flush=True)
        return 120


def _disable_non_arm_physics(stage) -> None:
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not any(path == prefix or path.startswith(prefix + "/") for prefix in NON_ARM_PHYSICS_PREFIXES):
            continue

        for attr_name, value in (
            ("physics:rigidBodyEnabled", False),
            ("physics:jointEnabled", False),
        ):
            attr = prim.GetAttribute(attr_name)
            if attr.IsValid():
                attr.Set(value)


def _rx(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _ry(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rz(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def _axis_angle_matrix(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    axis_vec = np.array(axis, dtype=np.float64)
    axis_vec /= np.linalg.norm(axis_vec)
    x, y, z = axis_vec
    c, s = np.cos(angle), np.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat(rotation: np.ndarray) -> Gf.Quatd:
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z)))


def _compute_arm_link_poses(center_deg: np.ndarray) -> list[tuple[np.ndarray, Gf.Quatf]]:
    joint_rad = np.deg2rad(center_deg.astype(np.float64))
    transforms: list[tuple[np.ndarray, Gf.Quatf]] = []
    current = np.eye(4, dtype=np.float64)
    for joint_idx, (xyz, rpy, axis) in enumerate(ARM_CHAIN):
        fixed = np.eye(4, dtype=np.float64)
        fixed[:3, :3] = _rpy_matrix(*rpy)
        fixed[:3, 3] = np.array(xyz, dtype=np.float64)
        current = current @ fixed
        if axis is not None:
            revolute = np.eye(4, dtype=np.float64)
            revolute[:3, :3] = _axis_angle_matrix(axis, joint_rad[joint_idx])
            current = current @ revolute
        transforms.append((current[:3, 3].copy(), _matrix_to_quat(current[:3, :3])))
    return transforms


def _apply_arm_link_poses(stage, center_deg: np.ndarray) -> None:
    for link_path, (translation, orientation) in zip(ARM_LINK_PATHS, _compute_arm_link_poses(center_deg)):
        prim = stage.GetPrimAtPath(link_path)
        if not prim.IsValid():
            print(f"[WARN] Missing arm link prim: {link_path}", flush=True)
            continue
        translate_attr = prim.GetAttribute("xformOp:translate")
        orient_attr = prim.GetAttribute("xformOp:orient")
        if translate_attr.IsValid():
            translate_attr.Set(Gf.Vec3d(*[float(v) for v in translation]))
        if orient_attr.IsValid():
            orient_attr.Set(orientation)


def main() -> None:
    stage_path = sys.argv[1] if len(sys.argv) >= 2 else os.environ.get("PIANIST_STAGE_PATH", "")
    if not stage_path:
        print("[WARN] No stage path passed to startup script.", flush=True)
        return

    stage_path = str(Path(stage_path).expanduser().resolve())
    opened = omni.usd.get_context().open_stage(stage_path)
    print(f"[INFO] open_stage_center_pose_on_startup: {stage_path} opened={opened}", flush=True)
    if not opened:
        return

    center_deg = _parse_center_deg()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[WARN] No stage available after open.", flush=True)
        return

    _disable_non_arm_physics(stage)
    _apply_arm_link_poses(stage, center_deg)

    for joint_path, target_deg in zip(ARM_JOINT_PATHS, center_deg):
        prim = stage.GetPrimAtPath(joint_path)
        if not prim.IsValid():
            print(f"[WARN] Missing joint prim: {joint_path}", flush=True)
            continue

        # Author the drive target and initial state, then let physics settle briefly so
        # the articulated link transforms in the viewport match the requested pose.
        for attr_name, value in (
            ("drive:angular:physics:targetPosition", float(target_deg)),
            ("drive:angular:physics:targetVelocity", 0.0),
            ("state:angular:physics:position", float(target_deg)),
            ("state:angular:physics:velocity", 0.0),
        ):
            attr = prim.GetAttribute(attr_name)
            if attr.IsValid():
                attr.Set(value)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    app = omni.kit.app.get_app()
    settle_frames = _parse_settle_frames()
    for _ in range(settle_frames):
        app.update()
    timeline.pause()

    print(f"[INFO] Center pose applied (deg): {center_deg.tolist()}", flush=True)
    print(f"[INFO] Timeline paused for inspection pose after {settle_frames} settle frames.", flush=True)


if __name__ == "__main__":
    main()
