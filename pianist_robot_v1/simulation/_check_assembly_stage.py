from __future__ import annotations

import argparse
import os
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--stage", required=True)
args = parser.parse_args()

# Avoid interactive EULA prompt in non-interactive checks.
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
stage_path = Path(args.stage).expanduser().resolve()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
from pxr import UsdPhysics

ctx = omni.usd.get_context()
opened = ctx.open_stage(str(stage_path))
stage = ctx.get_stage()
print(f"stage_opened={opened}", flush=True)
print(f"stage_path={stage_path}", flush=True)

paths = [
    "/pianist_robot/arm/link_eef",
    "/pianist_robot/arm/link_eef/adapter",
    "/pianist_robot/arm/link_eef/hand",
]
for p in paths:
    prim = stage.GetPrimAtPath(p)
    print(f"{p}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

hand_root_candidates = [
    "/pianist_robot/arm/link_eef/hand/base",
    "/pianist_robot/arm/link_eef/hand/hand_base_link",
    "/pianist_robot/arm/link_eef/adapter/hand/base",
    "/pianist_robot/arm/link_eef/adapter/hand/hand_base_link",
    # Legacy non-nested assembly paths (kept for compatibility).
    "/pianist_robot/hand/base",
    "/pianist_robot/hand/hand_base_link",
]
for p in hand_root_candidates:
    prim = stage.GetPrimAtPath(p)
    print(f"{p}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

joints = [
    "/pianist_robot/joint_arm_to_adapter",
    "/pianist_robot/joint_adapter_to_hand",
]
for jp in joints:
    prim = stage.GetPrimAtPath(jp)
    if not prim.IsValid():
        print(f"{jp}: MISSING", flush=True)
        continue
    joint = UsdPhysics.FixedJoint(prim)
    body0 = [str(t) for t in joint.GetBody0Rel().GetTargets()]
    body1 = [str(t) for t in joint.GetBody1Rel().GetTargets()]
    print(f"{jp}: OK", flush=True)
    print(f"  body0={body0}", flush=True)
    print(f"  body1={body1}", flush=True)

simulation_app.close()
