from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--stage", required=True)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd

ctx = omni.usd.get_context()
ctx.open_stage(args.stage)
stage = ctx.get_stage()

roots = [
    "/pianist_robot/arm",
    "/pianist_robot/arm/link_eef/adapter",
    "/pianist_robot/arm/link_eef/hand",
    "/pianist_robot/arm/link_eef/adapter/hand",
]
for root in roots:
    prim = stage.GetPrimAtPath(root)
    print(f"[{root}] valid={prim.IsValid()}")
    if prim.IsValid():
        children = prim.GetChildren()
        print("  children:", [c.GetName() for c in children][:40])

check_paths = [
    "/pianist_robot/arm/link_eef",
    "/pianist_robot/arm/link_eef/adapter",
    "/pianist_robot/arm/link_eef/adapter/geometry",
    "/pianist_robot/arm/link_eef/hand/hand_base_link",
    "/pianist_robot/arm/link_eef/hand/base",
    "/pianist_robot/arm/link_eef/adapter/hand/hand_base_link",
    "/pianist_robot/arm/link_eef/adapter/hand/base",
]
for p in check_paths:
    print(f"{p}: {'OK' if stage.GetPrimAtPath(p).IsValid() else 'MISSING'}")

simulation_app.close()
