from __future__ import annotations

import os
import sys

import omni.usd


def main() -> None:
    stage_path = sys.argv[1] if len(sys.argv) >= 2 else os.environ.get("PIANIST_STAGE_PATH", "")
    if not stage_path:
        print("[WARN] No stage path passed to startup script.", flush=True)
        return

    opened = omni.usd.get_context().open_stage(stage_path)
    print(f"[INFO] open_stage_on_startup: {stage_path} opened={opened}", flush=True)
    if opened:
        stage = omni.usd.get_context().get_stage()
        for path in (
            "/World/PianistRobot/arm",
            "/World/PianistRobot/arm/link_eef/adapter",
            "/World/PianistRobot/arm/link_eef/hand",
            # Backward-compatibility with older nested hand layout.
            "/World/PianistRobot/arm/link_eef/adapter/hand",
        ):
            prim = stage.GetPrimAtPath(path)
            print(f"[INFO] open_stage_on_startup prim {path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)


if __name__ == "__main__":
    main()
