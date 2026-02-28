from __future__ import annotations

import argparse
import os
from pathlib import Path

from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert Inspire hand URDF to USD with hand-specific safe defaults."
    )
    parser.add_argument("input", type=str, help="Input URDF path")
    parser.add_argument("output", type=str, help="Output USD/USDa path")
    parser.add_argument("--fix-base", action="store_true", default=False)
    parser.add_argument(
        "--merge-joints",
        action="store_true",
        default=False,
        help="Merge fixed joints during conversion (disabled by default).",
    )
    parser.add_argument("--joint-stiffness", type=float, default=100.0)
    parser.add_argument("--joint-damping", type=float, default=1.0)
    parser.add_argument(
        "--joint-target-type",
        type=str,
        default="position",
        choices=["position", "velocity", "none"],
    )
    parser.add_argument(
        "--make-instanceable",
        action="store_true",
        default=False,
        help="Enable USD instancing. Default is False for cleaner hand visuals.",
    )
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input URDF not found: {input_path}")

    cfg = UrdfConverterCfg(
        asset_path=str(input_path),
        usd_dir=str(output_path.parent),
        usd_file_name=output_path.name,
        force_usd_conversion=True,
        fix_base=args.fix_base,
        merge_fixed_joints=args.merge_joints,
        make_instanceable=args.make_instanceable,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=args.joint_stiffness,
                damping=args.joint_damping,
            ),
            target_type=args.joint_target_type,
        ),
    )

    converter = UrdfConverter(cfg)
    print(f"Generated USD file: {converter.usd_path}")

    # If user chose GUI mode, open stage to inspect.
    if not args.headless:
        sim_utils.open_stage(str(converter.usd_path))
        import omni.kit.app

        app = omni.kit.app.get_app_interface()
        while app.is_running():
            app.update()

    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
