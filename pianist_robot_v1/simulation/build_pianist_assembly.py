from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _validate_vec3(name: str, value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must be a 3-element list")
    return [float(v) for v in value]


def _as_rel_reference(target: str, output_dir: Path, config_dir: Path) -> str:
    target_path = Path(target)
    if not target_path.is_absolute():
        target_path = (config_dir / target_path).resolve()
    rel_path = os.path.relpath(target_path, output_dir)
    rel_path = rel_path.replace("\\", "/")
    if not rel_path.startswith("./") and not rel_path.startswith("../"):
        rel_path = f"./{rel_path}"
    return rel_path


def _xform_lines(mount: dict[str, Any], indent: str) -> list[str]:
    t = _validate_vec3("mount.translate_m", mount.get("translate_m", [0.0, 0.0, 0.0]))
    r = _validate_vec3("mount.rotate_xyz_deg", mount.get("rotate_xyz_deg", [0.0, 0.0, 0.0]))
    s = _validate_vec3("mount.scale", mount.get("scale", [1.0, 1.0, 1.0]))
    return [
        f"{indent}double3 xformOp:translate = ({t[0]}, {t[1]}, {t[2]})",
        f"{indent}double3 xformOp:rotateXYZ = ({r[0]}, {r[1]}, {r[2]})",
        f"{indent}double3 xformOp:scale = ({s[0]}, {s[1]}, {s[2]})",
        f'{indent}uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]',
    ]


@dataclass(frozen=True)
class _AssetSpec:
    label: str
    usd_ref: str
    mount: dict[str, Any]


def _parent_segments(parent_path: str, root_prim: str) -> list[str]:
    if not parent_path:
        return []
    if not parent_path.startswith("/"):
        raise ValueError(f"parent path must be absolute, got: {parent_path}")
    segments = [s for s in parent_path.strip("/").split("/") if s]
    if not segments or segments[0] != root_prim:
        raise ValueError(
            f"parent path must start with '/{root_prim}', got: {parent_path}"
        )
    return segments[1:]


def _build_children_map(paths: set[tuple[str, ...]]) -> dict[tuple[str, ...], list[str]]:
    nodes: set[tuple[str, ...]] = {()}
    for path in paths:
        for i in range(1, len(path) + 1):
            nodes.add(path[:i])

    edges: dict[tuple[str, ...], set[str]] = {}
    for node in nodes:
        if not node:
            continue
        parent = node[:-1]
        edges.setdefault(parent, set()).add(node[-1])

    return {parent: sorted(children) for parent, children in edges.items()}


def _emit_tree(
    lines: list[str],
    children_map: dict[tuple[str, ...], list[str]],
    assets_by_path: dict[tuple[str, ...], _AssetSpec],
    node_path: tuple[str, ...],
    indent: str,
) -> None:
    for child_name in children_map.get(node_path, []):
        child_path = node_path + (child_name,)
        asset = assets_by_path.get(child_path)
        if asset is None:
            lines.append(f'{indent}over "{child_name}"')
            lines.append(f"{indent}{{")
            _emit_tree(lines, children_map, assets_by_path, child_path, indent + "    ")
            lines.append(f"{indent}}}")
            continue

        lines.append(f'{indent}def Xform "{child_name}" (')
        lines.append(f"{indent}    prepend references = @{asset.usd_ref}@")
        lines.append(f"{indent})")
        lines.append(f"{indent}{{")
        lines.extend(_xform_lines(asset.mount, indent + "    "))
        _emit_tree(lines, children_map, assets_by_path, child_path, indent + "    ")
        lines.append(f"{indent}}}")


def build_assembly(config: dict[str, Any], config_path: Path) -> Path:
    output_usd = config.get("output_usd", "pianist_robot.usda")
    root_prim = str(config.get("root_prim", "pianist_robot"))
    up_axis = str(config.get("up_axis", "Z"))
    meters_per_unit = float(config.get("meters_per_unit", 1.0))

    output_path = Path(output_usd)
    if not output_path.is_absolute():
        output_path = (config_path.parent / output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    arm_cfg = config["arm"]
    adapter_cfg = config["adapter"]
    hand_cfg = config["hand"]
    joints_cfg = config["fixed_joints"]

    arm_ref = _as_rel_reference(str(arm_cfg["usd"]), output_path.parent, config_path.parent)
    adapter_ref = _as_rel_reference(str(adapter_cfg["usd"]), output_path.parent, config_path.parent)
    hand_ref = _as_rel_reference(str(hand_cfg["usd"]), output_path.parent, config_path.parent)

    arm_prim = str(arm_cfg.get("prim", "arm"))
    adapter_prim = str(adapter_cfg.get("prim", "adapter"))
    hand_prim = str(hand_cfg.get("prim", "hand"))

    assets_by_path: dict[tuple[str, ...], _AssetSpec] = {
        (arm_prim,): _AssetSpec(label=arm_prim, usd_ref=arm_ref, mount=arm_cfg.get("mount", {})),
    }

    adapter_parent_segments = _parent_segments(str(adapter_cfg.get("parent", "")), root_prim)
    adapter_path = tuple(adapter_parent_segments + [adapter_prim])
    assets_by_path[adapter_path] = _AssetSpec(
        label=adapter_prim,
        usd_ref=adapter_ref,
        mount=adapter_cfg.get("mount", {}),
    )

    hand_parent_segments = _parent_segments(str(hand_cfg.get("parent", "")), root_prim)
    hand_path = tuple(hand_parent_segments + [hand_prim])
    assets_by_path[hand_path] = _AssetSpec(
        label=hand_prim,
        usd_ref=hand_ref,
        mount=hand_cfg.get("mount", {}),
    )

    if len(assets_by_path) != 3:
        raise ValueError("Duplicate asset path generated; check parent/prim settings in config.")

    arm_to_adapter = joints_cfg["arm_to_adapter"]
    adapter_to_hand = joints_cfg["adapter_to_hand"]

    lines: list[str] = [
        "#usda 1.0",
        "(",
        f'    defaultPrim = "{root_prim}"',
        f'    upAxis = "{up_axis}"',
        f"    metersPerUnit = {meters_per_unit}",
        ")",
        "",
        f'def Xform "{root_prim}"',
        "{",
    ]
    children_map = _build_children_map(set(assets_by_path.keys()))
    _emit_tree(
        lines=lines,
        children_map=children_map,
        assets_by_path=assets_by_path,
        node_path=(),
        indent="    ",
    )
    lines.extend(
        [
            "",
            '    def PhysicsFixedJoint "joint_arm_to_adapter"',
            "    {",
            f'        rel physics:body0 = <{arm_to_adapter["body0"]}>',
            f'        rel physics:body1 = <{arm_to_adapter["body1"]}>',
            "    }",
            "",
            '    def PhysicsFixedJoint "joint_adapter_to_hand"',
            "    {",
            f'        rel physics:body0 = <{adapter_to_hand["body0"]}>',
            f'        rel physics:body1 = <{adapter_to_hand["body1"]}>',
            "    }",
            "}",
            "",
        ]
    )

    output_path.write_text("\n".join(lines), encoding="utf-8")
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Build combined pianist robot USD assembly.")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("assembly_config.json")),
        help="Path to assembly JSON config",
    )
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    config = json.loads(config_path.read_text(encoding="utf-8"))
    output_path = build_assembly(config, config_path)
    print(f"Assembly written: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
