from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional


def _resolve_mesh_path(base_dir: Path, mesh_filename: str) -> Path:
    if mesh_filename.startswith("file://"):
        return Path(mesh_filename[len("file://") :])
    mesh_path = Path(mesh_filename)
    if mesh_path.is_absolute():
        return mesh_path
    return (base_dir / mesh_path).resolve()


def _to_urdf_mesh_filename(mesh_path: Path) -> str:
    return str(mesh_path)


def _collision_mesh_for_visual(mesh_path: Path) -> Path:
    collision_candidate = str(mesh_path)
    collision_candidate = collision_candidate.replace("/meshes/visual/", "/meshes/collision/")
    collision_candidate = collision_candidate.replace("\\meshes\\visual\\", "\\meshes\\collision\\")
    if collision_candidate.endswith(".glb"):
        collision_candidate = collision_candidate[:-4] + ".obj"
    return Path(collision_candidate)


def _bake_glb_visual_to_obj(glb_path: Path, baked_mesh_dir: Path) -> Optional[Path]:
    try:
        import trimesh
    except Exception:
        return None

    try:
        scene = trimesh.load(str(glb_path), force="scene")
        if hasattr(scene, "to_geometry"):
            mesh = scene.to_geometry()
        else:
            # Backward-compatible path for older trimesh APIs.
            mesh = scene.dump(concatenate=True)
    except Exception:
        return None

    if mesh is None or getattr(mesh, "vertices", None) is None or len(mesh.vertices) == 0:
        return None

    baked_mesh_dir.mkdir(parents=True, exist_ok=True)
    out_path = baked_mesh_dir / f"{glb_path.stem}_baked.obj"
    try:
        mesh.export(str(out_path), file_type="obj")
    except Exception:
        return None
    return out_path.resolve()


def prepare_urdf(
    input_urdf: Path,
    output_urdf: Path,
    absolute_paths: bool,
    baked_mesh_dir: Path,
    prefer_collision_visuals: bool,
    bake_glb_visuals: bool,
) -> None:
    input_urdf = input_urdf.resolve()
    input_dir = input_urdf.parent

    tree = ET.parse(input_urdf)
    root = tree.getroot()

    for mesh in root.findall(".//visual/geometry/mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        resolved = _resolve_mesh_path(input_dir, filename)
        replacement = resolved

        # For this hand model, Isaac imports collision OBJ meshes more reliably
        # than GLB visuals for link-local alignment and hierarchy.
        if "meshes/visual/" in filename and filename.endswith(".glb"):
            collision_path = _collision_mesh_for_visual(resolved)
            if prefer_collision_visuals and collision_path.exists():
                replacement = collision_path.resolve()
            elif bake_glb_visuals:
                baked = _bake_glb_visual_to_obj(resolved, baked_mesh_dir)
                if baked is not None:
                    replacement = baked
                elif collision_path.exists():
                    replacement = collision_path.resolve()
            elif collision_path.exists():
                replacement = collision_path.resolve()

        if absolute_paths:
            mesh.set("filename", _to_urdf_mesh_filename(replacement.resolve()))
        else:
            try:
                mesh.set(
                    "filename",
                    str(replacement.resolve().relative_to(input_dir.resolve())),
                )
            except ValueError:
                mesh.set("filename", _to_urdf_mesh_filename(replacement.resolve()))

    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Create an Isaac-safe Inspire hand URDF by replacing GLB visual "
            "meshes with stable OBJ alternatives for Isaac conversion."
        )
    )
    parser.add_argument("input", type=Path, help="Input hand URDF path")
    parser.add_argument("output", type=Path, help="Output patched URDF path")
    parser.add_argument(
        "--absolute-paths",
        action="store_true",
        default=True,
        help="Write absolute mesh paths in output URDF (default: enabled).",
    )
    parser.add_argument(
        "--relative-paths",
        action="store_true",
        default=False,
        help="Write mesh paths relative to input URDF directory.",
    )
    parser.add_argument(
        "--baked-mesh-dir",
        type=Path,
        default=Path(__file__).with_name("generated").joinpath("inspire_visual_meshes"),
        help="Directory to place baked OBJ visual meshes.",
    )
    parser.add_argument(
        "--prefer-baked-visuals",
        action="store_true",
        default=False,
        help=(
            "Prefer baked GLB->OBJ visual meshes over collision OBJ meshes. "
            "Default is collision-first for robust link alignment."
        ),
    )
    args = parser.parse_args()

    absolute_paths = args.absolute_paths and not args.relative_paths
    prefer_collision_visuals = not args.prefer_baked_visuals
    prepare_urdf(
        args.input,
        args.output,
        absolute_paths=absolute_paths,
        baked_mesh_dir=args.baked_mesh_dir.expanduser().resolve(),
        prefer_collision_visuals=prefer_collision_visuals,
        bake_glb_visuals=True,
    )
    print(f"Prepared URDF: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
