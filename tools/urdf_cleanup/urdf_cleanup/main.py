"""CLI entry point for urdf_cleanup.

Usage::

    python -m urdf_cleanup \\
        --input <input_dir_or_urdf> \\
        --output <output_dir> \\
        [--mapping config.yaml] \\
        [--verbose]

``--input`` accepts either a directory (which is expected to contain a
``.urdf`` file plus a ``meshes/`` subdirectory next to it) or a single
``.urdf`` file (in which case meshes are searched for under
``<parent>/meshes/`` relative to the URDF). ``--output`` is a directory
that will be created if missing; the cleaned URDF, the launch.py, the
.rviz, and the renamed meshes/ subdirectory are written there.

The default FORTIS chassis mapping lives in
``configs/fortis_chassis_mapping.yaml`` next to the package. Pass
``--mapping`` to use a different one (a different export, or your own
custom names).
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from urdf_cleanup.launch_generator import generate_display_launch
from urdf_cleanup.link_filter import LinkFilter
from urdf_cleanup.mesh_renamer import MeshRenamer
from urdf_cleanup.parser import UrdfDoc
from urdf_cleanup.renamer import Renamer, load_mapping_from_dict
from urdf_cleanup.ros2_control import Ros2ControlInjector
from urdf_cleanup.rviz_generator import generate_rviz_config
from urdf_cleanup.topology_fixer import TopologyFixer


DEFAULT_MAPPING = (
    Path(__file__).resolve().parent.parent / "configs" / "fortis_chassis_mapping.yaml"
)


@dataclass
class RunPaths:
    input_urdf: Path
    input_meshes: Path
    output_dir: Path
    output_urdf: Path
    output_meshes: Path
    output_launch: Path
    output_rviz: Path


def _resolve_input(input_arg: Path) -> tuple[Path, Path]:
    """Resolve --input to (urdf_path, meshes_dir).

    If ``input_arg`` is a file, treat it as the URDF and look for
    ``<parent>/meshes/`` alongside it. If it's a directory, take the
    first ``*.urdf`` inside and ``meshes/`` within it.
    """
    if input_arg.is_file():
        return input_arg, input_arg.parent / "meshes"
    if input_arg.is_dir():
        urdfs = sorted(input_arg.glob("*.urdf"))
        if not urdfs:
            raise SystemExit(f"no .urdf found in {input_arg}")
        return urdfs[0], input_arg / "meshes"
    raise SystemExit(f"--input does not exist: {input_arg}")


def _build_paths(input_arg: Path, output_arg: Path) -> RunPaths:
    urdf_path, meshes_path = _resolve_input(input_arg)
    out_dir = output_arg
    return RunPaths(
        input_urdf=urdf_path,
        input_meshes=meshes_path,
        output_dir=out_dir,
        output_urdf=out_dir / "urdf" / urdf_path.name,
        output_meshes=out_dir / "meshes",
        output_launch=out_dir / "launch" / "display.launch.py",
        output_rviz=out_dir / "rviz" / "fortis.rviz",
    )


def _load_mapping(path: Optional[Path]) -> dict:
    cfg_path = path if path is not None else DEFAULT_MAPPING
    if not cfg_path.exists():
        # No config -> empty mapping. The pipeline still runs; it just
        # leaves names mangled and reports them in unmapped_*.
        return {}
    return yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}


def run(
    input_arg: Path,
    output_arg: Path,
    mapping_path: Optional[Path] = None,
    verbose: bool = False,
) -> RunPaths:
    """Run the full pipeline. Returns the resolved output paths."""
    paths = _build_paths(input_arg, output_arg)

    def log(msg: str) -> None:
        if verbose:
            print(msg)

    doc = UrdfDoc.from_path(paths.input_urdf)
    log(
        f"loaded URDF: {paths.input_urdf} -- "
        f"{len(doc.link_names())} links / {len(doc.joint_names())} joints"
    )

    # 1. Filter nuisance links.
    filter_report = LinkFilter().filter(doc)
    log(
        f"link_filter: removed {len(filter_report.removed_links)} links, "
        f"{len(filter_report.removed_joints)} joints; "
        f"kept {len(filter_report.skipped_load_bearing)} load-bearing"
    )

    # 2. Fix topology BEFORE renaming.
    #    Why this order: TopologyFixer matches hanging_node_to_root_joint_*
    #    by their original (mangled) names. If the Renamer pass renames
    #    those joints first, the topology pattern misses them and the
    #    rewire is a no-op. The chassis link is also still mangled at
    #    this point, so we look up the mangled name from the user's
    #    link_mapping (reverse lookup: clean name -> mangled key).
    mapping = _load_mapping(mapping_path)
    link_map, joint_map = load_mapping_from_dict(mapping)
    chassis_clean = (mapping.get("chassis_link") if mapping else None) or "base_link"
    mangled_chassis = next(
        (k for k, v in link_map.items() if v == chassis_clean),
        chassis_clean,
    )
    topology_report = TopologyFixer(chassis_link=mangled_chassis).fix(doc)
    log(
        f"topology_fixer: reparented {len(topology_report.reparented_joints)} "
        f"joint(s) onto {chassis_clean!r}; "
        f"dropped {len(topology_report.dropped_joints)} self-loop joint(s); "
        f"removed synthetic root: {topology_report.removed_synthetic_root}"
    )

    # 3. Apply user mapping (mangled -> clean) across surviving links and joints.
    rename_report = Renamer(link_map, joint_map).rename(doc)
    log(
        f"renamer: renamed {len(rename_report.renamed_links)} links, "
        f"{len(rename_report.renamed_joints)} joints"
    )
    if rename_report.unmapped_links and verbose:
        log("  unmapped links (kept original name):")
        for name in rename_report.unmapped_links:
            log(f"    {name}")

    chassis_link = chassis_clean

    # 4. Inject ros2_control block.
    control_report = Ros2ControlInjector.from_doc_wheel_joints(doc).inject(doc)
    log(
        f"ros2_control: injected {len(control_report.injected_joints)} wheel joint(s); "
        f"skipped {len(control_report.skipped_joints)}"
    )

    # 5. Rename meshes (in URDF + on disk).
    mesh_report = MeshRenamer(
        input_meshes_dir=paths.input_meshes,
        output_meshes_dir=paths.output_meshes,
        copy=paths.input_meshes.is_dir(),
    ).rename(doc)
    log(
        f"mesh_renamer: rewrote {len(mesh_report.rewritten_filenames)} mesh ref(s); "
        f"copied {len(mesh_report.copied_files)} file(s)"
    )

    # 6. Set the robot name to the cleaned chassis link if it still
    #    carries the mangled OnShape name. Cosmetic but matches what
    #    a human would do.
    if "__" in doc.robot_name:
        doc.set_robot_name(chassis_link if chassis_link != "base_link" else "fortis_chassis")

    # 7. Emit cleaned URDF + launch + rviz.
    paths.output_urdf.parent.mkdir(parents=True, exist_ok=True)
    doc.write(paths.output_urdf)
    log(f"wrote cleaned URDF: {paths.output_urdf}")

    generate_display_launch(
        paths.output_launch,
        urdf_filename=paths.output_urdf.name,
        rviz_filename=paths.output_rviz.name,
    )
    log(f"wrote launch:        {paths.output_launch}")

    generate_rviz_config(paths.output_rviz, fixed_frame=chassis_link)
    log(f"wrote rviz config:   {paths.output_rviz}")

    log(
        f"OK -- final URDF has {len(doc.link_names())} links / "
        f"{len(doc.joint_names())} joints"
    )
    return paths


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="urdf_cleanup",
        description=(
            "Post-process an OnShape URDF export into a clean, "
            "ROS 2 Humble-ready URDF + launch + rviz bundle."
        ),
    )
    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="Path to the input .urdf file, or a directory containing one.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Output directory (created if missing).",
    )
    parser.add_argument(
        "--mapping",
        type=Path,
        default=None,
        help=(
            "Optional path to a mapping YAML "
            "(default: configs/fortis_chassis_mapping.yaml)."
        ),
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print one log line per pipeline step.",
    )
    args = parser.parse_args(argv)

    try:
        run(args.input, args.output, args.mapping, args.verbose)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 -- top-level CLI handler
        print(f"urdf_cleanup: error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
