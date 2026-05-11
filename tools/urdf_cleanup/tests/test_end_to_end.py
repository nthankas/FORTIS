"""End-to-end tests for the urdf_cleanup CLI pipeline."""

from __future__ import annotations

from pathlib import Path

import yaml
from urdf_cleanup.main import main, run
from urdf_cleanup.parser import UrdfDoc


def _make_fake_meshes(meshes_dir: Path, urdf_path: Path) -> None:
    """Create empty .stl placeholders for every mesh the URDF references."""
    meshes_dir.mkdir(parents=True, exist_ok=True)
    doc = UrdfDoc.from_path(urdf_path)
    for mesh in doc.root.iter("mesh"):
        fname = mesh.get("filename")
        if not fname:
            continue
        basename = Path(fname).name
        (meshes_dir / basename).write_text("FAKE STL DATA")


def test_run_realistic_export(
    tmp_path: Path,
    realistic_export_path: Path,
) -> None:
    # Prepare a fresh input bundle: URDF + meshes/ in one dir.
    in_dir = tmp_path / "input"
    in_dir.mkdir()
    (in_dir / "fortis_chassis.urdf").write_text(realistic_export_path.read_text())
    _make_fake_meshes(in_dir / "meshes", realistic_export_path)

    out_dir = tmp_path / "output"
    paths = run(in_dir, out_dir, mapping_path=None, verbose=False)

    # Output bundle layout.
    assert paths.output_urdf.exists()
    assert paths.output_launch.exists()
    assert paths.output_rviz.exists()
    assert (out_dir / "meshes").is_dir()

    # Cleaned URDF: nuisance links gone, mangled names cleaned, wheel
    # joints reparented onto base_link, ros2_control block present.
    doc = UrdfDoc.from_path(paths.output_urdf)
    names = doc.link_names()

    # No nuisance links survive.
    for nuisance in (
        "1p5_spacer__1__",
        "part_1_42__1__",
        "open_cascade_step_translator_7_8_1_1__1__",
    ):
        assert nuisance not in names

    # Mangled names cleaned.
    assert "base_link" in names
    assert "front_left_wheel_link" in names
    assert "front_right_wheel_link" in names
    assert "rear_left_wheel_link" in names
    assert "rear_right_wheel_link" in names

    # Wheel joints renamed.
    joint_names = doc.joint_names()
    for clean in (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ):
        assert clean in joint_names

    # Every wheel revolute joint hangs off base_link, not root.
    for jn in joint_names:
        if jn.endswith("_wheel_joint"):
            j = doc.find_joint(jn)
            assert j is not None
            assert j.parent == "base_link", (
                f"{jn} parent is {j.parent!r}, expected base_link"
            )

    # Synthetic root link is gone.
    assert "root" not in names

    # ros2_control block injected.
    raw = paths.output_urdf.read_text(encoding="utf-8")
    assert "<ros2_control" in raw
    assert "fortis_drive_hw" in raw

    # The cleaned URDF should be substantially smaller. (~20 surviving
    # elements vs the source's ~30; the exact number depends on which
    # of the duplicate wheel hubs land in the count.)
    assert len(names) < 12
    assert len(joint_names) < 12


def test_run_with_urdf_path_directly(
    tmp_path: Path,
    realistic_export_path: Path,
) -> None:
    # Make sure --input <file.urdf> also works (the resolver falls back
    # to <parent>/meshes/).
    work = tmp_path / "work"
    work.mkdir()
    (work / "fortis_chassis.urdf").write_text(realistic_export_path.read_text())
    _make_fake_meshes(work / "meshes", realistic_export_path)

    out_dir = tmp_path / "out"
    paths = run(work / "fortis_chassis.urdf", out_dir, mapping_path=None)
    assert paths.output_urdf.exists()


def test_cli_main_returns_zero(tmp_path: Path, realistic_export_path: Path) -> None:
    in_dir = tmp_path / "in"
    in_dir.mkdir()
    (in_dir / "fortis_chassis.urdf").write_text(realistic_export_path.read_text())
    _make_fake_meshes(in_dir / "meshes", realistic_export_path)
    out_dir = tmp_path / "out"

    exit_code = main([
        "--input", str(in_dir),
        "--output", str(out_dir),
        "--verbose",
    ])
    assert exit_code == 0


def test_cli_handles_missing_input(tmp_path: Path) -> None:
    # SystemExit out of _resolve_input is bubbled up.
    try:
        main([
            "--input", str(tmp_path / "no_such_dir"),
            "--output", str(tmp_path / "out"),
        ])
    except SystemExit as exc:
        assert exc.code != 0


def test_run_with_custom_mapping(
    tmp_path: Path,
    realistic_export_path: Path,
) -> None:
    in_dir = tmp_path / "in"
    in_dir.mkdir()
    (in_dir / "fortis_chassis.urdf").write_text(realistic_export_path.read_text())
    _make_fake_meshes(in_dir / "meshes", realistic_export_path)
    out_dir = tmp_path / "out"

    custom = tmp_path / "custom.yaml"
    custom.write_text(yaml.safe_dump({
        "chassis_link": "chassis_body",
        "link_mapping": {
            "base_link__1__": "chassis_body",
            "wheel_hub_fl__1__": "fl_wheel",
        },
        "joint_mapping": {
            "hanging_node_to_root_joint_1": "fl_wheel_axis",
        },
    }))

    paths = run(in_dir, out_dir, mapping_path=custom)
    doc = UrdfDoc.from_path(paths.output_urdf)
    assert "chassis_body" in doc.link_names()
    assert "fl_wheel" in doc.link_names()
    assert "fl_wheel_axis" in doc.joint_names()
    # The wheel that was renamed should hang off chassis_body now.
    j = doc.find_joint("fl_wheel_axis")
    assert j is not None
    assert j.parent == "chassis_body"
