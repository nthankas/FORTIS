"""Tests for urdf_cleanup.launch_generator and urdf_cleanup.rviz_generator."""

from __future__ import annotations

from pathlib import Path

from urdf_cleanup.launch_generator import generate_display_launch
from urdf_cleanup.rviz_generator import generate_rviz_config


def test_launch_file_is_syntactically_valid_python(tmp_path: Path) -> None:
    out = generate_display_launch(
        tmp_path / "display.launch.py",
        urdf_filename="fortis_chassis.urdf",
        rviz_filename="fortis.rviz",
    )
    assert out.exists()
    body = out.read_text(encoding="utf-8")
    # Must compile as Python. (We don't import it because that pulls in
    # the launch package at runtime, which isn't available on the host.)
    compile(body, str(out), "exec")
    # The two filenames the caller supplied appear verbatim.
    assert "fortis_chassis.urdf" in body
    assert "fortis.rviz" in body
    # Contains the three node spawns we expect.
    assert "robot_state_publisher" in body
    assert "joint_state_publisher_gui" in body
    assert "rviz2" in body


def test_launch_filename_substitution_isolates_template_braces(tmp_path: Path) -> None:
    # Regression guard: the template ships a dict literal {{...}} that
    # must NOT be eaten by str.format. The generated launch file should
    # still parse and should still contain `{"robot_description": ...}`.
    out = generate_display_launch(tmp_path / "display.launch.py")
    body = out.read_text(encoding="utf-8")
    assert '{"robot_description"' in body


def test_rviz_config_is_parseable_yaml(tmp_path: Path) -> None:
    import yaml

    out = generate_rviz_config(tmp_path / "fortis.rviz")
    assert out.exists()
    parsed = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert "Visualization Manager" in parsed
    assert "Displays" in parsed["Visualization Manager"]
    # Default fixed frame is base_link.
    assert parsed["Visualization Manager"]["Global Options"]["Fixed Frame"] == "base_link"


def test_rviz_config_honors_custom_fixed_frame(tmp_path: Path) -> None:
    import yaml

    out = generate_rviz_config(tmp_path / "fortis.rviz", fixed_frame="base_footprint")
    parsed = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert parsed["Visualization Manager"]["Global Options"]["Fixed Frame"] == "base_footprint"
