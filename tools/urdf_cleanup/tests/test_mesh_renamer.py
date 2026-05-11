"""Tests for urdf_cleanup.mesh_renamer."""

from __future__ import annotations

from pathlib import Path

import pytest
from urdf_cleanup.mesh_renamer import MeshRenamer
from urdf_cleanup.parser import UrdfDoc


SAMPLE = """<?xml version="1.0" ?>
<robot name="x">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="meshes/base_link__1__.stl"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="meshes/base_link__1__.stl"/>
      </geometry>
    </collision>
  </link>
  <link name="front_left_wheel_link">
    <visual>
      <geometry>
        <mesh filename="meshes/wheel_hub_fl__1__.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"""


def test_rewrites_mesh_filenames_no_copy() -> None:
    doc = UrdfDoc.from_string(SAMPLE)
    r = MeshRenamer(copy=False)
    report = r.rename(doc)
    # Two unique meshes -> two rewrites (the duplicate collision/visual
    # collapses onto the same name because both belong to base_link).
    assert (
        report.rewritten_filenames["meshes/base_link__1__.stl"]
        == "meshes/base_link.stl"
    )
    assert (
        report.rewritten_filenames["meshes/wheel_hub_fl__1__.stl"]
        == "meshes/front_left_wheel_link.stl"
    )
    # Verify the URDF now references the clean names.
    out = doc.to_string()
    assert "meshes/base_link.stl" in out
    assert "meshes/front_left_wheel_link.stl" in out
    assert "wheel_hub_fl__1__" not in out


def test_copies_meshes_to_output_dir(tmp_path: Path) -> None:
    in_dir = tmp_path / "in"
    out_dir = tmp_path / "out"
    in_dir.mkdir()
    (in_dir / "base_link__1__.stl").write_text("FAKE STL CONTENT")
    (in_dir / "wheel_hub_fl__1__.stl").write_text("FAKE STL CONTENT")

    doc = UrdfDoc.from_string(SAMPLE)
    r = MeshRenamer(input_meshes_dir=in_dir, output_meshes_dir=out_dir, copy=True)
    report = r.rename(doc)

    assert (out_dir / "base_link.stl").exists()
    assert (out_dir / "front_left_wheel_link.stl").exists()
    assert any("base_link.stl" in p for p in report.copied_files)


def test_records_warning_when_mesh_missing_on_disk(tmp_path: Path) -> None:
    in_dir = tmp_path / "in"
    out_dir = tmp_path / "out"
    in_dir.mkdir()
    # Intentionally do NOT create the .stl files.

    doc = UrdfDoc.from_string(SAMPLE)
    r = MeshRenamer(input_meshes_dir=in_dir, output_meshes_dir=out_dir, copy=True)
    report = r.rename(doc)
    # URDF gets rewritten regardless; we just warn about the missing files.
    assert "meshes/base_link__1__.stl" in report.rewritten_filenames
    assert any("not found" in w for w in report.warnings)


def test_skips_meshes_without_enclosing_link() -> None:
    xml = """<?xml version="1.0" ?>
<robot name="x">
  <link name="a"><visual><geometry><mesh filename="meshes/a.stl"/></geometry></visual></link>
  <orphan_mesh_holder><mesh filename="orphan.stl"/></orphan_mesh_holder>
</robot>"""
    doc = UrdfDoc.from_string(xml)
    report = MeshRenamer(copy=False).rename(doc)
    assert "orphan.stl" in report.skipped_files
    assert any("no enclosing" in w for w in report.warnings)
