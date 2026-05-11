"""Tests for urdf_cleanup.parser."""

from __future__ import annotations

from pathlib import Path

import pytest
from urdf_cleanup.parser import UrdfDoc, iter_mesh_filenames


def test_from_path_loads_robot_root(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    assert doc.robot_name == "fortis_chassis"
    assert doc.root.tag == "robot"


def test_from_string_round_trip() -> None:
    xml = '<?xml version="1.0" ?><robot name="x"><link name="a"/></robot>'
    doc = UrdfDoc.from_string(xml)
    assert doc.link_names() == ["a"]


def test_rejects_non_robot_root() -> None:
    with pytest.raises(ValueError):
        UrdfDoc.from_string("<garbage/>")


def test_link_and_joint_listing(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    assert "root" in doc.link_names()
    assert "base_link__1__" in doc.link_names()
    assert "hanging_node_to_root_joint_0" in doc.joint_names()


def test_joint_parent_child_accessors(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    joint = doc.find_joint("hanging_node_to_root_joint_1")
    assert joint is not None
    assert joint.parent == "root"
    assert joint.child == "wheel_hub_fl__1__"
    assert joint.joint_type == "continuous"


def test_remove_link_and_joint(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    assert doc.remove_joint("spacer_attach_0") is True
    assert doc.find_joint("spacer_attach_0") is None
    assert doc.remove_link("1p5_spacer__1__") is True
    assert "1p5_spacer__1__" not in doc.link_names()
    assert doc.remove_link("does_not_exist") is False


def test_joints_with_parent_iterator(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    children_of_root = [j.child for j in doc.joints_with_parent("root")]
    assert "base_link__1__" in children_of_root
    assert "wheel_hub_fl__1__" in children_of_root


def test_iter_mesh_filenames(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    files = list(iter_mesh_filenames(doc))
    assert "meshes/base_link__1__.stl" in files
    assert "meshes/wheel_hub_fl__1__.stl" in files


def test_joint_setters_update_underlying_xml() -> None:
    doc = UrdfDoc.from_string(
        '<?xml version="1.0" ?><robot name="x">'
        '<link name="a"/><link name="b"/>'
        '<joint name="j" type="fixed">'
        '<parent link="a"/><child link="b"/></joint></robot>'
    )
    joint = doc.find_joint("j")
    assert joint is not None
    joint.parent = "new_parent"
    joint.child = "new_child"
    joint.set_name("renamed")
    assert doc.find_joint("renamed") is not None
    assert doc.find_joint("renamed").parent == "new_parent"


def test_to_string_round_trip(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    serialised = doc.to_string()
    reparsed = UrdfDoc.from_string(serialised)
    assert reparsed.link_names() == doc.link_names()
    assert reparsed.joint_names() == doc.joint_names()
