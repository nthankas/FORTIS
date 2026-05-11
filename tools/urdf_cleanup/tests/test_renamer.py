"""Tests for urdf_cleanup.renamer."""

from __future__ import annotations

from urdf_cleanup.parser import UrdfDoc
from urdf_cleanup.renamer import Renamer, load_mapping_from_dict


SAMPLE = """<?xml version="1.0" ?>
<robot name="x">
  <link name="base_link__1__"/>
  <link name="wheel_fl__1__"/>
  <link name="unmapped__1__"/>
  <joint name="hanging_node_to_root_joint_1" type="continuous">
    <parent link="base_link__1__"/><child link="wheel_fl__1__"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""


def test_renames_links_and_updates_joint_refs() -> None:
    doc = UrdfDoc.from_string(SAMPLE)
    r = Renamer(
        link_mapping={
            "base_link__1__": "base_link",
            "wheel_fl__1__": "front_left_wheel_link",
        },
        joint_mapping={"hanging_node_to_root_joint_1": "front_left_wheel_joint"},
    )
    report = r.rename(doc)

    assert report.renamed_links == {
        "base_link__1__": "base_link",
        "wheel_fl__1__": "front_left_wheel_link",
    }
    assert report.renamed_joints == {
        "hanging_node_to_root_joint_1": "front_left_wheel_joint"
    }
    j = doc.find_joint("front_left_wheel_joint")
    assert j is not None
    assert j.parent == "base_link"
    assert j.child == "front_left_wheel_link"


def test_unmapped_names_are_recorded_not_renamed() -> None:
    doc = UrdfDoc.from_string(SAMPLE)
    r = Renamer(link_mapping={"base_link__1__": "base_link"})
    report = r.rename(doc)
    assert "unmapped__1__" in report.unmapped_links
    assert "wheel_fl__1__" in report.unmapped_links
    assert "unmapped__1__" in doc.link_names()


def test_load_mapping_from_dict_full() -> None:
    data = {
        "link_mapping": {"a": "b"},
        "joint_mapping": {"j1": "j2"},
    }
    link_map, joint_map = load_mapping_from_dict(data)
    assert link_map == {"a": "b"}
    assert joint_map == {"j1": "j2"}


def test_load_mapping_from_dict_tolerates_missing_sections() -> None:
    link_map, joint_map = load_mapping_from_dict({})
    assert link_map == {}
    assert joint_map == {}
    link_map, joint_map = load_mapping_from_dict({"link_mapping": None})
    assert link_map == {}


def test_rename_pass_handles_joint_with_unmapped_parent() -> None:
    # If the parent link is in the map but the joint is not, joint
    # name stays the same but parent/child get rewritten.
    doc = UrdfDoc.from_string(SAMPLE)
    r = Renamer(link_mapping={"base_link__1__": "base_link"})
    r.rename(doc)
    j = doc.find_joint("hanging_node_to_root_joint_1")
    assert j is not None
    assert j.parent == "base_link"
    # The child was not in the map -- stays mangled.
    assert j.child == "wheel_fl__1__"
