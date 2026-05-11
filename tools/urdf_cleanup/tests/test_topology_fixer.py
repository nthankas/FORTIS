"""Tests for urdf_cleanup.topology_fixer."""

from __future__ import annotations

from pathlib import Path

from urdf_cleanup.parser import UrdfDoc
from urdf_cleanup.topology_fixer import TopologyFixer


def test_reparents_hanging_wheel_hubs(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)

    # First, rename base_link__1__ to base_link so the fixer can find it.
    # (In the real pipeline the Renamer runs before TopologyFixer.)
    base = doc.find_link("base_link__1__")
    assert base is not None
    base.set_name("base_link")
    for j in doc.joints():
        if j.parent == "base_link__1__":
            j.parent = "base_link"
        if j.child == "base_link__1__":
            j.child = "base_link"

    fixer = TopologyFixer(chassis_link="base_link")
    report = fixer.fix(doc)

    # hanging_node_to_root_joint_1 was root->wheel_hub_fl; now base_link->wheel_hub_fl.
    j = doc.find_joint("hanging_node_to_root_joint_1")
    assert j is not None
    assert j.parent == "base_link"
    assert j.child == "wheel_hub_fl__1__"
    assert "hanging_node_to_root_joint_1" in report.reparented_joints

    # hanging_node_to_root_joint_0 was root->base_link; also reparented.
    # That's a self-loop now (base_link -> base_link), which is a real
    # ugliness in the source URDF -- but the fixer's job is to redirect
    # the parent edge, not to delete bogus joints. Downstream cleanup
    # handles dedup. We just verify the parent was rewritten.
    j0 = doc.find_joint("hanging_node_to_root_joint_0")
    assert j0 is not None
    assert j0.parent == "base_link"


def test_drops_synthetic_root_when_unreferenced() -> None:
    xml = """<?xml version="1.0" ?>
<robot name="x">
  <link name="root"/>
  <link name="base_link"/>
  <link name="wheel"/>
  <joint name="hanging_node_to_root_joint_0" type="continuous">
    <parent link="root"/><child link="wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""
    doc = UrdfDoc.from_string(xml)
    report = TopologyFixer(chassis_link="base_link").fix(doc)
    assert report.removed_synthetic_root is True
    assert "root" not in doc.link_names()
    assert doc.find_joint("hanging_node_to_root_joint_0").parent == "base_link"


def test_keeps_synthetic_root_if_still_referenced_after_pass() -> None:
    xml = """<?xml version="1.0" ?>
<robot name="x">
  <link name="root"/>
  <link name="base_link"/>
  <link name="other"/>
  <joint name="hanging_node_to_root_joint_0" type="fixed">
    <parent link="root"/><child link="base_link"/>
  </joint>
  <joint name="not_hanging" type="fixed">
    <parent link="root"/><child link="other"/>
  </joint>
</robot>"""
    doc = UrdfDoc.from_string(xml)
    # not_hanging keeps root alive; only the matching joint is rewritten.
    fixer = TopologyFixer(chassis_link="base_link")
    report = fixer.fix(doc)
    assert "hanging_node_to_root_joint_0" in report.reparented_joints
    assert report.removed_synthetic_root is False
    assert "root" in doc.link_names()


def test_noop_if_chassis_link_missing() -> None:
    xml = """<?xml version="1.0" ?>
<robot name="x">
  <link name="root"/>
  <joint name="hanging_node_to_root_joint_0" type="fixed">
    <parent link="root"/><child link="root"/>
  </joint>
</robot>"""
    doc = UrdfDoc.from_string(xml)
    # No base_link present at all. Fixer should not crash; should not
    # touch anything either.
    report = TopologyFixer(chassis_link="base_link").fix(doc)
    assert report.reparented_joints == []
    assert "root" in doc.link_names()


def test_custom_pattern_and_synthetic_root() -> None:
    xml = """<?xml version="1.0" ?>
<robot name="x">
  <link name="weird_root"/>
  <link name="chassis"/>
  <link name="wheel"/>
  <joint name="weird_attach_0" type="continuous">
    <parent link="weird_root"/><child link="wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""
    doc = UrdfDoc.from_string(xml)
    fixer = TopologyFixer(
        chassis_link="chassis",
        hanging_joint_pattern=r"^weird_attach_\d+$",
        synthetic_root="weird_root",
    )
    report = fixer.fix(doc)
    assert "weird_attach_0" in report.reparented_joints
    assert report.removed_synthetic_root is True
    assert "weird_root" not in doc.link_names()
