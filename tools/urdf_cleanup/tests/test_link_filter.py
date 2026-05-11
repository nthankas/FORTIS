"""Tests for urdf_cleanup.link_filter."""

from __future__ import annotations

from pathlib import Path

from urdf_cleanup.link_filter import (
    DEFAULT_NUISANCE_PATTERNS,
    LinkFilter,
)
from urdf_cleanup.parser import UrdfDoc


def test_default_patterns_match_known_offenders() -> None:
    f = LinkFilter()
    assert f.matches("1p5_spacer__1__")
    assert f.matches("1p5_spacer")
    assert f.matches("part_1_42__3__")
    assert f.matches("open_cascade_step_translator_7_8_1_1__1__")
    # Should NOT match real links.
    assert not f.matches("base_link__1__")
    assert not f.matches("wheel_hub_fl__1__")
    assert not f.matches("root")


def test_filter_removes_nuisance_subtrees(minimal_export_path: Path) -> None:
    doc = UrdfDoc.from_path(minimal_export_path)
    f = LinkFilter()
    report = f.filter(doc)

    # Three nuisance links go away, plus their attach joints.
    assert "1p5_spacer__1__" in report.removed_links
    assert "open_cascade_step_translator_7_8_1_1__1__" in report.removed_links
    assert "part_1_42__1__" in report.removed_links
    assert "spacer_attach_0" in report.removed_joints
    assert "step_translator_attach_0" in report.removed_joints
    assert "part_attach_0" in report.removed_joints

    # Real links and joints survive.
    assert "base_link__1__" in doc.link_names()
    assert "wheel_hub_fl__1__" in doc.link_names()
    assert "hanging_node_to_root_joint_1" in doc.joint_names()


def test_filter_spares_load_bearing_links() -> None:
    # A nuisance-pattern link with a revolute joint hanging off it must
    # not be dropped.
    xml = """<?xml version="1.0" ?>
<robot name="x">
  <link name="base_link"/>
  <link name="1p5_spacer__1__"/>
  <link name="downstream__1__"/>
  <joint name="attach" type="fixed">
    <parent link="base_link"/><child link="1p5_spacer__1__"/>
  </joint>
  <joint name="downstream_joint" type="continuous">
    <parent link="1p5_spacer__1__"/><child link="downstream__1__"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""
    doc = UrdfDoc.from_string(xml)
    report = LinkFilter().filter(doc)
    assert "1p5_spacer__1__" not in report.removed_links
    assert "1p5_spacer__1__" in report.skipped_load_bearing
    assert "1p5_spacer__1__" in doc.link_names()


def test_extra_patterns_extend_default_set() -> None:
    f = LinkFilter(extra_patterns=[r"^foo_bar(__\d+__)?$"])
    assert f.matches("foo_bar")
    assert f.matches("foo_bar__7__")
    # Defaults still active.
    assert f.matches("1p5_spacer__1__")


def test_default_patterns_module_constant_is_a_sequence() -> None:
    # Cheap regression guard: someone changing the default-patterns
    # type to a string would break iteration in the constructor.
    assert all(isinstance(p, str) for p in DEFAULT_NUISANCE_PATTERNS)
