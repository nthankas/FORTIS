"""Tests for urdf_cleanup.ros2_control."""

from __future__ import annotations

from urdf_cleanup.parser import UrdfDoc
from urdf_cleanup.ros2_control import (
    DEFAULT_HARDWARE_NAME,
    DEFAULT_PLUGIN,
    Ros2ControlInjector,
)


FOUR_WHEEL = """<?xml version="1.0" ?>
<robot name="x">
  <link name="base_link"/>
  <link name="front_left_wheel_link"/>
  <link name="front_right_wheel_link"/>
  <link name="rear_left_wheel_link"/>
  <link name="rear_right_wheel_link"/>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_left_wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_right_wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="rear_left_wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="rear_right_wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="arm_mount_joint" type="fixed">
    <parent link="base_link"/><child link="base_link"/>
  </joint>
</robot>"""


def test_injects_block_with_explicit_joint_list() -> None:
    doc = UrdfDoc.from_string(FOUR_WHEEL)
    injector = Ros2ControlInjector(
        joint_names=[
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
    )
    report = injector.inject(doc)
    assert len(report.injected_joints) == 4
    # Re-parse the serialised XML and check the block is present.
    out = doc.to_string()
    assert "<ros2_control" in out
    assert DEFAULT_HARDWARE_NAME in out
    assert DEFAULT_PLUGIN in out
    assert "command_interface" in out
    # All four wheel joints should appear inside ros2_control.
    for name in (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ):
        assert out.count(name) >= 2  # once as <joint>, once inside <ros2_control>


def test_from_doc_wheel_joints_autodetect() -> None:
    doc = UrdfDoc.from_string(FOUR_WHEEL)
    injector = Ros2ControlInjector.from_doc_wheel_joints(doc)
    report = injector.inject(doc)
    assert sorted(report.injected_joints) == [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]


def test_skips_missing_and_non_actuated_joints() -> None:
    doc = UrdfDoc.from_string(FOUR_WHEEL)
    injector = Ros2ControlInjector(
        joint_names=[
            "front_left_wheel_joint",
            "does_not_exist",
            "arm_mount_joint",  # exists but is fixed
        ]
    )
    report = injector.inject(doc)
    assert report.injected_joints == ["front_left_wheel_joint"]
    assert "does_not_exist" in report.skipped_joints
    assert "arm_mount_joint" in report.skipped_joints


def test_custom_hardware_name_and_plugin() -> None:
    doc = UrdfDoc.from_string(FOUR_WHEEL)
    injector = Ros2ControlInjector(
        joint_names=["front_left_wheel_joint"],
        hardware_name="fortis_drive_real",
        plugin="odrive_ros2_control/ODrivePlugin",
    )
    injector.inject(doc)
    out = doc.to_string()
    assert "fortis_drive_real" in out
    assert "odrive_ros2_control/ODrivePlugin" in out
