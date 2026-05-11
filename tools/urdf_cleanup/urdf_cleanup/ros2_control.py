"""Inject a ``<ros2_control>`` block plus a hardware-interface stub.

ROS 2 Humble's ``controller_manager`` reads a ``<ros2_control>`` block
attached to ``<robot>`` to discover joints and their command / state
interfaces. The OnShape exporter never emits this block; the cleanup
tool adds it for the wheel revolute (or continuous) joints so the
URDF can be loaded straight into a ros2_control-aware launch graph.

We inject:

    <ros2_control name="fortis_drive_hw" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
      </hardware>
      <joint name="<wheel_joint_name>">
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      ...one block per wheel joint...
    </ros2_control>

``mock_components/GenericSystem`` is the canonical fake plugin shipped
by ros2_control. It lets the URDF + controller stack come up before
the real ``odrive_ros2_control`` plugin is wired in, which keeps
software development unblocked while hardware bring-up is in flight.
When the real plugin lands, the operator swaps the ``<plugin>`` tag.

We do NOT add ``<actuator>`` or ``<sensor>`` blocks; FORTIS's wheels
are simple direct-drive setups and the joint-level interfaces are
enough for current planning needs. Add those manually if a later
sensor (encoder, current monitor) demands a top-level component.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List

from lxml import etree

from urdf_cleanup.parser import UrdfDoc


# Joint types we treat as actuated for ros2_control purposes. Fixed and
# floating joints are excluded; planar / prismatic could be added later
# if FORTIS grows a linear actuator.
ACTUATED_JOINT_TYPES = frozenset({"continuous", "revolute"})

DEFAULT_HARDWARE_NAME = "fortis_drive_hw"
DEFAULT_PLUGIN = "mock_components/GenericSystem"


@dataclass
class ControlReport:
    injected_joints: List[str] = field(default_factory=list)
    skipped_joints: List[str] = field(default_factory=list)


class Ros2ControlInjector:
    """Append a ``<ros2_control>`` block describing the wheel drivetrain."""

    def __init__(
        self,
        joint_names: Iterable[str],
        hardware_name: str = DEFAULT_HARDWARE_NAME,
        plugin: str = DEFAULT_PLUGIN,
    ) -> None:
        # Keep insertion order so the emitted XML mirrors the user's
        # intent (FL, FR, BL, BR usually).
        self._joint_names = list(joint_names)
        self._hardware_name = hardware_name
        self._plugin = plugin

    @classmethod
    def from_doc_wheel_joints(
        cls,
        doc: UrdfDoc,
        wheel_joint_substring: str = "wheel",
        **kwargs,
    ) -> "Ros2ControlInjector":
        """Heuristic auto-detector for the FORTIS wheel joints.

        Picks every actuated joint whose name contains the wheel
        substring. Fine for the FORTIS chassis where the only revolute
        joints are the four omni wheels; revisit if the arm lands on
        the same URDF before the joint names get more specific.
        """
        names = [
            j.name
            for j in doc.joints()
            if j.joint_type in ACTUATED_JOINT_TYPES
            and wheel_joint_substring in j.name
        ]
        return cls(joint_names=names, **kwargs)

    def inject(self, doc: UrdfDoc) -> ControlReport:
        report = ControlReport()

        # Guard: drop any joint name the caller asked for but that does
        # not exist in the URDF (post-rename, this can happen if the
        # mapping config was stale).
        present = {j.name for j in doc.joints()}
        actuated = {
            j.name for j in doc.joints() if j.joint_type in ACTUATED_JOINT_TYPES
        }

        block = etree.Element("ros2_control", attrib={
            "name": self._hardware_name,
            "type": "system",
        })
        hardware = etree.SubElement(block, "hardware")
        plugin_el = etree.SubElement(hardware, "plugin")
        plugin_el.text = self._plugin

        for jname in self._joint_names:
            if jname not in present:
                report.skipped_joints.append(jname)
                continue
            if jname not in actuated:
                report.skipped_joints.append(jname)
                continue
            joint_el = etree.SubElement(block, "joint", attrib={"name": jname})
            etree.SubElement(joint_el, "command_interface", attrib={"name": "velocity"})
            etree.SubElement(joint_el, "state_interface", attrib={"name": "position"})
            etree.SubElement(joint_el, "state_interface", attrib={"name": "velocity"})
            report.injected_joints.append(jname)

        doc.add_raw(block)
        return report
