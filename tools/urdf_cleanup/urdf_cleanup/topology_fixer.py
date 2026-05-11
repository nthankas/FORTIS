"""Re-parent wheel hubs from the synthetic ``root`` node onto the chassis frame.

OnShape's URDF exporter likes to invent a dummy ``root`` link and
attach anything it can't fit cleanly into the chassis assembly to it
via ``hanging_node_to_root_joint_<N>`` fixed joints. For FORTIS this
catches the four wheel hubs (which the exporter treats as
top-of-assembly bodies). The resulting URDF technically validates but
the wheel revolute joints have ``root`` as their kinematic ancestor,
not the chassis -- which breaks every downstream consumer (RViz draws
wheels at the origin, MoveIt cannot resolve TF, ros2_control can't
plug into a controller manager that expects a single base frame).

This module rewires the hanging-node joints so wheel hubs hang off
the canonical chassis frame (``base_link`` by default). It also drops
the synthetic ``root`` link itself once nothing references it.

The fix is mechanical: any joint whose name matches
``hanging_node_to_root_joint_*`` and whose parent is ``root`` gets its
parent rewritten to the chassis link name. We do NOT try to invent a
joint origin offset; the exporter usually carries the correct
xyz/rpy on the joint itself, and re-deriving from CAD is out of
scope here.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import List

from urdf_cleanup.parser import UrdfDoc


DEFAULT_HANGING_JOINT_PATTERN = r"^hanging_node_to_root_joint_\d+$"
DEFAULT_SYNTHETIC_ROOT = "root"


@dataclass
class TopologyReport:
    """Result of a topology pass."""

    reparented_joints: List[str] = field(default_factory=list)
    new_parent: str = ""
    removed_synthetic_root: bool = False


class TopologyFixer:
    """Rewire hanging-node joints onto the chassis frame."""

    def __init__(
        self,
        chassis_link: str = "base_link",
        hanging_joint_pattern: str = DEFAULT_HANGING_JOINT_PATTERN,
        synthetic_root: str = DEFAULT_SYNTHETIC_ROOT,
    ) -> None:
        self._chassis = chassis_link
        self._pattern = re.compile(hanging_joint_pattern)
        self._synthetic_root = synthetic_root

    def fix(self, doc: UrdfDoc) -> TopologyReport:
        report = TopologyReport(new_parent=self._chassis)

        # The chassis must exist as a real link or we have nothing to
        # parent onto. The caller is expected to either (a) have already
        # renamed the chassis to base_link via the Renamer, or (b)
        # construct TopologyFixer with the actual chassis link name. We
        # tolerate the chassis being absent (e.g. a URDF that genuinely
        # has nothing but root) by no-op-ing rather than crashing.
        if doc.find_link(self._chassis) is None:
            return report

        for joint in doc.joints():
            if not self._pattern.match(joint.name):
                continue
            if joint.parent != self._synthetic_root:
                continue
            # Re-parent.
            joint.parent = self._chassis
            report.reparented_joints.append(joint.name)

        # If the synthetic root still has anything hanging off it, leave
        # it alone -- we do not have evidence it is safe to drop. If
        # nothing references it as parent OR child, retire it.
        still_referenced = any(
            j.parent == self._synthetic_root or j.child == self._synthetic_root
            for j in doc.joints()
        )
        if not still_referenced and doc.find_link(self._synthetic_root) is not None:
            doc.remove_link(self._synthetic_root)
            report.removed_synthetic_root = True

        return report
