"""Apply a mangled-name -> clean-name mapping across an entire URDF.

OnShape's URDF exporter mangles every link and joint name by appending
``__<N>__`` for OnShape's ``<N>`` instance markers and by lowercasing
spaces to underscores. The result is unreadable in RViz and brittle to
reference from launch files. The fix is a flat string-substitution
table the user supplies via ``--mapping config.yaml``.

The mapping is applied to:

- every ``<link name=...>``
- every ``<joint name=...>``
- every ``<parent link=...>`` and ``<child link=...>`` inside joints

Mesh filenames are NOT rewritten here; that lives in ``mesh_renamer``
because the file rename has to happen on disk alongside the URDF
update.

Names absent from the mapping are left untouched. We deliberately do
NOT try to auto-clean (e.g. strip ``__\\d+__`` blindly) because every
real export contains at least one link whose suffix the user wants to
keep, and a clean failure is better than a silent rename of something
load-bearing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from urdf_cleanup.parser import UrdfDoc


@dataclass
class RenameReport:
    """Result of a rename pass."""

    renamed_links: Dict[str, str] = field(default_factory=dict)
    renamed_joints: Dict[str, str] = field(default_factory=dict)
    unmapped_links: List[str] = field(default_factory=list)
    unmapped_joints: List[str] = field(default_factory=list)


class Renamer:
    """Apply a flat string-substitution map to link + joint names."""

    def __init__(
        self,
        link_mapping: Dict[str, str],
        joint_mapping: Dict[str, str] | None = None,
    ) -> None:
        self._link_map = dict(link_mapping)
        self._joint_map = dict(joint_mapping or {})

    def rename(self, doc: UrdfDoc) -> RenameReport:
        report = RenameReport()

        # Pass 1: rename links. Capture before/after so the joint pass
        # can also update parent/child references using the same map.
        for link in doc.links():
            old = link.name
            new = self._link_map.get(old)
            if new is None:
                report.unmapped_links.append(old)
                continue
            link.set_name(new)
            report.renamed_links[old] = new

        # Pass 2: rename joints + rewrite parent/child link refs.
        for joint in doc.joints():
            # Rewrite parent / child first so we do not race the joint
            # rename in the report.
            if joint.parent in report.renamed_links:
                joint.parent = report.renamed_links[joint.parent]
            if joint.child in report.renamed_links:
                joint.child = report.renamed_links[joint.child]

            old = joint.name
            new = self._joint_map.get(old)
            if new is None:
                report.unmapped_joints.append(old)
                continue
            joint.set_name(new)
            report.renamed_joints[old] = new

        return report


def load_mapping_from_dict(data: dict) -> tuple[Dict[str, str], Dict[str, str]]:
    """Pull link/joint name mappings out of a parsed YAML/JSON config.

    Expected schema::

        link_mapping:
          base_link__1__: base_link
          wheel_hub_fl__1__: front_left_wheel_link
        joint_mapping:
          hanging_node_to_root_joint_1: front_left_wheel_joint

    Both sections are optional. Empty mappings are tolerated so a config
    that only needs joint renames does not require an empty
    ``link_mapping: {}`` line.
    """
    link_map = dict(data.get("link_mapping") or {})
    joint_map = dict(data.get("joint_mapping") or {})
    return link_map, joint_map
