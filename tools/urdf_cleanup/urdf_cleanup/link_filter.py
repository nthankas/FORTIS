"""Drop nuisance links produced by the OnShape URDF exporter.

The exporter emits one ``<link>`` per OnShape feature, which means a
chassis with a dozen real bodies ends up with ~95 links once spacers,
the OpenCascade STEP translator helpers, and the part-number aliases
get spliced in. The vast majority are leaf nodes attached by a fixed
joint and contribute nothing to the kinematic tree the planner cares
about. They also slow RViz, MoveIt, and the URDF parser.

This module identifies and removes those leaf nuisance links plus the
fixed joints that attach them. It is intentionally conservative:

- Only ``fixed`` joints are removed. Continuous / revolute / prismatic
  joints survive even if they touch a nuisance-pattern link, because
  removing them would silently break a kinematic chain.
- A link is only removed if no surviving joint references it as parent
  or child. So if a "nuisance" name is somehow load-bearing, it stays.

The patterns are pulled from the May 10 FORTIS OnShape export. Add
more via the constructor's ``extra_patterns`` argument from the CLI
``--mapping`` config rather than editing this file.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence

from urdf_cleanup.parser import UrdfDoc


# Default nuisance-name regexes. The trailing __<n>__ suffix handles
# OnShape's <1>/<2>/<N> instance markers mangled through the URDF
# exporter (which turns angle brackets into double underscores).
DEFAULT_NUISANCE_PATTERNS: Sequence[str] = (
    r"^1p5_spacer(__\d+__)?$",
    r"^part_\d+_\d+(__\d+__)?$",
    r"^open_cascade_step_translator_\d+_\d+_\d+_\d+(__\d+__)?$",
)


@dataclass
class FilterReport:
    """Result of a filter pass.

    Returned so the CLI can print an honest "removed X links, Y joints"
    line and so tests can assert against specific names rather than
    eyeballing the URDF.
    """

    removed_links: List[str] = field(default_factory=list)
    removed_joints: List[str] = field(default_factory=list)
    skipped_load_bearing: List[str] = field(default_factory=list)


class LinkFilter:
    """Strip nuisance fixed-joint subtrees from a URDF in place."""

    def __init__(self, extra_patterns: Iterable[str] = ()) -> None:
        patterns = list(DEFAULT_NUISANCE_PATTERNS) + list(extra_patterns)
        self._regexes = [re.compile(p) for p in patterns]

    def matches(self, link_name: str) -> bool:
        return any(rx.match(link_name) for rx in self._regexes)

    def filter(self, doc: UrdfDoc) -> FilterReport:
        """Remove every nuisance link + the fixed joint attaching it."""
        report = FilterReport()

        candidates = [name for name in doc.link_names() if self.matches(name)]

        for link_name in candidates:
            attach_joints = [
                j for j in doc.joints() if j.child == link_name
            ]
            child_joints = [
                j for j in doc.joints() if j.parent == link_name
            ]

            # Load-bearing guard: any non-fixed joint touching the link
            # disqualifies it. So does being a parent to anything (we
            # would orphan the children).
            non_fixed_attach = [
                j for j in attach_joints if j.joint_type != "fixed"
            ]
            if non_fixed_attach or child_joints:
                report.skipped_load_bearing.append(link_name)
                continue

            # Safe to remove. Drop the attach joint(s) first so the
            # subsequent link removal does not orphan dangling refs.
            for joint in attach_joints:
                doc.remove_joint(joint.name)
                report.removed_joints.append(joint.name)
            doc.remove_link(link_name)
            report.removed_links.append(link_name)

        return report
