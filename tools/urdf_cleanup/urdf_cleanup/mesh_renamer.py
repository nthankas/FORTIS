"""Rename mesh files on disk to match the cleaned link names.

After the Renamer pass, link names look like ``front_left_wheel_link``
but the ``<mesh filename="meshes/wheel_hub_fl__1__.stl"/>`` references
still point at the mangled mesh files OnShape exported. This module
walks every ``<mesh>`` filename in the cleaned URDF, derives a clean
mesh name from the corresponding link, copies the .stl to the output
mesh directory under the clean name, and rewrites the URDF reference.

The mapping is derived heuristically:

1. Find the ``<link>`` that owns the ``<mesh>`` (the nearest parent
   element with a ``name=`` attribute).
2. Replace the original mesh basename with ``<link_name>.stl`` (or
   ``.dae``, preserving extension).

If two visual / collision meshes share a link, both get the same
clean name -- which is wrong. We fall back to the original basename
in that case and emit a warning in the report so the operator can
choose.

The actual file copy is gated by ``output_meshes_dir`` being writable;
when ``copy=False`` we only rewrite the URDF text and leave the
filesystem alone, which is the right behaviour for tests.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from lxml import etree

from urdf_cleanup.parser import UrdfDoc


@dataclass
class MeshReport:
    rewritten_filenames: Dict[str, str] = field(default_factory=dict)
    copied_files: List[str] = field(default_factory=list)
    skipped_files: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def _enclosing_link_name(mesh_el: etree._Element) -> Optional[str]:
    """Walk up until we find the <link name="..."> that owns this mesh."""
    cur = mesh_el.getparent()
    while cur is not None:
        if cur.tag == "link" and cur.get("name"):
            return cur.get("name")
        cur = cur.getparent()
    return None


def _output_basename(link_name: str, original: str) -> str:
    """Compose ``<link_name>.<ext>`` preserving the original extension."""
    ext = Path(original).suffix or ".stl"
    return f"{link_name}{ext}"


class MeshRenamer:
    """Rewrite mesh filenames in the URDF and (optionally) on disk."""

    def __init__(
        self,
        input_meshes_dir: Optional[Path] = None,
        output_meshes_dir: Optional[Path] = None,
        copy: bool = True,
        mesh_url_prefix: str = "meshes/",
    ) -> None:
        self._input = input_meshes_dir
        self._output = output_meshes_dir
        self._copy = copy and (input_meshes_dir is not None) and (output_meshes_dir is not None)
        self._prefix = mesh_url_prefix

    def rename(self, doc: UrdfDoc) -> MeshReport:
        report = MeshReport()
        used_basenames: Dict[str, str] = {}  # new -> link that took it

        for mesh in doc.root.iter("mesh"):
            old_filename = mesh.get("filename")
            if not old_filename:
                continue
            link_name = _enclosing_link_name(mesh)
            if link_name is None:
                report.warnings.append(
                    f"mesh {old_filename!r} has no enclosing <link>; skipped"
                )
                report.skipped_files.append(old_filename)
                continue

            new_basename = _output_basename(link_name, old_filename)

            # Collision check: two meshes on the same link both want the
            # same clean name. Keep the first, warn, leave the second
            # alone (rewriting it would produce a bogus duplicate ref).
            if new_basename in used_basenames and used_basenames[new_basename] != old_filename:
                report.warnings.append(
                    f"link {link_name!r} owns multiple meshes; "
                    f"left {old_filename!r} untouched"
                )
                report.skipped_files.append(old_filename)
                continue
            used_basenames[new_basename] = old_filename

            new_url = f"{self._prefix}{new_basename}"
            mesh.set("filename", new_url)
            report.rewritten_filenames[old_filename] = new_url

            if self._copy:
                self._copy_mesh(old_filename, new_basename, report)

        return report

    def _copy_mesh(
        self,
        old_url: str,
        new_basename: str,
        report: MeshReport,
    ) -> None:
        # Strip the urdf-ish prefix to land in the actual fs path.
        old_basename = Path(old_url).name
        assert self._input is not None and self._output is not None  # type guard
        src = self._input / old_basename
        if not src.exists():
            # Some exports use package:// URLs whose path is not on disk.
            # Don't crash; record it so the operator can find them later.
            report.warnings.append(
                f"mesh source not found on disk: {src}; URDF still rewritten"
            )
            return
        self._output.mkdir(parents=True, exist_ok=True)
        dst = self._output / new_basename
        shutil.copy2(src, dst)
        report.copied_files.append(str(dst))
