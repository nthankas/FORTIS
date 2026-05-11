"""URDF parsing and serialisation.

Wraps ``lxml.etree`` so the rest of the pipeline operates on a single
``UrdfDoc`` object instead of raw element trees. Everything mutates the
underlying ``ElementTree`` in place; the wrapper exists so callers can
ask "list me the link names" without rediscovering the XPath.

Why lxml and not the stdlib ``xml.etree``: lxml preserves attribute
order, comments, and CDATA on round-trip, which keeps the diff between
a hand-edited URDF and a tool-emitted URDF small. Pretty-printing also
matters for review.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, List, Optional

from lxml import etree


@dataclass
class Joint:
    """Lightweight view over a ``<joint>`` element."""

    element: etree._Element

    @property
    def name(self) -> str:
        return self.element.get("name", "")

    @property
    def joint_type(self) -> str:
        return self.element.get("type", "")

    @property
    def parent(self) -> str:
        node = self.element.find("parent")
        return node.get("link", "") if node is not None else ""

    @parent.setter
    def parent(self, value: str) -> None:
        node = self.element.find("parent")
        if node is None:
            node = etree.SubElement(self.element, "parent")
        node.set("link", value)

    @property
    def child(self) -> str:
        node = self.element.find("child")
        return node.get("link", "") if node is not None else ""

    @child.setter
    def child(self, value: str) -> None:
        node = self.element.find("child")
        if node is None:
            node = etree.SubElement(self.element, "child")
        node.set("link", value)

    def set_name(self, value: str) -> None:
        self.element.set("name", value)


@dataclass
class Link:
    """Lightweight view over a ``<link>`` element."""

    element: etree._Element

    @property
    def name(self) -> str:
        return self.element.get("name", "")

    def set_name(self, value: str) -> None:
        self.element.set("name", value)


class UrdfDoc:
    """A parsed URDF document with helpers the pipeline needs."""

    def __init__(self, tree: etree._ElementTree):
        self._tree = tree
        self._root = tree.getroot()
        if self._root.tag != "robot":
            raise ValueError(
                f"expected <robot> root, got <{self._root.tag}>"
            )

    @classmethod
    def from_string(cls, xml: str) -> "UrdfDoc":
        parser = etree.XMLParser(remove_blank_text=False)
        root = etree.fromstring(xml.encode("utf-8"), parser)
        return cls(etree.ElementTree(root))

    @classmethod
    def from_path(cls, path: Path) -> "UrdfDoc":
        parser = etree.XMLParser(remove_blank_text=False)
        tree = etree.parse(str(path), parser)
        return cls(tree)

    # --- accessors ----------------------------------------------------

    @property
    def root(self) -> etree._Element:
        return self._root

    @property
    def robot_name(self) -> str:
        return self._root.get("name", "")

    def set_robot_name(self, value: str) -> None:
        self._root.set("name", value)

    def links(self) -> List[Link]:
        return [Link(el) for el in self._root.findall("link")]

    def joints(self) -> List[Joint]:
        return [Joint(el) for el in self._root.findall("joint")]

    def link_names(self) -> List[str]:
        return [el.get("name", "") for el in self._root.findall("link")]

    def joint_names(self) -> List[str]:
        return [el.get("name", "") for el in self._root.findall("joint")]

    def find_link(self, name: str) -> Optional[Link]:
        for link in self.links():
            if link.name == name:
                return link
        return None

    def find_joint(self, name: str) -> Optional[Joint]:
        for joint in self.joints():
            if joint.name == name:
                return joint
        return None

    def joints_with_parent(self, parent_name: str) -> Iterator[Joint]:
        for joint in self.joints():
            if joint.parent == parent_name:
                yield joint

    def joints_with_child(self, child_name: str) -> Iterator[Joint]:
        for joint in self.joints():
            if joint.child == child_name:
                yield joint

    # --- mutation -----------------------------------------------------

    def remove_link(self, name: str) -> bool:
        """Remove a ``<link>`` by name; True if removed."""
        link = self.find_link(name)
        if link is None:
            return False
        self._root.remove(link.element)
        return True

    def remove_joint(self, name: str) -> bool:
        """Remove a ``<joint>`` by name; True if removed."""
        joint = self.find_joint(name)
        if joint is None:
            return False
        self._root.remove(joint.element)
        return True

    def add_raw(self, element: etree._Element) -> None:
        """Append a pre-built element (e.g. ``<ros2_control>``) to ``<robot>``."""
        self._root.append(element)

    # --- serialisation ------------------------------------------------

    def to_string(self, pretty: bool = True) -> str:
        # Re-indent so the diff against a hand-edited URDF stays small.
        if pretty:
            _indent(self._root)
        body = etree.tostring(
            self._root,
            encoding="unicode",
            pretty_print=pretty,
        )
        return '<?xml version="1.0" ?>\n' + body

    def write(self, path: Path, pretty: bool = True) -> None:
        path.write_text(self.to_string(pretty=pretty), encoding="utf-8")


def _indent(elem: etree._Element, level: int = 0) -> None:
    """Pretty-print ``elem`` in place with two-space indentation."""
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        for child in elem:
            _indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = indent + "  "
        if not child.tail or not child.tail.strip():  # noqa: F821  -- last child
            child.tail = indent
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent


def iter_mesh_filenames(doc: UrdfDoc) -> Iterable[str]:
    """Yield every ``filename=...`` value on every ``<mesh>`` in the doc."""
    for mesh in doc.root.iter("mesh"):
        fname = mesh.get("filename")
        if fname:
            yield fname
