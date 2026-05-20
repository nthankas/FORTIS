"""URDF-kinematics drift regression.

The X-drive IK module uses three geometry constants that must stay in lockstep
with the canonical robot description: LEN_X, LEN_Y (wheel position from
chassis center) and WHEEL_RADIUS. The URDF is the source of truth; this test
fails the moment either side drifts.

The test parses the xacro files as plain XML (no xacro/ROS dependency in the
test path) so it runs both inside `colcon test` and under bare pytest with
PYTHONPATH=src/fortis_comms.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from fortis_comms.xdrive_kinematics import LEN_X, LEN_Y, WHEEL_RADIUS

XACRO_NS = "http://www.ros.org/wiki/xacro"
TOL_M = 1e-4


def _repo_root() -> Path:
    """Walk up from this test file until we find the FORTIS workspace root."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src" / "fortis_description" / "urdf").is_dir():
            return parent
    raise RuntimeError("Could not locate FORTIS repo root from test file path")


def _load_constants() -> dict:
    """Extract numeric xacro:property values from fortis_constants.xacro."""
    path = (
        _repo_root()
        / "src"
        / "fortis_description"
        / "urdf"
        / "fortis_constants.xacro"
    )
    tree = ET.parse(path)
    out = {}
    for prop in tree.iter(f"{{{XACRO_NS}}}property"):
        name = prop.get("name")
        raw = prop.get("value")
        if name is None or raw is None:
            continue
        try:
            out[name] = float(raw)
        except ValueError:
            continue
    return out


@pytest.fixture(scope="module")
def urdf_constants() -> dict:
    return _load_constants()


def test_len_x_matches_urdf_wheel_x_offset(urdf_constants):
    assert "wheel_x_offset" in urdf_constants, (
        "fortis_constants.xacro is missing wheel_x_offset"
    )
    delta = abs(LEN_X - urdf_constants["wheel_x_offset"])
    assert delta < TOL_M, (
        f"LEN_X ({LEN_X} m) drifted from URDF wheel_x_offset "
        f"({urdf_constants['wheel_x_offset']} m), delta={delta:.6f} m"
    )


def test_len_y_matches_urdf_wheel_y_offset(urdf_constants):
    assert "wheel_y_offset" in urdf_constants, (
        "fortis_constants.xacro is missing wheel_y_offset"
    )
    delta = abs(LEN_Y - urdf_constants["wheel_y_offset"])
    assert delta < TOL_M, (
        f"LEN_Y ({LEN_Y} m) drifted from URDF wheel_y_offset "
        f"({urdf_constants['wheel_y_offset']} m), delta={delta:.6f} m"
    )


def test_wheel_radius_matches_urdf(urdf_constants):
    assert "wheel_radius" in urdf_constants, (
        "fortis_constants.xacro is missing wheel_radius"
    )
    delta = abs(WHEEL_RADIUS - urdf_constants["wheel_radius"])
    assert delta < TOL_M, (
        f"WHEEL_RADIUS ({WHEEL_RADIUS} m) drifted from URDF wheel_radius "
        f"({urdf_constants['wheel_radius']} m), delta={delta:.6f} m"
    )


def test_urdf_wheel_origins_are_symmetric():
    """The omni_wheel invocations must place wheels at all four sign
    combinations of (+/- wheel_x_offset, +/- wheel_y_offset)."""
    chassis_path = (
        _repo_root()
        / "src"
        / "fortis_description"
        / "urdf"
        / "fortis_chassis.urdf.xacro"
    )
    text = chassis_path.read_text()
    pattern = re.compile(
        r'<xacro:omni_wheel\s+name="([^"]+)"\s+'
        r'x_pos="\s*(-?)\$\{wheel_x_offset\}"\s+'
        r'y_pos="\s*(-?)\$\{wheel_y_offset\}"',
    )
    matches = pattern.findall(text)
    assert len(matches) == 4, (
        f"Expected 4 omni_wheel invocations, found {len(matches)}: {matches}"
    )
    sign_combos = {(x_sign, y_sign) for _name, x_sign, y_sign in matches}
    expected = {("", ""), ("", "-"), ("-", ""), ("-", "-")}
    assert sign_combos == expected, (
        f"Wheel origin sign combinations are not symmetric across "
        f"chassis center: got {sign_combos}, expected {expected}"
    )
