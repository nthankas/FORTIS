"""
Functional tests for the FORTIS chassis URDF + ros2_control xacro.

These run xacro against fortis_robot.urdf.xacro for every combination of
the new ros2_control args and assert on the resulting XML. They catch:

  * Typos in the <param name="node_id"> blocks (silently swapped wheels
    on the bus would be very hard to diagnose later).
  * Drift between the xacro args and the YAML's `joints:` list (wrong
    motor responds to wrong wheel).
  * Mock-hardware fallback breaking (CI runs with mock; if it stops
    swapping the plugin, CI silently exercises nothing).
  * check_urdf failure (any malformed URDF the change introduces).

The tests do NOT spin up controller_manager. That is covered separately
by test_bench_launch.py.
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory


def _xacro_path() -> str:
    """Resolve the installed share/ path of the top-level chassis xacro."""
    share = Path(get_package_share_directory("fortis_description"))
    p = share / "urdf" / "fortis_robot.urdf.xacro"
    assert p.exists(), f"missing {p}; did `colcon build fortis_description` run?"
    return str(p)


def _run_xacro(**args: str) -> str:
    """
    Expand the top-level URDF xacro with the given mappings.

    Equivalent to:
        xacro fortis_robot.urdf.xacro KEY:=VAL KEY:=VAL ...

    Returns the rendered URDF XML as a string. Raises CalledProcessError
    with stderr surfaced so a xacro syntax error is loud.
    """
    if shutil.which("xacro") is None:
        pytest.skip("xacro not on PATH (ROS not sourced?)")
    cmd = ["xacro", _xacro_path()] + [f"{k}:={v}" for k, v in args.items()]
    result = subprocess.run(
        cmd, capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, (
        f"xacro failed (exit {result.returncode}):\n"
        f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )
    return result.stdout


def _check_urdf(urdf_xml: str, tmp_path: Path) -> None:
    """Run check_urdf on the expanded URDF; fail noisily if it rejects it."""
    if shutil.which("check_urdf") is None:
        pytest.skip("check_urdf not on PATH")
    urdf_file = tmp_path / "expanded.urdf"
    urdf_file.write_text(urdf_xml)
    result = subprocess.run(
        ["check_urdf", str(urdf_file)],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 0, (
        f"check_urdf rejected the expanded URDF:\n"
        f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )


# --- Backwards-compat: display.launch.py path --------------------------------


def test_default_args_omit_ros2_control_block(tmp_path):
    """
    With enable_ros2_control:=false (the default), no <ros2_control>
    tag is rendered. This preserves the legacy display.launch.py /
    RViz-only workflow that pre-dates this change.
    """
    urdf = _run_xacro()
    root = ET.fromstring(urdf)
    assert root.find("ros2_control") is None, (
        "default xacro expansion must not contain a <ros2_control> block; "
        "display.launch.py depends on this"
    )
    _check_urdf(urdf, tmp_path)


def test_default_args_still_contain_chassis_wheels(tmp_path):
    """Smoke: the default expansion is still a complete chassis URDF."""
    urdf = _run_xacro()
    root = ET.fromstring(urdf)
    joint_names = {j.get("name") for j in root.findall("joint")}
    # All four wheel joints must exist as kinematic joints regardless
    # of whether ros2_control is enabled.
    assert {"fl_wheel_joint", "fr_wheel_joint",
            "rl_wheel_joint", "rr_wheel_joint"} <= joint_names


# --- Bench variant: one wheel, real plugin -----------------------------------


def test_bench_single_wheel_renders_only_fl(tmp_path):
    """
    With wheels:=fl, the <ros2_control> block contains exactly one
    <joint name="fl_wheel_joint"> and no other wheel joints.

    This is the contract with fortis_drive_controllers_bench.yaml's
    `joints: [fl_wheel_joint]`. A drift here would either fail to start
    controller_manager ("joint not declared in URDF") or silently load
    a controller against an unbound joint.
    """
    urdf = _run_xacro(
        enable_ros2_control="true",
        wheels="fl",
        can_interface="can0",
        use_mock_hardware="false",
    )
    root = ET.fromstring(urdf)
    r2c = root.find("ros2_control")
    assert r2c is not None
    joint_names = [j.get("name") for j in r2c.findall("joint")]
    assert joint_names == ["fl_wheel_joint"], (
        f"bench variant must wire exactly fl_wheel_joint; got {joint_names}"
    )

    # node_id assignment for FL is 0; if this drifts, the wrong S1 will
    # answer.
    fl = r2c.find("joint")
    node_id = fl.find("param[@name='node_id']").text
    assert node_id == "0"

    # Plugin must be the real ODrive interface, not the mock — the
    # test name says "real plugin".
    plugin = r2c.find("hardware/plugin").text
    assert plugin == "odrive_ros2_control_plugin/ODriveHardwareInterface"

    # CAN interface param threaded through from the launch arg.
    can_param = r2c.find("hardware/param[@name='can']").text
    assert can_param == "can0"

    _check_urdf(urdf, tmp_path)


# --- Full variant: four wheels, real plugin ----------------------------------


def test_full_chassis_renders_all_four_wheels_with_correct_node_ids(tmp_path):
    """
    With wheels:=fl,fr,rl,rr, the <ros2_control> block contains all four
    wheel joints with node_ids assigned per the CAN daisy-chain order
    FL → FR → RR → RL (FL=0, FR=1, RR=2, RL=3).

    The node_id-to-wheel binding is the entire safety surface for "which
    motor moves when I command which wheel velocity." If this regresses,
    the robot moves diagonally when commanded forward.
    """
    urdf = _run_xacro(
        enable_ros2_control="true",
        wheels="fl,fr,rl,rr",
        can_interface="can0",
        use_mock_hardware="false",
    )
    root = ET.fromstring(urdf)
    r2c = root.find("ros2_control")
    assert r2c is not None

    by_name = {
        j.get("name"): j.find("param[@name='node_id']").text
        for j in r2c.findall("joint")
    }
    assert by_name == {
        "fl_wheel_joint": "0",
        "fr_wheel_joint": "1",
        "rr_wheel_joint": "2",
        "rl_wheel_joint": "3",
    }, f"node_id mapping drift: {by_name}"

    _check_urdf(urdf, tmp_path)


@pytest.mark.parametrize(
    "wheel,joint_name,expected_node_id",
    [
        ("fl", "fl_wheel_joint", "0"),
        ("fr", "fr_wheel_joint", "1"),
        ("rr", "rr_wheel_joint", "2"),
        ("rl", "rl_wheel_joint", "3"),
    ],
)
def test_each_wheel_can_be_loaded_alone(wheel, joint_name, expected_node_id):
    """
    Each wheel can be loaded on its own. Lets the bench operator
    move from FL to FR/RL/RR without editing the xacro itself; only
    the launch arg changes.
    """
    urdf = _run_xacro(
        enable_ros2_control="true",
        wheels=wheel,
    )
    root = ET.fromstring(urdf)
    r2c = root.find("ros2_control")
    joints = r2c.findall("joint")
    assert len(joints) == 1
    assert joints[0].get("name") == joint_name
    assert joints[0].find("param[@name='node_id']").text == expected_node_id


# --- Mock-hardware fallback (CI / no-CAN dev path) ---------------------------


def test_mock_hardware_swaps_plugin(tmp_path):
    """
    With use_mock_hardware:=true, the plugin is mock_components/GenericSystem,
    NOT the ODrive plugin. This is the path CI uses; if it ever silently
    falls back to the real plugin, CI starts failing on machines without
    SocketCAN — or worse, hangs trying to talk to a bus that does not exist.
    """
    urdf = _run_xacro(
        enable_ros2_control="true",
        wheels="fl",
        use_mock_hardware="true",
    )
    root = ET.fromstring(urdf)
    r2c = root.find("ros2_control")
    assert r2c is not None
    plugin = r2c.find("hardware/plugin").text
    assert plugin == "mock_components/GenericSystem", (
        f"mock fallback must swap plugin; got {plugin}"
    )
    # And the calculate_dynamics param must be present so mock state
    # mirrors commanded velocity — without it, the launch_testing
    # integration test has nothing to assert against.
    calc = r2c.find("hardware/param[@name='calculate_dynamics']").text
    assert calc == "true"
    _check_urdf(urdf, tmp_path)


def test_mock_hardware_full_chassis_renders_all_four_wheels(tmp_path):
    """The CI path must also support the 4-wheel variant under mock."""
    urdf = _run_xacro(
        enable_ros2_control="true",
        wheels="fl,fr,rl,rr",
        use_mock_hardware="true",
    )
    root = ET.fromstring(urdf)
    r2c = root.find("ros2_control")
    assert len(r2c.findall("joint")) == 4
    plugin = r2c.find("hardware/plugin").text
    assert plugin == "mock_components/GenericSystem"
    _check_urdf(urdf, tmp_path)


# --- Command/state interface contract ----------------------------------------


def test_each_wheel_exposes_velocity_command_and_pve_state(tmp_path):
    """
    Every wheel joint must declare:
      command_interface: velocity
      state_interface:   position, velocity, effort

    velocity is what odrive_ros2_control's auto-mode selection uses to
    pick velocity control mode on the S1. Missing state interfaces
    cause /joint_states to be incomplete; that breaks robot_localization
    downstream.
    """
    urdf = _run_xacro(enable_ros2_control="true", wheels="fl,fr,rl,rr")
    root = ET.fromstring(urdf)
    r2c = root.find("ros2_control")

    for joint in r2c.findall("joint"):
        cmd_ifaces = {ci.get("name") for ci in joint.findall("command_interface")}
        state_ifaces = {si.get("name") for si in joint.findall("state_interface")}
        assert cmd_ifaces == {"velocity"}, (
            f"{joint.get('name')} command_interfaces drift: {cmd_ifaces}"
        )
        assert state_ifaces == {"position", "velocity", "effort"}, (
            f"{joint.get('name')} state_interfaces drift: {state_ifaces}"
        )
