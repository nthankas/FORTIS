"""
Structural tests for the controller_manager YAML files.

These files are the single source of truth for which joint each index of
the Float64MultiArray published by fortis_drive maps to. If the joints
list re-orders silently, the robot drives diagonally. If the controller
type changes type silently, the controller_manager refuses to load.

The tests below parse the YAML directly (no controller_manager spin-up)
and assert on the structure the xacro + drive_node both depend on.
"""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def _yaml(name: str) -> dict:
    share = Path(get_package_share_directory("fortis_control"))
    p = share / "config" / name
    assert p.exists(), f"missing {p}; did `colcon build fortis_control` run?"
    return yaml.safe_load(p.read_text())


# --- Full chassis YAML -------------------------------------------------------


def test_full_chassis_yaml_loads():
    cfg = _yaml("fortis_drive_controllers.yaml")
    assert "controller_manager" in cfg
    assert "wheel_velocity_controller" in cfg


def test_full_chassis_yaml_joint_order_is_fl_fr_rl_rr():
    """
    The joint order in this list is the contract with
    fortis_drive.drive_node._wheel_command_to_controller_array, which
    publishes [fl, fr, rl, rr]. If the YAML re-orders, that mapping
    silently sends FR's velocity to FL.
    """
    cfg = _yaml("fortis_drive_controllers.yaml")
    joints = cfg["wheel_velocity_controller"]["ros__parameters"]["joints"]
    assert joints == [
        "fl_wheel_joint",
        "fr_wheel_joint",
        "rl_wheel_joint",
        "rr_wheel_joint",
    ], f"joint order drift: {joints}"


def test_full_chassis_yaml_controller_type_is_jointgroup_velocity():
    cfg = _yaml("fortis_drive_controllers.yaml")
    cm = cfg["controller_manager"]["ros__parameters"]
    wvc = cm["wheel_velocity_controller"]
    assert wvc["type"] == "velocity_controllers/JointGroupVelocityController"


def test_full_chassis_yaml_joint_state_broadcaster_type_correct():
    cfg = _yaml("fortis_drive_controllers.yaml")
    cm = cfg["controller_manager"]["ros__parameters"]
    jsb = cm["joint_state_broadcaster"]
    assert jsb["type"] == "joint_state_broadcaster/JointStateBroadcaster"


def test_full_chassis_yaml_interface_is_velocity():
    """
    Controller MUST claim velocity, not position or effort. The ODrive
    plugin auto-selects control mode from the claimed interface; if this
    drifts to "position", the S1 will refuse to spin under a velocity
    command.
    """
    cfg = _yaml("fortis_drive_controllers.yaml")
    iface = cfg["wheel_velocity_controller"]["ros__parameters"]["interface_name"]
    assert iface == "velocity"


def test_full_chassis_update_rate_is_at_least_50hz():
    """
    update_rate must comfortably exceed the ODrive cyclic encoder
    message rate (10 ms = 100 Hz). Going below 50 Hz means the
    controller is consuming joint state slower than the bus is
    publishing, and we lose all benefit of fast feedback.
    """
    cfg = _yaml("fortis_drive_controllers.yaml")
    rate = cfg["controller_manager"]["ros__parameters"]["update_rate"]
    assert rate >= 50, f"update_rate too low: {rate} Hz"


# --- Bench YAML --------------------------------------------------------------


def test_bench_yaml_loads():
    cfg = _yaml("fortis_drive_controllers_bench.yaml")
    assert "controller_manager" in cfg


def test_bench_yaml_lists_only_fl():
    """
    The bench variant lists exactly fl_wheel_joint. If a second joint
    leaks in, the controller_manager will refuse to start because the
    xacro (rendered with wheels:=fl) does not declare it.
    """
    cfg = _yaml("fortis_drive_controllers_bench.yaml")
    joints = cfg["wheel_velocity_controller"]["ros__parameters"]["joints"]
    assert joints == ["fl_wheel_joint"], f"bench joints drift: {joints}"


def test_bench_yaml_controller_type_matches_full():
    """Both YAMLs must use the same controller type; they only differ in joint list."""
    bench = _yaml("fortis_drive_controllers_bench.yaml")
    full = _yaml("fortis_drive_controllers.yaml")
    bench_cm = bench["controller_manager"]["ros__parameters"]
    full_cm = full["controller_manager"]["ros__parameters"]
    bench_type = bench_cm["wheel_velocity_controller"]["type"]
    full_type = full_cm["wheel_velocity_controller"]["type"]
    assert bench_type == full_type


def test_bench_yaml_interface_matches_full():
    bench = _yaml("fortis_drive_controllers_bench.yaml")
    full = _yaml("fortis_drive_controllers.yaml")
    b = bench["wheel_velocity_controller"]["ros__parameters"]["interface_name"]
    f = full["wheel_velocity_controller"]["ros__parameters"]["interface_name"]
    assert b == f == "velocity"
