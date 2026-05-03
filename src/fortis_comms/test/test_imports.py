"""
Smoke tests: every public module imports, and the basic APIs round-trip.

These tests catch packaging regressions (missing module, broken relative
import, missing package_data entry, ekf.py default-config not shipped)
without depending on any ROS runtime.
"""

import numpy as np
import pytest


def test_motor_base_imports():
    from fortis_comms.motor_base import Motor, MotorStatus  # noqa: F401
    # Motor is abstract; instantiation requires a concrete subclass.
    assert MotorStatus.MOTOR_DISCONNECTED.value == 2


def test_xdrive_kinematics_imports():
    from fortis_comms.xdrive_kinematics import (  # noqa: F401
        H,
        LEN_X,
        LEN_Y,
        MAX_WHEEL_SPEED,
        WHEEL_RADIUS,
        wheel_rot_to_lin_vel,
        xdrive_fk_solver,
        xdrive_ik_solver,
    )


def test_xdrive_ik_zero_command_returns_four_zeros():
    """IK on (0, 0, 0) must produce four zero wheel speeds."""
    from fortis_comms.xdrive_kinematics import xdrive_ik_solver

    result = xdrive_ik_solver(0.0, 0.0, 0.0)
    assert result.shape == (4,)
    np.testing.assert_array_almost_equal(result, np.zeros(4))


def test_xdrive_fk_zero_wheel_speeds_returns_three_zeros():
    """FK on four zero wheel speeds must yield (Vx, Vy, omega) = (0, 0, 0)."""
    from fortis_comms.xdrive_kinematics import xdrive_fk_solver

    result = xdrive_fk_solver(np.zeros(4))
    assert result.shape == (3,)
    np.testing.assert_array_almost_equal(result, np.zeros(3))


def test_ekf_imports_and_constructs_with_default_config():
    """EKF() loads bundled cfgs/ekf_params.json via importlib.resources."""
    from fortis_comms.ekf import EKF

    ekf = EKF()
    assert ekf.state.shape == (6,)
    np.testing.assert_array_equal(ekf.state, np.zeros(6))
    assert ekf.P.shape == (6, 6)
    assert ekf.Q.shape == (6, 6)
    assert ekf.R_of.shape == (2, 2)
    assert ekf.R_imu.shape == (3, 3)


def test_odrive_s1_imports():
    """ODrive_S1 imports cleanly. Requires python3-can (declared in package.xml)."""
    pytest.importorskip("can", reason="python3-can not installed; declared as rosdep")
    from fortis_comms.odrive_s1 import ODrive_S1  # noqa: F401
