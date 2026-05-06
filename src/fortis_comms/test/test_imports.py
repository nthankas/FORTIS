"""
Smoke tests: every public module imports, and the basic APIs round-trip.

These tests catch packaging regressions (missing module, broken relative
import, missing package_data entry) without depending on any ROS runtime.

Note: motor_base, odrive_s1, and ekf were moved to ``legacy/`` and are
no longer imported by the build. Replaced by ros2_control + standard
ROS 2 packages.
"""

import numpy as np


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
