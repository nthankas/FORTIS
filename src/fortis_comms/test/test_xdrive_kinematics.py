"""IK/FK round-trip sanity tests for the X-drive kinematics."""

import numpy as np
import pytest

from fortis_comms.xdrive_kinematics import (
    H,
    MAX_WHEEL_SPEED,
    xdrive_fk_solver,
    xdrive_ik_solver,
)


@pytest.mark.parametrize(
    "label, vx, vy, omega",
    [
        ("Pure forward", 1.0, 0.0, 0.0),
        ("Pure strafe", 0.0, 1.0, 0.0),
        ("Pure rotation", 0.0, 0.0, 1.0),
        ("Diagonal", 1.0, 1.0, 0.0),
        ("Combined", 1.0, 0.5, 0.3),
    ],
)
def test_ik_then_fk_round_trip(label, vx, vy, omega):
    """FK(IK(v)) recovers v, accounting for the IK saturator's scale-down."""
    ik = xdrive_ik_solver(vx, vy, omega)
    fk = xdrive_fk_solver(ik)

    expected = np.array([vx, vy, omega], dtype=float)
    raw_max = np.max(np.abs(H @ expected))
    scale = max(1.0, raw_max / MAX_WHEEL_SPEED)
    expected /= scale

    np.testing.assert_allclose(fk, expected, atol=1e-6, err_msg=label)
