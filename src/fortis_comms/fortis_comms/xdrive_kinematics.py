"""X-drive inverse and forward kinematics.

Geometry source of truth: src/fortis_description/urdf/fortis_constants.xacro,
specifically wheel_x_offset, wheel_y_offset, and wheel_radius. The
omni_wheel macro in fortis_chassis.urdf.xacro applies these at the four
sign combinations (+/-, +/-) about base_link. Values below mirror that
URDF, locked 2026-05-01 from the ROS_Expanded_Chassis CAD.

A drift regression in test/test_kinematics_urdf_sync.py enforces that
LEN_X, LEN_Y, and WHEEL_RADIUS continue to match the URDF; that test
fails the moment either side moves.

H-matrix sign convention: the rotation column applies (LEN_X + LEN_Y) as
a single effective lever arm. The per-wheel Vy and omega signs in the
matrix below are NOT re-derived from URDF wheel yaws as part of this
change; they are preserved verbatim from the senior-design module. See
README.md "Drift invariants" for the open question on convention.
"""

import numpy as np

# Wheel center positions in base_link frame, in meters. Mirror of:
#   wheel_x_offset = 0.176 m (6.93 in)   -- half longitudinal spread
#   wheel_y_offset = 0.125 m (4.92 in)   -- half lateral spread
# from fortis_description/urdf/fortis_constants.xacro.
LEN_X = 0.176
LEN_Y = 0.125

# AndyMark 8 in Dualie Omni (am-0463); mirror of wheel_radius in the URDF.
WHEEL_RADIUS = 0.1016  # 4.000 in

MAX_WHEEL_SPEED = 1  # m/s, saturator clamp at the contact patch

H = np.array([
    [1,  1,  (LEN_X + LEN_Y)],   # FL wheel
    [1, -1, -(LEN_X + LEN_Y)],   # FR wheel
    [1, -1,  (LEN_X + LEN_Y)],   # RL wheel
    [1,  1, -(LEN_X + LEN_Y)]    # RR wheel
])


def xdrive_ik_solver(Vx: float, Vy: float, omega: float) -> np.ndarray:
    """Solve for required wheel speeds given chassis Vx, Vy, omega."""
    wheel_speeds = H @ np.array([Vx, Vy, omega])
    # scale down if wheel speed exceeds 1
    scale = max(1.0, np.max(np.abs(wheel_speeds)) / MAX_WHEEL_SPEED)
    wheel_speeds /= scale
    return wheel_speeds


def xdrive_fk_solver(wheel_speeds) -> np.ndarray:
    return np.linalg.pinv(H) @ np.array(wheel_speeds)


def wheel_rot_to_lin_vel(omega: float) -> float:
    return omega * WHEEL_RADIUS
