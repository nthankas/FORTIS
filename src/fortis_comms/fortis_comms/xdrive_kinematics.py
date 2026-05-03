import numpy as np

IN_TO_M_CONV = 0.0254

WHEEL_RADIUS = 0.1016  # 4 inches in meters
LEN_X = 4.405 * IN_TO_M_CONV
LEN_Y = 6.462 * IN_TO_M_CONV
MAX_WHEEL_SPEED = 1  # m/s

H = np.array([
    [1,  1,  (LEN_X + LEN_Y)],   # FL wheel
    [1, -1, -(LEN_X + LEN_Y)],   # FR wheel
    [1, -1,  (LEN_X + LEN_Y)],   # BL wheel
    [1,  1, -(LEN_X + LEN_Y)]    # BR wheel
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
