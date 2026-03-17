"""
Shared config for FORTIS X-drive simulations.
Measurements from CAD (OnShape) and Rotacaster specs. Units: meters (SI).
"""
import os
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Active reactor USD -- has triangle mesh collision applied via GUI
REACTOR_SIM_USD = os.path.join(BASE_DIR, "D-IIIDTokamak_sim.usd")

# ---- Wheel (AndyMark 8" Dualie Plastic am-0463, 203mm) ----
# Simulation still uses 180mm (Rotacaster) geometry for the sphere rollers.
# Real wheel is 203mm -- 12.8% larger radius, within torque margins.
WHEEL_DIAMETER_MM = 180.0
WHEEL_RADIUS_M = WHEEL_DIAMETER_MM / 2.0 / 1000.0  # 0.09 m
WHEEL_WIDTH_M = 0.055  # ~55mm estimated from CAD bbox
WHEEL_MASS_KG = 1.0
OMNI_ROLLER_ANGLE_DEG = 90.0  # 90 for omni, 45 for mecanum

# ---- Active chassis: 15x9x6" rectangular octagonal prism ----
# Built from USD primitives in the sim scripts (not from CAD -- OnShape CAD
# has internal gearbox joints that create closed articulation loops in PhysX).

# ---- X-Drive wheel orientations ----
# Each wheel's drive axis is at 45 degrees to the chassis forward (+X) axis.
# The spin axis direction for each wheel (unit vectors in chassis frame, horizontal):
#   FL: forward is toward (+X,+Y) diagonal -> spin axis perpendicular = (+0.707, -0.707, 0)
#   FR: forward is toward (+X,-Y) diagonal -> spin axis perpendicular = (+0.707, +0.707, 0)
#   BL: spin axis parallel to FR = (+0.707, +0.707, 0)
#   BR: spin axis parallel to FL = (+0.707, -0.707, 0)
# Note: "spin axis" here is the revolute joint axis (the axle direction).
XDRIVE_WHEEL_AXES = np.array([
    [ 0.7071, -0.7071, 0.0],  # FL axle direction
    [ 0.7071,  0.7071, 0.0],  # FR axle direction
    [ 0.7071,  0.7071, 0.0],  # BL axle direction (parallel to FR)
    [ 0.7071, -0.7071, 0.0],  # BR axle direction (parallel to FL)
])

# Wheel angle relative to chassis forward (+X), measured CCW from +X
XDRIVE_WHEEL_ANGLES_DEG = np.array([45.0, -45.0, -45.0, 45.0])  # FL, FR, BL, BR
WHEEL_NAMES = ["FL", "FR", "BL", "BR"]

# ---- Mass ----
MASS_VALUES_KG = [13.6, 18.1, 22.7]  # 30, 40, 50 lbs
DEFAULT_MASS_KG = 18.1  # 40 lbs

# ---- Friction ----
GRAPHITE_STATIC_FRICTION = 0.5
GRAPHITE_DYNAMIC_FRICTION = 0.5
WHEEL_DRIVE_FRICTION = 0.8    # along drive direction
WHEEL_SLIDE_FRICTION = 0.05   # along roller direction
WHEEL_COMBINED_FRICTION = 0.5  # isotropic fallback if anisotropic not available

# ---- Physics ----
PHYSICS_DT = 1.0 / 120.0
PHYSICS_DT_TIER2 = 1.0 / 240.0

# ---- Drive ----
MANUAL_DRIVE_SPEED = 0.2   # m/s per key press
MANUAL_ROTATE_SPEED = 0.3  # rad/s per key press
MAX_LINEAR_SPEED = 0.5     # m/s
MAX_ANGULAR_SPEED = 0.5    # rad/s
MAX_WHEEL_SPEED = 50.0     # rad/s
SPEED_INCREMENT = 0.05     # m/s per +/- press

# ---- Strafe (Phase 2) ----
STRAFE_SPEEDS = [0.1, 0.2, 0.3, 0.5]
DEFAULT_STRAFE_SPEED = 0.2

# ---- Reactor geometry (from Phase 1 STL analysis, inches) ----
Z_INNER_IN = -53.8
Z_OUTER_IN = -49.3
STEP_HEIGHT_IN = Z_OUTER_IN - Z_INNER_IN  # 4.5"
STEP_R_IN = 55.5
R_INNER_FLOOR_MIN_IN = 45.8
R_INNER_FLOOR_MAX_IN = 54.8
R_OUTER_FLOOR_MIN_IN = 56.2
R_OUTER_FLOOR_MAX_IN = 70.3
R_CENTER_POST_IN = 38.0

IN_TO_M = 0.0254

# ---- Joint drive parameters ----
WHEEL_JOINT_STIFFNESS = 0.0
WHEEL_JOINT_DAMPING = 10.0
WHEEL_JOINT_MAX_FORCE = 50.0  # Nm
