"""
Analytical IK/FK, gravity torques, and collision checks for the FORTIS 4-DOF arm.

V3 HARDWARE SPEC (2026-05-03) -- single 30" carbon-fiber build, no variants.

  J1 (chassis yaw)    : NEMA 17 + Cricket MK II 25:1                      (0.580 kg)
  J2 (shoulder)       : NEMA 23 + EG23-G20-D10 20:1 + adapter             (2.665 kg)
  J3 (elbow)          : NEMA 17 (590 g) + Cricket gearbox (80 g)
                        + 0.593 lb AL connector (269 g)                    (0.939 kg)
  J4 (wrist)          : Hitec D845WP servo + metal mount
                        (227 g + 0.709 lb)                                 (0.5486 kg)
  Links               : CF square hollow tube, 1.128"x1.128" OD,
                        1.000"x1.000" ID, 0.064" wall.
                        Linear density 0.01431 lb/in (0.2557 kg/m).
  L2 (J2 -> J3)       : 14.168 in  (91.9 g)
  L3 (J3 -> J4)       : 11.222 in  (72.8 g)
  L4 (J4 -> EE tip)   :  4.97 in   (32.3 g)
  Camera              : OAK-D Pro (91 g) on L4 midpoint, 1.5" from J4
  End effector        : ServoCity parallel kit + D645MW (133 g total)
  Loaded payload      : 3 lb (1361 g) at L4 tip (always loaded in v3)

This module is the v3 counterpart to arm_ik_v2.py. Callers should import
arm_ik_v3 directly when they want the v3 spec; v2 remains untouched.

No variant flags: 30 inch CF, loaded, is the only configuration.

Kinematic chain identical to v2: J1 (Z yaw) + J2/J3/J4 (Y pitch),
yaw-decoupled planar in the vertical plane spanned by J1.
FK convention: standard 3R planar.
  x_j3 = L2*cos(A2),  z_j3 = L2*sin(A2)
  x_j4 = x_j3 + L3*cos(A2+A3),  ...

No Isaac Sim dependency. Pure Python + numpy.
"""

import math
import numpy as np

# ==========================================================================
# Constants
# ==========================================================================
IN = 0.0254  # inches to meters
GRAV = 9.81

# ----------------------------------------------------------------------
# Per-joint masses (V3 hardware, kg)
# ----------------------------------------------------------------------
M_J1 = 0.580   # NEMA 17 (500 g) + Cricket MK II 25:1 (80 g) -- unchanged from v2
M_J2 = 2.665   # NEMA 23 + EG23-G20-D10 20:1 + adapter -- unchanged from v2
M_J3 = 0.939   # NEMA 17 (590 g) + Cricket gearbox (80 g) + AL connector 0.593 lb (269 g)
M_J4 = 0.5486  # Hitec D845WP + metal mount (227 g + 0.709 lb)

# Legacy single-joint alias (some callers reference this).
M_JOINT = M_J1

# End-effector and instrumentation
# 30" CF v3: lighter EE assembly (133 g), payload always present.
M_GRIPPER_BARE = 0.133    # ServoCity parallel kit + D645MW (lighter assembly than v2's 216 g)
M_PAYLOAD = 1.361         # 3 lb max payload (always loaded in v3)
M_CAMERA = 0.091          # OAK-D Pro on L4 midpoint
CAM_OFFSET_ON_L4 = 1.5 * IN   # camera position along L4 from J4

# Torque limits (Nm) -- continuous ratings per joint
TORQUE_LIMITS_NM = {
    "J1": 12.0,   # NEMA 17 + Cricket 25:1
    "J2": 30.0,   # EG23-G20 20:1 continuous (60 Nm peak)
    "J3": 12.0,   # Cricket MK II 25:1 continuous
    "J4": 4.9,    # D845WP at 7.4 V
}

# Joint ranges (V3 hardware reality):
#   J1, J2, J3 are continuous-rotation steppers with gearboxes. Software
#   ranges below are set to a full revolution; sweep code can use any subset
#   without hitting a hard limit.
#   J4 is a Hitec D845WP servo: 202 deg total travel = +/-101 deg.
JOINT_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),   # continuous (NEMA 17 + Cricket 25:1)
    "ArmJ2": (-180.0, 180.0),   # continuous (NEMA 23 + EG23 20:1)
    "ArmJ3": (-180.0, 180.0),   # continuous (NEMA 17 + Cricket 25:1)
    "ArmJ4": (-101.0, 101.0),   # servo hardware limit (D845WP, 202 deg total)
}
JOINT_CONTINUOUS = {"ArmJ1": True, "ArmJ2": True, "ArmJ3": True, "ArmJ4": False}

# ----------------------------------------------------------------------
# CF tube spec (square hollow, rounded corners)
# ----------------------------------------------------------------------
# 1.128" x 1.128" OD square, 1.000" x 1.000" ID, 0.064" wall
# Mass: 1.03 lb / 72 in => 0.01431 lb/in => 0.2557 kg/m
CF_DENSITY_LB_PER_IN = 1.03 / 72.0           # 0.01431 lb/in
CF_DENSITY_KG_PER_M = CF_DENSITY_LB_PER_IN * 0.453592 / IN  # ~0.2557 kg/m

# Cross-section (used for inertia + collision envelope)
CF_TUBE_OD = 1.128 * IN   # square OD per side
CF_TUBE_ID = 1.000 * IN   # square ID per side
CF_TUBE_WALL = 0.064 * IN
CF_TUBE_R_OUT_CORNER = 0.189 * IN
CF_TUBE_R_IN_CORNER = 0.125 * IN

# For inertia/collision the tube is treated as a square box of side CF_TUBE_OD.
# Y/Z aliases match v2 callers that expect rectangular cross-section dims.
CF_TUBE_Y = CF_TUBE_OD
CF_TUBE_Z = CF_TUBE_OD

# ----------------------------------------------------------------------
# Link lengths (V3, fixed)
# ----------------------------------------------------------------------
L_L2_IN = 14.168
L_L3_IN = 11.222
L_L4_IN = 4.970

L_L2 = L_L2_IN * IN
L_L3 = L_L3_IN * IN
L_L4 = L_L4_IN * IN

M_L2 = CF_DENSITY_KG_PER_M * L_L2
M_L3 = CF_DENSITY_KG_PER_M * L_L3
M_L4 = CF_DENSITY_KG_PER_M * L_L4

# ----------------------------------------------------------------------
# Chassis geometry (matches xdrive_realwheel.py / arm_ik_v2.py)
# ----------------------------------------------------------------------
CHASSIS_L = 13.082 * IN
CHASSIS_W = 8.54 * IN
CHASSIS_H = 6.0 * IN

# Arm mount: 3" forward from back edge, centerline, flush on top
ARM_MOUNT_X = -CHASSIS_L / 2.0 + 3.0 * IN
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0

J1_STACK_H = 1.5 * IN

# Wheel positions (rectangular bounding box, wheels flush at corners)
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
WHEEL_XY = {
    "FR": ( SL, -SW),
    "FL": ( SL,  SW),
    "BL": (-SL,  SW),
    "BR": (-SL, -SW),
}
WHEEL_RADIUS = 203.0 / 2.0 / 1000.0  # 0.1015 m (AndyMark 8" Dualie)
CHASSIS_MASS = 20.4
BELLY_HEIGHT = 2.0 * IN


# ==========================================================================
# Arm config helper (single config in v3 -- no variants, no args)
# ==========================================================================
def get_arm_params():
    """Return arm parameters for the v3 30" CF loaded build."""
    M_gripper = M_GRIPPER_BARE + M_PAYLOAD  # always loaded
    total_arm = (M_J1 + M_J2 + M_J3 + M_J4
                 + M_L2 + M_L3 + M_L4
                 + M_CAMERA + M_gripper)
    return {
        "build": "v3_30in_CF_loaded",
        "L2": L_L2, "L3": L_L3, "L4": L_L4,
        "L2_in": L_L2_IN, "L3_in": L_L3_IN, "L4_in": L_L4_IN,
        "M_L2": M_L2, "M_L3": M_L3, "M_L4": M_L4,
        "M_J1": M_J1, "M_J2": M_J2, "M_J3": M_J3, "M_J4": M_J4,
        "M_joint": M_J1,                # legacy alias
        "M_gripper": M_gripper,         # gripper + payload
        "M_gripper_bare": M_GRIPPER_BARE,
        "M_payload": M_PAYLOAD,
        "M_camera": M_CAMERA,
        "cam_offset_L4": CAM_OFFSET_ON_L4,
        "max_reach": L_L2 + L_L3 + L_L4,
        "total_arm_mass": total_arm,
        "cf_tube_y": CF_TUBE_Y,
        "cf_tube_z": CF_TUBE_Z,
        "cf_density_kg_per_m": CF_DENSITY_KG_PER_M,
    }


# ==========================================================================
# Forward Kinematics (identical math to v2; camera location updated)
# ==========================================================================
def fk_planar(j2_rad, j3_rad, j4_rad, L2=L_L2, L3=L_L3, L4=L_L4):
    """Standard 3R planar FK. Positions relative to J2 pivot."""
    A2 = j2_rad
    A23 = j2_rad + j3_rad
    A234 = j2_rad + j3_rad + j4_rad

    x_j3 = L2 * math.cos(A2)
    z_j3 = L2 * math.sin(A2)
    x_j4 = x_j3 + L3 * math.cos(A23)
    z_j4 = z_j3 + L3 * math.sin(A23)
    x_ee = x_j4 + L4 * math.cos(A234)
    z_ee = z_j4 + L4 * math.sin(A234)

    x_L2_mid = 0.5 * L2 * math.cos(A2)
    z_L2_mid = 0.5 * L2 * math.sin(A2)
    x_L3_mid = x_j3 + 0.5 * L3 * math.cos(A23)
    z_L3_mid = z_j3 + 0.5 * L3 * math.sin(A23)
    x_L4_mid = x_j4 + 0.5 * L4 * math.cos(A234)
    z_L4_mid = z_j4 + 0.5 * L4 * math.sin(A234)

    x_cam = x_j4 + CAM_OFFSET_ON_L4 * math.cos(A234)
    z_cam = z_j4 + CAM_OFFSET_ON_L4 * math.sin(A234)

    return {
        "j3": (x_j3, z_j3),
        "j4": (x_j4, z_j4),
        "ee": (x_ee, z_ee),
        "L2_mid": (x_L2_mid, z_L2_mid),
        "L3_mid": (x_L3_mid, z_L3_mid),
        "L4_mid": (x_L4_mid, z_L4_mid),
        "cam": (x_cam, z_cam),
        "A2": A2, "A23": A23, "A234": A234,
    }


def fk_planar_vec(j2, j3, j4, L2=L_L2, L3=L_L3, L4=L_L4):
    A2 = j2
    A23 = j2 + j3
    A234 = j2 + j3 + j4

    x_j3 = L2 * np.cos(A2)
    z_j3 = L2 * np.sin(A2)
    x_j4 = x_j3 + L3 * np.cos(A23)
    z_j4 = z_j3 + L3 * np.sin(A23)
    x_ee = x_j4 + L4 * np.cos(A234)
    z_ee = z_j4 + L4 * np.sin(A234)

    return {
        "j3": np.column_stack([x_j3, z_j3]),
        "j4": np.column_stack([x_j4, z_j4]),
        "ee": np.column_stack([x_ee, z_ee]),
    }


# ==========================================================================
# Inverse Kinematics
# ==========================================================================
def ik_planar(target_r, target_z, ee_pitch_rad, L2=L_L2, L3=L_L3, L4=L_L4):
    """Analytical IK for the 3R planar arm. Returns up to 2 solutions."""
    wx = target_r - L4 * math.cos(ee_pitch_rad)
    wz = target_z - L4 * math.sin(ee_pitch_rad)

    d_sq = wx * wx + wz * wz
    d = math.sqrt(d_sq)
    if d > L2 + L3 or d < abs(L2 - L3):
        return []
    if d < 1e-10:
        return []

    cos_j3 = (d_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    cos_j3 = max(-1.0, min(1.0, cos_j3))

    solutions = []
    for sign in [1.0, -1.0]:
        j3 = sign * math.acos(cos_j3)
        beta = math.atan2(L3 * math.sin(j3), L2 + L3 * math.cos(j3))
        j2 = math.atan2(wz, wx) - beta
        j4 = ee_pitch_rad - j2 - j3
        solutions.append({"j2": j2, "j3": j3, "j4": j4})

    return solutions


def ik_3d(target_x, target_y, target_z, ee_pitch_rad, arm_params,
          z_j2_above_floor=None):
    """Full 3D IK: J1 yaw + 3R planar."""
    L2 = arm_params["L2"]
    L3 = arm_params["L3"]
    L4 = arm_params["L4"]

    dx = target_x - ARM_MOUNT_X
    dy = target_y - ARM_MOUNT_Y
    r = math.sqrt(dx * dx + dy * dy)
    j1 = math.atan2(dy, dx)

    if z_j2_above_floor is not None:
        pz = target_z
    else:
        pz = target_z - (ARM_MOUNT_Z + J1_STACK_H)

    planar = ik_planar(r, pz, ee_pitch_rad, L2, L3, L4)
    return [{"j1": j1, **sol} for sol in planar]


# ==========================================================================
# Gravity Torques (analytical)
# ==========================================================================
def gravity_torques(j2_rad, j3_rad, j4_rad, arm_params):
    """Static gravity torques at J2/J3/J4 with v3 per-joint masses."""
    L2 = arm_params["L2"]
    L3 = arm_params["L3"]
    L4 = arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    M_J3_ = arm_params["M_J3"]
    M_J4_ = arm_params["M_J4"]
    M_L2_ = arm_params["M_L2"]
    M_L3_ = arm_params["M_L3"]
    M_L4_ = arm_params["M_L4"]
    M_grip = arm_params["M_gripper"]
    M_cam = arm_params["M_camera"]

    x_j3, _ = fk["j3"]
    x_j4, _ = fk["j4"]
    x_ee, _ = fk["ee"]
    x_L2m, _ = fk["L2_mid"]
    x_L3m, _ = fk["L3_mid"]
    x_L4m, _ = fk["L4_mid"]
    x_cam, _ = fk["cam"]

    tau_j2 = -GRAV * (
        M_L2_ * x_L2m +
        M_J3_ * x_j3 +
        M_L3_ * x_L3m +
        M_J4_ * x_j4 +
        M_L4_ * x_L4m +
        M_cam * x_cam +
        M_grip * x_ee
    )

    tau_j3 = -GRAV * (
        M_L3_ * (x_L3m - x_j3) +
        M_J4_ * (x_j4 - x_j3) +
        M_L4_ * (x_L4m - x_j3) +
        M_cam * (x_cam - x_j3) +
        M_grip * (x_ee - x_j3)
    )

    tau_j4 = -GRAV * (
        M_L4_ * (x_L4m - x_j4) +
        M_cam * (x_cam - x_j4) +
        M_grip * (x_ee - x_j4)
    )

    return {"tau_j2": tau_j2, "tau_j3": tau_j3, "tau_j4": tau_j4}


def gravity_torque_j1(j1_rad, j2_rad, j3_rad, j4_rad, arm_params):
    """J1 axis is vertical; static gravity creates no J1 torque."""
    return 0.0


def j1_inertia_about_axis(j2_rad, j3_rad, j4_rad, arm_params):
    """Moment of inertia of the arm (everything outboard of J1) about
    the vertical J1 axis at the arm-mount X position.

    All point masses contribute m * r^2, where r is the horizontal radial
    distance from the J1 axis. The arm bodies are arranged in a vertical
    plane that pivots around J1; their radial distance is the FK x-value
    (which is signed but enters as r^2, so sign doesn't matter).

    Returns I_zz in kg*m^2.
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    # Each labeled point mass at radial distance |fk[label][0]|
    parts = [
        ("L2_mid", arm_params["M_L2"]),
        ("j3",     arm_params["M_J3"]),
        ("L3_mid", arm_params["M_L3"]),
        ("j4",     arm_params["M_J4"]),
        ("L4_mid", arm_params["M_L4"]),
        ("cam",    arm_params["M_camera"]),
        ("ee",     arm_params["M_gripper"]),
    ]
    I_zz = 0.0
    for label, mass in parts:
        rx = fk[label][0]
        I_zz += mass * (rx * rx)
    return I_zz


def j1_inertial_torque(j2_rad, j3_rad, j4_rad, arm_params,
                       angular_accel_rad_s2):
    """J1 torque needed to angularly accelerate the arm at a given alpha.

    T_j1 = I_j1 * alpha. Useful for sizing J1 against slewing requirements
    (which dominates J1 loading since gravity torque is zero by axis
    geometry).
    """
    return j1_inertia_about_axis(j2_rad, j3_rad, j4_rad, arm_params) * \
        angular_accel_rad_s2


# ==========================================================================
# Collision Checks (same logic as v2; uses v3 tube cross-section)
# ==========================================================================
def _segment_in_box(p1x, p1z, p2x, p2z, box_x_min, box_x_max,
                    box_z_min, box_z_max, n_samples=5):
    for t in np.linspace(0.0, 1.0, n_samples):
        px = p1x + t * (p2x - p1x)
        pz = p1z + t * (p2z - p1z)
        if box_x_min < px < box_x_max and box_z_min < pz < box_z_max:
            return True
    return False


def _segment_distance(seg1, seg2):
    (ax1, az1), (ax2, az2) = seg1
    (bx1, bz1), (bx2, bz2) = seg2
    min_d = float('inf')
    for s in np.linspace(0, 1, 12):
        px = ax1 + s * (ax2 - ax1)
        pz = az1 + s * (az2 - az1)
        for t in np.linspace(0, 1, 12):
            qx = bx1 + t * (bx2 - bx1)
            qz = bz1 + t * (bz2 - bz1)
            d = math.sqrt((px - qx) ** 2 + (pz - qz) ** 2)
            if d < min_d:
                min_d = d
    return min_d


def check_collisions(j2_rad, j3_rad, j4_rad, arm_params, z_j2_above_floor,
                     margin=0.025):
    """Check analytical collisions for a planar arm pose."""
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    LINK_HALF_THICK = max(CF_TUBE_Y, CF_TUBE_Z) / 2.0 + 0.005

    floor_margin = margin + LINK_HALF_THICK
    for label in ["j3", "j4", "ee", "L2_mid", "L3_mid", "L4_mid"]:
        _, z = fk[label]
        if z + z_j2_above_floor < floor_margin:
            return {"valid": False, "reason": "floor"}

    x_expand = margin + LINK_HALF_THICK
    z_down_expand = margin + LINK_HALF_THICK
    z_up_expand = margin
    chassis_x_min = -x_expand
    chassis_x_max = CHASSIS_L + x_expand
    chassis_z_min = -(CHASSIS_H + J1_STACK_H) - z_down_expand
    chassis_z_max = -J1_STACK_H + z_up_expand

    segments = [
        ((0, 0), fk["j3"]),
        (fk["j3"], fk["j4"]),
        (fk["j4"], fk["ee"]),
    ]
    for seg_idx, ((p1x, p1z), (p2x, p2z)) in enumerate(segments):
        t_start = 0.08 if seg_idx == 0 else 0.0
        for t in np.linspace(t_start, 1.0, 10):
            px = p1x + t * (p2x - p1x)
            pz = p1z + t * (p2z - p1z)
            if chassis_x_min < px < chassis_x_max and chassis_z_min < pz < chassis_z_max:
                return {"valid": False, "reason": "chassis"}

    L4_seg = (fk["j4"], fk["ee"])
    L2_seg = ((0, 0), fk["j3"])
    if _segment_distance(L4_seg, L2_seg) < 2 * LINK_HALF_THICK:
        return {"valid": False, "reason": "self_L4_L2"}

    j1_half = 0.030
    for t in np.linspace(0, 1.0, 8):
        px = fk["j4"][0] + t * (fk["ee"][0] - fk["j4"][0])
        pz = fk["j4"][1] + t * (fk["ee"][1] - fk["j4"][1])
        if abs(px) < j1_half + LINK_HALF_THICK and -J1_STACK_H < pz < 0:
            return {"valid": False, "reason": "self_L4_J1"}

    return {"valid": True, "reason": None}


# ==========================================================================
# Tipping analysis (V3): combined CG of (chassis + 4 wheels + J1 + J2 +
# arm bodies that actually move) versus the wheel support polygon.
#
# v2 had two bugs in this calculation: J1 was lumped onto the arm side
# (it doesn't move during a J2/J3/J4 sweep -- it's part of the chassis
# tower), J2 mass was missing entirely, and the wheel mass was not
# counted. v3 fixes all three. Result: more conservative tipping margins
# (heavier inboard side pulls CG back toward chassis center, generally
# improving margins, but the math now reflects reality).
# ==========================================================================
WHEELS_TOTAL_MASS = 4.0 * 1.0   # 4x AndyMark 8" Dualie hubs (+rollers) ~1 kg each


def _chassis_side_mass_and_cg():
    """Return (M_static, x_static_weighted, y_static_weighted) for everything
    that does NOT move when only J2/J3/J4 change: chassis body, 4 wheels,
    J1 motor stack, J2 motor stack.

    Returns components so callers can fold the moving arm CG into a single
    weighted average without recomputing the static side.
    """
    # Chassis body at origin
    M_chassis = CHASSIS_MASS
    cx_chassis = 0.0
    cy_chassis = 0.0

    # 4 wheels at corners. Symmetric -> x_w*M_w sum = 0, y_w*M_w sum = 0
    M_wheels = WHEELS_TOTAL_MASS
    cx_wheels = 0.0
    cy_wheels = 0.0
    for (wx, wy) in WHEEL_XY.values():
        cx_wheels += (1.0) * wx
        cy_wheels += (1.0) * wy
    # Each wheel is 1 kg in this model; the loop above computes sum of
    # (wheel_mass * position). Result is (0, 0) by symmetry.

    # J1 + J2 sit at the arm-mount X, on the chassis Y centerline
    M_j12 = M_J1 + M_J2
    cx_j12 = (M_J1 + M_J2) * ARM_MOUNT_X
    cy_j12 = (M_J1 + M_J2) * ARM_MOUNT_Y

    M_static = M_chassis + M_wheels + M_j12
    Mx_static = cx_chassis * M_chassis + cx_wheels + cx_j12
    My_static = cy_chassis * M_chassis + cy_wheels + cy_j12
    return M_static, Mx_static, My_static


def tipping_at_j1(j1_rad, j2_rad, j3_rad, j4_rad, arm_params,
                  chassis_world_pos=(0, 0, 0)):
    """Static tipping check at a given J1 yaw and arm pose.

    Combined CG = (chassis + 4 wheels + J1 + J2 + moving arm) / total mass.
    Project onto ground plane, check vs wheel-corner support polygon.
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    c1 = math.cos(j1_rad)
    s1 = math.sin(j1_rad)
    mx, my = ARM_MOUNT_X, ARM_MOUNT_Y

    # Static side: chassis + wheels + J1 + J2
    M_static, Mx_static, My_static = _chassis_side_mass_and_cg()

    # Moving side: bodies outboard of J2 (rotated by J1 around mount)
    M_arm = 0.0
    Mx_arm = 0.0
    My_arm = 0.0
    for label, mass in [
        ("L2_mid", arm_params["M_L2"]),
        ("j3", arm_params["M_J3"]),
        ("L3_mid", arm_params["M_L3"]),
        ("j4", arm_params["M_J4"]),
        ("L4_mid", arm_params["M_L4"]),
        ("cam", arm_params["M_camera"]),
        ("ee", arm_params["M_gripper"]),
    ]:
        fx, _ = fk[label]
        bx = mx + fx * c1
        by = my + fx * s1
        M_arm += mass
        Mx_arm += mass * bx
        My_arm += mass * by

    total_mass = M_static + M_arm
    cg_x = (Mx_static + Mx_arm) / total_mass
    cg_y = (My_static + My_arm) / total_mass

    wheel_xs = [pos[0] for pos in WHEEL_XY.values()]
    wheel_ys = [pos[1] for pos in WHEEL_XY.values()]
    margin_front = max(wheel_xs) - cg_x
    margin_back = cg_x - min(wheel_xs)
    margin_left = max(wheel_ys) - cg_y
    margin_right = cg_y - min(wheel_ys)
    worst = min(margin_front, margin_back, margin_left, margin_right)

    return {
        "stable": worst > 0,
        "margin_worst": worst,
        "margin_front": margin_front,
        "margin_back": margin_back,
        "margin_left": margin_left,
        "margin_right": margin_right,
        "cg_x": cg_x,
        "cg_y": cg_y,
        "total_mass": total_mass,
        "arm_mass": M_arm,
        "static_mass": M_static,
    }


def tipping_sweep_j1(j2_rad, j3_rad, j4_rad, arm_params, j1_angles_deg=None):
    if j1_angles_deg is None:
        j1_angles_deg = list(range(0, 360, 15))
    worst_margin = float("inf")
    worst_j1 = 0.0
    n_stable = 0
    for j1_deg in j1_angles_deg:
        tip = tipping_at_j1(math.radians(j1_deg), j2_rad, j3_rad, j4_rad,
                            arm_params)
        if tip["margin_worst"] < worst_margin:
            worst_margin = tip["margin_worst"]
            worst_j1 = j1_deg
        if tip["stable"]:
            n_stable += 1
    return {
        "worst_margin": worst_margin,
        "worst_j1_deg": worst_j1,
        "stable_all": n_stable == len(j1_angles_deg),
        "n_stable": n_stable,
        "n_total": len(j1_angles_deg),
    }


def tipping_at_j1_tilted(j1_rad, j2_rad, j3_rad, j4_rad, arm_params,
                         tilt_rad=0.0, tilt_axis="Y"):
    """Tilt-corrected tipping check (chassis + wheels + J1 + J2 + arm)."""
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)
    c1 = math.cos(j1_rad)
    s1 = math.sin(j1_rad)
    mx, my = ARM_MOUNT_X, ARM_MOUNT_Y
    tan_tilt = math.tan(tilt_rad)

    # ---- Static side (chassis-mounted bodies) with their heights ----
    bodies = []
    # Chassis body: assume CG at chassis center (z=0 in our convention)
    bodies.append((0.0, 0.0, 0.0, CHASSIS_MASS))
    # 4 wheels at corners; height = wheel center = chassis center - half_h + WHEEL_RADIUS
    wheel_z = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
    for (wx, wy) in WHEEL_XY.values():
        bodies.append((wx, wy, wheel_z, 1.0))
    # J1 motor stack (centered at top of chassis + half J1_STACK_H)
    j1_z = ARM_MOUNT_Z + J1_STACK_H / 2.0
    bodies.append((mx, my, j1_z, M_J1))
    # J2 motor stack (just above J1 stack)
    j2_z = ARM_MOUNT_Z + J1_STACK_H
    bodies.append((mx, my, j2_z + 0.05, M_J2))  # +5 cm above J2 pivot for motor body

    # ---- Moving side ----
    for label, mass in [
        ("L2_mid", arm_params["M_L2"]),
        ("j3", arm_params["M_J3"]),
        ("L3_mid", arm_params["M_L3"]),
        ("j4", arm_params["M_J4"]),
        ("L4_mid", arm_params["M_L4"]),
        ("cam", arm_params["M_camera"]),
        ("ee", arm_params["M_gripper"]),
    ]:
        fx, fz = fk[label]
        bx = mx + fx * c1
        by = my + fx * s1
        bz = j2_z + fz
        bodies.append((bx, by, bz, mass))

    total_mass = sum(m for _, _, _, m in bodies)
    cg_x = 0.0
    cg_y = 0.0
    for bx, by, bz, m in bodies:
        apparent_y = by + bz * tan_tilt
        cg_x += m * bx
        cg_y += m * apparent_y
    cg_x /= total_mass
    cg_y /= total_mass

    wheel_xs = [pos[0] for pos in WHEEL_XY.values()]
    wheel_ys = [pos[1] for pos in WHEEL_XY.values()]
    margin_front = max(wheel_xs) - cg_x
    margin_back = cg_x - min(wheel_xs)
    margin_left = max(wheel_ys) - cg_y
    margin_right = cg_y - min(wheel_ys)
    worst = min(margin_front, margin_back, margin_left, margin_right)

    return {
        "stable": worst > 0,
        "margin_worst": worst,
        "margin_front": margin_front,
        "margin_back": margin_back,
        "margin_left": margin_left,
        "margin_right": margin_right,
        "cg_x": cg_x,
        "cg_y": cg_y,
        "tilt_deg": math.degrees(tilt_rad),
    }


def tipping_sweep_j1_tilted(j2_rad, j3_rad, j4_rad, arm_params,
                            tilt_rad=0.0, j1_angles_deg=None):
    if j1_angles_deg is None:
        j1_angles_deg = list(range(0, 360, 15))
    worst_margin = float("inf")
    best_margin = float("-inf")
    worst_j1 = 0.0
    best_j1 = 0.0
    n_stable = 0
    for j1_deg in j1_angles_deg:
        tip = tipping_at_j1_tilted(
            math.radians(j1_deg), j2_rad, j3_rad, j4_rad,
            arm_params, tilt_rad=tilt_rad)
        if tip["margin_worst"] < worst_margin:
            worst_margin = tip["margin_worst"]
            worst_j1 = j1_deg
        if tip["margin_worst"] > best_margin:
            best_margin = tip["margin_worst"]
            best_j1 = j1_deg
        if tip["stable"]:
            n_stable += 1
    return {
        "worst_margin": worst_margin,
        "worst_j1_deg": worst_j1,
        "best_margin": best_margin,
        "best_j1_deg": best_j1,
        "stable_all": n_stable == len(j1_angles_deg),
        "n_stable": n_stable,
        "n_total": len(j1_angles_deg),
    }


# ==========================================================================
# Workspace sweep helper -- same shape as v2's, no config arg
# ==========================================================================
def sweep_workspace(arm_params, r_range, z_range, ee_pitches_deg,
                    z_j2_above_floor, j1_tipping_angles_deg=None):
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    results = []
    for r in r_range:
        for z in z_range:
            for phi_deg in ee_pitches_deg:
                phi_rad = math.radians(phi_deg)
                for sol_idx, sol in enumerate(ik_planar(r, z, phi_rad, L2, L3, L4)):
                    j2, j3, j4 = sol["j2"], sol["j3"], sol["j4"]
                    if any(abs(a) > math.pi for a in [j2, j3, j4]):
                        continue
                    if not check_collisions(j2, j3, j4, arm_params,
                                            z_j2_above_floor)["valid"]:
                        continue
                    fk = fk_planar(j2, j3, j4, L2, L3, L4)
                    torques = gravity_torques(j2, j3, j4, arm_params)
                    tip = tipping_sweep_j1(j2, j3, j4, arm_params,
                                           j1_tipping_angles_deg)
                    results.append({
                        "target_r": r,
                        "target_z": z,
                        "ee_pitch_deg": phi_deg,
                        "solution_idx": sol_idx,
                        "j2_deg": math.degrees(j2),
                        "j3_deg": math.degrees(j3),
                        "j4_deg": math.degrees(j4),
                        "ee_x": fk["ee"][0],
                        "ee_z": fk["ee"][1],
                        "tau_j2": torques["tau_j2"],
                        "tau_j3": torques["tau_j3"],
                        "tau_j4": torques["tau_j4"],
                        "tipping_worst_margin": tip["worst_margin"],
                        "tipping_worst_j1_deg": tip["worst_j1_deg"],
                        "tipping_stable_all_j1": tip["stable_all"],
                        "tipping_n_stable": tip["n_stable"],
                        "tipping_n_total": tip["n_total"],
                    })
    return results
