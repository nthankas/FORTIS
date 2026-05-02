"""
Analytical IK/FK, gravity torques, and collision checks for the FORTIS 4-DOF arm.

V2 HARDWARE SPEC (2026-04-11) — heterogeneous motors + larger CF tube:
  J1: NEMA 17 + Cricket MK II 25:1                        (0.580 kg)
  J2: NEMA 23 + EG23-G20-D10 20:1 + adapter               (2.665 kg)
  J3: NEMA 17 + Cricket MK II 25:1 + adapter              (0.655 kg)
  J4: D845WP servo + adapter                              (0.302 kg)
  Links: CF square tube 1.25"x1.38" OD, 0.065" wall       (0.226 lb/ft)
  Camera (Orbbec Gemini 2, 98 g) mounted at L4 midpoint   (1.5" from J4)
  Gripper: ServoCity parallel kit + D645MW + adapter      (0.216 kg)
  Payload: 3 lb (1.361 kg) at L4 tip

Kinematic chain: J1 (Z yaw) + J2/J3/J4 (Y pitch), yaw-decoupled planar.
FK convention: standard 3R planar, no direction multipliers.
  x_j3 = L2*cos(A2),  z_j3 = L2*sin(A2)
  x_j4 = x_j3 + L3*cos(A2+A3),  etc.

No Isaac Sim dependency. Pure Python + numpy.
"""

import math
import numpy as np

# ==========================================================================
# Constants
# ==========================================================================
IN = 0.0254  # inches to meters
GRAV = 9.81

# Arm configurations: reach_inches -> (L2_in, L3_in, L4_in)
ARM_CONFIGS = {
    36: (17.0, 15.0, 4.0),
    30: (15.0, 12.0, 3.0),
    24: (12.0, 10.0, 2.0),
}

# ----------------------------------------------------------------------
# Per-joint masses (heterogeneous hardware spec)
# ----------------------------------------------------------------------
M_J1 = 0.580   # NEMA 17 (500 g) + Cricket MK II 25:1 (80 g)
M_J2 = 2.665   # NEMA 23 (1500 g) + EG23-G20-D10 20:1 (1090 g) + adapter (75 g)
M_J3 = 0.655   # NEMA 17 (500 g) + Cricket MK II 25:1 (80 g) + adapter (75 g)
M_J4 = 0.302   # D845WP servo (227 g) + adapter (75 g)

# Legacy placeholder — some callers still reference a single M_JOINT. Not
# used in torque math below; the per-joint values above are authoritative.
M_JOINT = M_J1

# End-effector and instrumentation
# Gripper total: ServoCity kit 81 g + D645MW servo 60 g + adapter 75 g = 216 g
M_GRIPPER_BARE = 0.216
M_PAYLOAD = 1.361         # kg (3 lb max payload)
M_CAMERA = 0.098          # kg (Orbbec Gemini 2) — now on L4 midpoint
CAM_OFFSET_ON_L4 = 1.5 * IN  # camera position along L4 from J4

# Torque limits (Nm) — continuous ratings per joint
TORQUE_LIMITS_NM = {
    "J1": 12.0,   # NEMA 17 + Cricket 25:1, conservative cont limit
    "J2": 30.0,   # EG23-G20 20:1 continuous (60 Nm peak)
    "J3": 12.0,   # Cricket MK II 25:1 continuous
    "J4": 4.9,    # D845WP at 7.4 V
}

# Joint ranges (USD convention, degrees)
JOINT_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),   # software-limited yaw
    "ArmJ2": (-30.0, 150.0),    # physical range
    "ArmJ3": (-180.0, 180.0),   # self-collision limited
    "ArmJ4": (-101.0, 101.0),   # hardware-limited servo
}

# CF square tube: RockWest Composites 1.25" x 1.38" OD, 0.065" wall
# Linear density: 0.226 lb/ft = 0.018833 lb/in -> 0.3362 kg/m
CF_DENSITY_LB_PER_FT = 0.226
CF_DENSITY_KG_PER_M = CF_DENSITY_LB_PER_FT * 0.453592 / (12.0 * IN)  # ~0.3362 kg/m

# CF tube cross-section (for inertia + collision envelope)
CF_TUBE_Y = 1.25 * IN   # tube Y dimension (~31.8 mm)
CF_TUBE_Z = 1.38 * IN   # tube Z dimension (~35.1 mm)

# Chassis geometry (rectangular skeleton, no chamfer)
CHASSIS_L = 13.082 * IN  # 0.3323 m
CHASSIS_W = 8.54 * IN    # 0.2169 m
CHASSIS_H = 6.0 * IN     # 0.1524 m

# Arm mount: 3" forward from back edge, centerline, flush on top
ARM_MOUNT_X = -CHASSIS_L / 2.0 + 3.0 * IN
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0

J1_STACK_H = 1.5 * IN  # 0.0381 m

# Wheel positions (rectangular chassis, wheels flush at corners)
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0

WHEEL_XY = {
    "FR": ( SL, -SW),
    "FL": ( SL,  SW),
    "BL": (-SL,  SW),
    "BR": (-SL, -SW),
}
WHEEL_RADIUS = 203.0 / 2.0 / 1000.0  # 0.1015 m (AndyMark 8" Dualie)
CHASSIS_MASS = 20.4  # kg

BELLY_HEIGHT = 2.0 * IN  # 0.0508 m


# ==========================================================================
# Arm config helper
# ==========================================================================
def get_arm_params(reach_in, loaded=False):
    """Return arm parameters for a given reach config (v2 heterogeneous)."""
    if reach_in not in ARM_CONFIGS:
        raise ValueError(f"Unknown arm config: {reach_in}\" (valid: {list(ARM_CONFIGS.keys())})")
    l2_in, l3_in, l4_in = ARM_CONFIGS[reach_in]
    L2 = l2_in * IN
    L3 = l3_in * IN
    L4 = l4_in * IN
    M_L2 = CF_DENSITY_KG_PER_M * L2
    M_L3 = CF_DENSITY_KG_PER_M * L3
    M_L4 = CF_DENSITY_KG_PER_M * L4
    M_gripper = M_GRIPPER_BARE + M_PAYLOAD if loaded else M_GRIPPER_BARE
    total_arm = M_J1 + M_J2 + M_J3 + M_J4 + M_L2 + M_L3 + M_L4 + M_CAMERA + M_gripper
    return {
        "reach_in": reach_in,
        "loaded": loaded,
        "L2": L2, "L3": L3, "L4": L4,
        "M_L2": M_L2, "M_L3": M_L3, "M_L4": M_L4,
        # Per-joint masses (heterogeneous)
        "M_J1": M_J1, "M_J2": M_J2, "M_J3": M_J3, "M_J4": M_J4,
        # Legacy single-joint alias (some callers still use this; points at J1)
        "M_joint": M_J1,
        "M_gripper": M_gripper,
        "M_camera": M_CAMERA,
        # Camera now rides L4 at its midpoint, not L2 shoulder
        "cam_offset_L4": CAM_OFFSET_ON_L4,
        "max_reach": L2 + L3 + L4,
        "total_arm_mass": total_arm,
    }


# ==========================================================================
# Forward Kinematics
# ==========================================================================
def fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4):
    """
    Standard 3R planar FK in the arm's vertical plane.

    Returns positions relative to J2 pivot:
      x = radial (horizontal in the vertical plane)
      z = vertical (up positive)

    All joints are Y-axis pitch. Convention: j2=0 means L2 horizontal (+X),
    j2=90° means L2 points straight up.
    """
    A2 = j2_rad
    A23 = j2_rad + j3_rad
    A234 = j2_rad + j3_rad + j4_rad

    x_j3 = L2 * math.cos(A2)
    z_j3 = L2 * math.sin(A2)
    x_j4 = x_j3 + L3 * math.cos(A23)
    z_j4 = z_j3 + L3 * math.sin(A23)
    x_ee = x_j4 + L4 * math.cos(A234)
    z_ee = z_j4 + L4 * math.sin(A234)

    # Link midpoints (for gravity torque CG computation)
    x_L2_mid = 0.5 * L2 * math.cos(A2)
    z_L2_mid = 0.5 * L2 * math.sin(A2)
    x_L3_mid = x_j3 + 0.5 * L3 * math.cos(A23)
    z_L3_mid = z_j3 + 0.5 * L3 * math.sin(A23)
    x_L4_mid = x_j4 + 0.5 * L4 * math.cos(A234)
    z_L4_mid = z_j4 + 0.5 * L4 * math.sin(A234)

    # Camera on L4 at CAM_OFFSET_ON_L4 from the J4 pivot (along L4)
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


def fk_planar_vec(j2, j3, j4, L2, L3, L4):
    """Vectorized FK for arrays of joint angles (radians). Returns (N,2) arrays."""
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
def ik_planar(target_r, target_z, ee_pitch_rad, L2, L3, L4):
    """
    Analytical IK for the 3R planar arm.

    Args:
        target_r: radial distance from J2 in the vertical plane (m)
        target_z: height relative to J2 pivot (m)
        ee_pitch_rad: desired cumulative EE angle (j2+j3+j4) from horizontal
        L2, L3, L4: link lengths (m)

    Returns:
        List of dicts with keys {j2, j3, j4} in radians, or empty if unreachable.
        Up to 2 solutions (elbow-up and elbow-down).
    """
    # Wrist point: subtract L4 contribution
    wx = target_r - L4 * math.cos(ee_pitch_rad)
    wz = target_z - L4 * math.sin(ee_pitch_rad)

    # 2R IK for J2, J3 to reach wrist point (wx, wz)
    d_sq = wx * wx + wz * wz
    d = math.sqrt(d_sq)

    # Reachability check
    if d > L2 + L3 or d < abs(L2 - L3):
        return []
    if d < 1e-10:
        return []

    cos_j3 = (d_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    # Numerical clamp
    cos_j3 = max(-1.0, min(1.0, cos_j3))

    solutions = []
    for sign in [1.0, -1.0]:  # elbow-down, elbow-up
        j3 = sign * math.acos(cos_j3)
        beta = math.atan2(L3 * math.sin(j3), L2 + L3 * math.cos(j3))
        j2 = math.atan2(wz, wx) - beta
        j4 = ee_pitch_rad - j2 - j3

        solutions.append({"j2": j2, "j3": j3, "j4": j4})

    return solutions


def ik_3d(target_x, target_y, target_z, ee_pitch_rad, arm_params,
          z_j2_above_floor=None):
    """
    Full 3D IK: J1 yaw + 3R planar.

    Args:
        target_x, target_y, target_z: world-frame target position (m)
        ee_pitch_rad: desired EE pitch angle from horizontal
        arm_params: dict from get_arm_params()
        z_j2_above_floor: height of J2 above the lowest floor (m).
            If None, target_z is relative to J2.

    Returns:
        List of dicts with {j1, j2, j3, j4} in radians, or empty.
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]

    # J1: yaw to face target
    dx = target_x - ARM_MOUNT_X
    dy = target_y - ARM_MOUNT_Y
    r = math.sqrt(dx * dx + dy * dy)
    j1 = math.atan2(dy, dx)

    # Vertical offset: target_z relative to J2 pivot
    if z_j2_above_floor is not None:
        pz = target_z  # already relative to J2
    else:
        pz = target_z - (ARM_MOUNT_Z + J1_STACK_H)

    planar = ik_planar(r, pz, ee_pitch_rad, L2, L3, L4)
    results = []
    for sol in planar:
        results.append({
            "j1": j1,
            "j2": sol["j2"],
            "j3": sol["j3"],
            "j4": sol["j4"],
        })
    return results


# ==========================================================================
# Gravity Torques (analytical)
# ==========================================================================
def gravity_torques(j2_rad, j3_rad, j4_rad, arm_params):
    """
    Compute static gravity torques at J2, J3, J4 with per-joint masses.

    Returns dict with tau_j2, tau_j3, tau_j4 in Nm (signed).
    Positive = motor must push against gravity in the positive joint direction.
    J1 torque depends on J1 angle (yaw changes moment arm direction);
    computed separately via gravity_torque_j1().
    """
    L2 = arm_params["L2"]
    L3 = arm_params["L3"]
    L4 = arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    M_J3_ = arm_params["M_J3"]   # elbow motor at J3 pivot
    M_J4_ = arm_params["M_J4"]   # wrist motor at J4 pivot
    M_L2 = arm_params["M_L2"]
    M_L3 = arm_params["M_L3"]
    M_L4 = arm_params["M_L4"]
    M_grip = arm_params["M_gripper"]
    M_cam = arm_params["M_camera"]

    # Positions relative to J2 pivot (x = horizontal in vertical plane)
    x_j3, _ = fk["j3"]
    x_j4, _ = fk["j4"]
    x_ee, _ = fk["ee"]
    x_L2m, _ = fk["L2_mid"]
    x_L3m, _ = fk["L3_mid"]
    x_L4m, _ = fk["L4_mid"]
    x_cam, _ = fk["cam"]   # camera now sits on L4 (see fk_planar)

    # Gravity torque at J2: horizontal distance from J2 to each mass
    # (tau = -g * sum(m_i * x_i); negative = motor pushes up against gravity)
    tau_j2 = -GRAV * (
        M_L2 * x_L2m +            # L2 link CG
        M_J3_ * x_j3 +            # J3 elbow motor stack
        M_L3 * x_L3m +            # L3 link CG
        M_J4_ * x_j4 +            # J4 wrist motor stack
        M_L4 * x_L4m +            # L4 link CG
        M_cam * x_cam +           # camera on L4 midpoint
        M_grip * x_ee             # gripper at L4 tip
    )

    # Gravity torque at J3: horizontal distance from J3
    tau_j3 = -GRAV * (
        M_L3 * (x_L3m - x_j3) +
        M_J4_ * (x_j4 - x_j3) +
        M_L4 * (x_L4m - x_j3) +
        M_cam * (x_cam - x_j3) +
        M_grip * (x_ee - x_j3)
    )

    # Gravity torque at J4: horizontal distance from J4
    tau_j4 = -GRAV * (
        M_L4 * (x_L4m - x_j4) +
        M_cam * (x_cam - x_j4) +
        M_grip * (x_ee - x_j4)
    )

    return {"tau_j2": tau_j2, "tau_j3": tau_j3, "tau_j4": tau_j4}


def gravity_torque_j1(j1_rad, j2_rad, j3_rad, j4_rad, arm_params):
    """
    J1 gravity torque at a given yaw angle.

    J1 rotates around Z. Gravity doesn't directly create a torque on a
    vertical-axis joint. However, if the arm CG is off the J1 axis,
    friction and dynamics create a small torque. For static analysis,
    tau_j1 = 0 (gravity acts in Z, J1 rotates in XY).

    Returns 0.0 for static analysis.
    """
    return 0.0


# ==========================================================================
# Collision Checks
# ==========================================================================
def _segment_in_box(p1x, p1z, p2x, p2z, box_x_min, box_x_max, box_z_min, box_z_max,
                    n_samples=5):
    """
    Check if a line segment from (p1x,p1z) to (p2x,p2z) intersects a 2D box.
    Uses sampling along the segment.
    """
    for t in np.linspace(0.0, 1.0, n_samples):
        px = p1x + t * (p2x - p1x)
        pz = p1z + t * (p2z - p1z)
        if box_x_min < px < box_x_max and box_z_min < pz < box_z_max:
            return True
    return False


def _segment_distance(seg1, seg2):
    """Minimum distance between two 2D line segments. Each seg = ((x1,z1),(x2,z2))."""
    (ax1, az1), (ax2, az2) = seg1
    (bx1, bz1), (bx2, bz2) = seg2
    # Sample both segments and compute min distance
    min_d = float('inf')
    for s in np.linspace(0, 1, 12):
        px = ax1 + s * (ax2 - ax1)
        pz = az1 + s * (az2 - az1)
        for t in np.linspace(0, 1, 12):
            qx = bx1 + t * (bx2 - bx1)
            qz = bz1 + t * (bz2 - bz1)
            d = math.sqrt((px - qx)**2 + (pz - qz)**2)
            if d < min_d:
                min_d = d
    return min_d


def check_collisions(j2_rad, j3_rad, j4_rad, arm_params, z_j2_above_floor,
                     margin=0.025):
    """
    Check analytical collisions for a planar arm pose.

    Args:
        j2_rad, j3_rad, j4_rad: joint angles
        arm_params: from get_arm_params()
        z_j2_above_floor: J2 height above the lowest floor (m)
        margin: collision margin (m)

    Returns:
        dict with {valid: bool, reason: str or None}
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    x_j3, z_j3 = fk["j3"]
    x_j4, z_j4 = fk["j4"]
    x_ee, z_ee = fk["ee"]

    # Link half-thickness: CF tube 1.25"x1.38" + joint motor clearance.
    # Use the larger of the two cross-section dims for a conservative bound.
    LINK_HALF_THICK = max(CF_TUBE_Y, CF_TUBE_Z) / 2.0 + 0.005  # ~23 mm

    # --- Floor collision: any key point below ground ---
    floor_margin = margin + LINK_HALF_THICK
    for label, (_, z) in [("j3", fk["j3"]), ("j4", fk["j4"]), ("ee", fk["ee"])]:
        if z + z_j2_above_floor < floor_margin:
            return {"valid": False, "reason": "floor"}
    for label in ["L2_mid", "L3_mid", "L4_mid"]:
        _, z = fk[label]
        if z + z_j2_above_floor < floor_margin:
            return {"valid": False, "reason": "floor"}

    # --- Chassis collision: link segments vs chassis bounding box ---
    # Asymmetric expansion:
    # - x: expand by margin + link_half (links have width)
    # - z_min: expand downward by margin + link_half
    # - z_max: only expand by margin (NOT link_half, since J2 is at z=0
    #   and we don't want to flag links that clear the chassis top via J1 stack)
    x_expand = margin + LINK_HALF_THICK
    z_down_expand = margin + LINK_HALF_THICK
    z_up_expand = margin  # conservative but not too aggressive near J2
    chassis_x_min = -x_expand
    chassis_x_max = CHASSIS_L + x_expand
    # J2 is at height (ARM_MOUNT_Z + J1_STACK_H) above chassis center.
    # Chassis bottom is ARM_MOUNT_Z below center = (2*ARM_MOUNT_Z + J1_STACK_H) below J2
    chassis_z_min = -(CHASSIS_H + J1_STACK_H) - z_down_expand
    chassis_z_max = -J1_STACK_H + z_up_expand

    # Check link segments, skipping the J2 origin (t=0 for L2)
    segments = [
        ((0, 0), fk["j3"]),       # J2 to J3 (L2)
        (fk["j3"], fk["j4"]),     # J3 to J4 (L3)
        (fk["j4"], fk["ee"]),     # J4 to EE (L4)
    ]
    for seg_idx, ((p1x, p1z), (p2x, p2z)) in enumerate(segments):
        # For L2 (starts at J2), skip the first sample to avoid flagging J2 itself
        t_start = 0.08 if seg_idx == 0 else 0.0
        for t in np.linspace(t_start, 1.0, 10):
            px = p1x + t * (p2x - p1x)
            pz = p1z + t * (p2z - p1z)
            if chassis_x_min < px < chassis_x_max and chassis_z_min < pz < chassis_z_max:
                return {"valid": False, "reason": "chassis"}

    # --- Self-collision: L4 vs L2, L4 vs J1 base ---
    # Minimum distance between non-adjacent link segments
    L4_seg = (fk["j4"], fk["ee"])
    L2_seg = ((0, 0), fk["j3"])
    min_dist = _segment_distance(L4_seg, L2_seg)
    if min_dist < 2 * LINK_HALF_THICK:
        return {"valid": False, "reason": "self_L4_L2"}

    # L4 vs J1 base (J1 base is a small box at origin, height J1_STACK_H)
    j1_half = 0.030  # ~57mm/2 box around J1 base
    for t in np.linspace(0, 1.0, 8):
        px = fk["j4"][0] + t * (fk["ee"][0] - fk["j4"][0])
        pz = fk["j4"][1] + t * (fk["ee"][1] - fk["j4"][1])
        if abs(px) < j1_half + LINK_HALF_THICK and -J1_STACK_H < pz < 0:
            return {"valid": False, "reason": "self_L4_J1"}

    return {"valid": True, "reason": None}


# ==========================================================================
# Tipping analysis (analytical)
# ==========================================================================
def tipping_at_j1(j1_rad, j2_rad, j3_rad, j4_rad, arm_params,
                  chassis_world_pos=(0, 0, 0)):
    """
    Analytical tipping check: does the combined robot+arm CG project
    inside the wheel support polygon at a given J1 yaw?

    This is the Phase 1 analytical estimate. Phase 2 uses physics sim.

    Returns dict with {stable, margin_worst, margin_front/back/left/right}.
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    # Arm body positions in the vertical plane (relative to J2)
    # Rotate into world XY frame using J1 yaw
    c1 = math.cos(j1_rad)
    s1 = math.sin(j1_rad)

    # Mount point in chassis frame
    mx = ARM_MOUNT_X
    my = ARM_MOUNT_Y

    # Body positions in chassis XY frame
    bodies = []

    # J1 base (at mount point, no horizontal offset)
    bodies.append((mx, my, arm_params["M_J1"]))

    # Each outboard body: rotate (fk_x, 0) by J1 around mount
    for label, mass in [
        ("L2_mid", arm_params["M_L2"]),
        ("j3", arm_params["M_J3"]),   # J3 elbow motor
        ("L3_mid", arm_params["M_L3"]),
        ("j4", arm_params["M_J4"]),   # J4 wrist motor
        ("L4_mid", arm_params["M_L4"]),
        ("cam", arm_params["M_camera"]),  # camera on L4 midpoint
        ("ee", arm_params["M_gripper"]),
    ]:
        fx, fz = fk[label]
        # In world XY: rotate the radial component by J1
        bx = mx + fx * c1
        by = my + fx * s1
        bodies.append((bx, by, mass))

    # Combined CG (XY only, arm + chassis)
    total_mass = CHASSIS_MASS + sum(m for _, _, m in bodies)
    cg_x = CHASSIS_MASS * 0.0  # chassis CG at origin
    cg_y = CHASSIS_MASS * 0.0
    for bx, by, m in bodies:
        cg_x += m * bx
        cg_y += m * by
    cg_x /= total_mass
    cg_y /= total_mass

    # Wheel support polygon (AABB)
    wheel_xs = [pos[0] for pos in WHEEL_XY.values()]
    wheel_ys = [pos[1] for pos in WHEEL_XY.values()]
    x_min = min(wheel_xs)
    x_max = max(wheel_xs)
    y_min = min(wheel_ys)
    y_max = max(wheel_ys)

    margin_front = x_max - cg_x
    margin_back = cg_x - x_min
    margin_left = y_max - cg_y
    margin_right = cg_y - y_min
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
    }


def tipping_sweep_j1(j2_rad, j3_rad, j4_rad, arm_params,
                     j1_angles_deg=None):
    """
    Sweep J1 yaw and find worst-case tipping margin.

    Args:
        j1_angles_deg: list of J1 angles to test (default: 0 to 345, step 15)

    Returns:
        dict with {worst_margin, worst_j1_deg, stable_all, n_stable, n_total}
    """
    if j1_angles_deg is None:
        j1_angles_deg = list(range(0, 360, 15))

    worst_margin = float("inf")
    worst_j1 = 0.0
    n_stable = 0

    for j1_deg in j1_angles_deg:
        j1_rad = math.radians(j1_deg)
        tip = tipping_at_j1(j1_rad, j2_rad, j3_rad, j4_rad, arm_params)
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
    """
    Tipping check with chassis tilt correction for step straddling.

    When the chassis straddles the step, it tilts (rolls) around the X axis.
    The gravity vector is no longer aligned with the chassis Z axis —
    it has a lateral component that shifts the effective CG projection.

    The correction: each body's CG position, when projected onto the ground
    plane along the TRUE gravity vector (not chassis Z), shifts laterally
    by  body_height_above_tilt_axis * tan(tilt).

    For a roll around X: the shift is in Y (toward the lower side).
    tilt_rad > 0 means the +Y side (FL, BL wheels) is lower.

    Args:
        tilt_rad: chassis roll angle in radians (measured or computed from step)
        tilt_axis: "Y" means step edge runs along X, chassis rolls around X
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    fk = fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)

    c1 = math.cos(j1_rad)
    s1 = math.sin(j1_rad)

    mx = ARM_MOUNT_X
    my = ARM_MOUNT_Y

    # tan(tilt) for CG shift computation
    tan_tilt = math.tan(tilt_rad)

    # Build body list: (x, y, z_above_chassis_center, mass)
    # z_above_chassis_center is used for the tilt shift
    bodies = []

    # J1 base at mount point, z = mount height above chassis center
    j1_z = ARM_MOUNT_Z  # chassis top
    bodies.append((mx, my, j1_z, arm_params["M_J1"]))

    # Outboard bodies: fk gives (x, z) in the planar arm frame
    # z in FK is height above J2, and J2 is at ARM_MOUNT_Z + J1_STACK_H
    j2_z = ARM_MOUNT_Z + J1_STACK_H
    for label, mass in [
        ("L2_mid", arm_params["M_L2"]),
        ("j3", arm_params["M_J3"]),
        ("L3_mid", arm_params["M_L3"]),
        ("j4", arm_params["M_J4"]),
        ("L4_mid", arm_params["M_L4"]),
        ("cam", arm_params["M_camera"]),  # camera on L4 midpoint
        ("ee", arm_params["M_gripper"]),
    ]:
        fx, fz = fk[label]
        bx = mx + fx * c1
        by = my + fx * s1
        bz = j2_z + fz  # height above chassis center
        bodies.append((bx, by, bz, mass))

    # Combined CG in chassis XY frame, with tilt-corrected projection
    # For each body, the true gravity projection shifts its apparent Y
    # position by: body_height * tan(tilt) toward the lower side (+Y).
    total_mass = CHASSIS_MASS + sum(m for _, _, _, m in bodies)

    # Chassis CG: at chassis center (0, 0, 0). Height above tilt axis = 0
    # (the tilt axis passes through chassis center approximately)
    cg_x = CHASSIS_MASS * 0.0
    cg_y = CHASSIS_MASS * 0.0

    for bx, by, bz, m in bodies:
        # Tilt shifts the apparent Y by bz * tan(tilt)
        apparent_y = by + bz * tan_tilt
        cg_x += m * bx
        cg_y += m * apparent_y
    cg_x /= total_mass
    cg_y /= total_mass

    # Wheel support polygon (AABB) — same as level version
    wheel_xs = [pos[0] for pos in WHEEL_XY.values()]
    wheel_ys = [pos[1] for pos in WHEEL_XY.values()]
    x_min = min(wheel_xs)
    x_max = max(wheel_xs)
    y_min = min(wheel_ys)
    y_max = max(wheel_ys)

    margin_front = x_max - cg_x
    margin_back = cg_x - x_min
    margin_left = y_max - cg_y
    margin_right = cg_y - y_min
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
    """
    Sweep J1 yaw with tilt correction. Same interface as tipping_sweep_j1()
    but accounts for chassis roll from step straddling.
    """
    if j1_angles_deg is None:
        j1_angles_deg = list(range(0, 360, 15))

    worst_margin = float("inf")
    worst_j1 = 0.0
    best_margin = float("-inf")
    best_j1 = 0.0
    n_stable = 0

    for j1_deg in j1_angles_deg:
        j1_rad = math.radians(j1_deg)
        tip = tipping_at_j1_tilted(
            j1_rad, j2_rad, j3_rad, j4_rad, arm_params, tilt_rad=tilt_rad)
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
# Workspace sweep helper
# ==========================================================================
def sweep_workspace(arm_params, r_range, z_range, ee_pitches_deg,
                    z_j2_above_floor, j1_tipping_angles_deg=None):
    """
    Sweep a cylindrical (r, z) grid, solve IK, filter, compute torques.

    Args:
        arm_params: from get_arm_params()
        r_range: array of radial distances (m)
        z_range: array of heights relative to J2 (m)
        ee_pitches_deg: list of EE pitch angles (degrees)
        z_j2_above_floor: J2 height above lowest floor (m)
        j1_tipping_angles_deg: J1 angles for tipping sweep

    Returns:
        List of result dicts for each valid IK solution.
    """
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]
    results = []

    for r in r_range:
        for z in z_range:
            for phi_deg in ee_pitches_deg:
                phi_rad = math.radians(phi_deg)
                solutions = ik_planar(r, z, phi_rad, L2, L3, L4)

                for sol_idx, sol in enumerate(solutions):
                    j2, j3, j4 = sol["j2"], sol["j3"], sol["j4"]

                    # Joint limits (±180°)
                    if any(abs(a) > math.pi for a in [j2, j3, j4]):
                        continue

                    # Collision check
                    col = check_collisions(j2, j3, j4, arm_params,
                                          z_j2_above_floor)
                    if not col["valid"]:
                        continue

                    # FK for EE position
                    fk = fk_planar(j2, j3, j4, L2, L3, L4)

                    # Gravity torques
                    torques = gravity_torques(j2, j3, j4, arm_params)

                    # Tipping sweep
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
