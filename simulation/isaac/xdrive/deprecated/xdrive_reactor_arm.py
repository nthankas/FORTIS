"""
FORTIS Sim3 phase 1: X-drive chassis + 4-DOF arm in the DIII-D reactor.

Built on xdrive_realwheel.py (canonical arched chassis + real Kaya omni wheel
meshes, see Projects/FORTIS/STATE.md). Adds Carlos's 4-DOF arm rigidly
mounted on a forward bracket at BASE_X_OFFSET = +9", stowed in a VERTICAL
zigzag above the mount so every link sits at world r ~53.6" (safely between
the central column at r=38" and the step at r=55.5"). Phase 1 goal is visual
verification + keyboard joint control; no stability sweep yet.

Sources of truth:
- Chassis / wheel model: xdrive_realwheel.py (DO NOT re-import xdrive_reactor.py)
- Arm link dimensions: ../inverse_kinetmatic_solver.py DH_PARAMS (Carlos,
  CoppeliaSim). Numbers are copied, the module is NOT imported.
- Arm mass / motor / gearbox: arm_spec.md beside this file. All-NEMA-17 +
  Cricket Drive MK II 25:1 variant; gearbox rated torque 12 Nm = DriveAPI maxForce.

Usage: IsaacSim\\python.bat xdrive_reactor_arm.py --gui
"""
import os, sys, math, argparse
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--belly", type=float, default=2.5,
                    help="Belly height above ground in inches (default 2.5)")
parser.add_argument("--hz", type=int, default=360,
                    help="Physics Hz (360-480, default 360)")
args, _ = parser.parse_known_args()
headless = args.headless or not args.gui

from isaacsim import SimulationApp
app = SimulationApp({"headless": headless, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
import carb.input
import omni.appwindow
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

# ============================================================================
# Constants
# ============================================================================
IN = 0.0254
MM = 0.001
# --- xdrive path bootstrap (reorg 2026-04-04) ---
XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ASSETS_DIR  = os.path.join(XDRIVE_ROOT, "assets")
RESULTS_ROOT = os.path.join(XDRIVE_ROOT, "results")
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
# --- end bootstrap ---
OMNIWHEEL_USD = os.path.join(ASSETS_DIR, "omniwheels.usd")

# Chassis: straight-edge rectangular body
CHASSIS_L = 15.354 * IN     # front-to-back (X)
CHASSIS_W = 9.353 * IN      # side-to-side (Y)
CHASSIS_H = 7.1 * IN        # height (Z)
CHAMFER_FACE = 3.0 * IN     # 45-deg chamfer face for wheel mounting
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)

# Motor mount flats at front/rear (where gearbox sits, along X axis)
MOTOR_MOUNT_LEN = 2.5 * IN  # flat area at each end for gearbox + motor

# Belly arch (recessed center section for step clearance)
BELLY_HEIGHT = args.belly * IN  # how high the belly center is raised above chassis bottom
# The arch is between the motor mount flats. Ramps connect the flat to the raised center.
ARCH_FLAT_WIDTH = 3.0 * IN  # flat section at the raised belly center

# Reactor USD
REACTOR_USD = os.path.join(ASSETS_DIR, "diiid_reactor.usd")

# Wheel target dimensions (AndyMark 8" Dualie)
TARGET_DIA_MM = 203.0
TARGET_WIDTH_MM = 51.8
WHEEL_RADIUS = TARGET_DIA_MM / 2.0 * MM  # 0.1015 m
WHEEL_WIDTH = TARGET_WIDTH_MM * MM        # 0.0518 m

# Source wheel dimensions (measured from omniwheels.usd inspection)
SRC_DIA_MM = 82.5
SRC_WIDTH_MM = 44.87
SRC_CENTER_Y_MM = -17.43  # Y offset of wheel center in source

SCALE_XZ = TARGET_DIA_MM / SRC_DIA_MM   # 2.46
SCALE_Y = TARGET_WIDTH_MM / SRC_WIDTH_MM  # 1.155
SCALE_UNIFORM = SCALE_XZ                  # uniform scale for roller geometry (keeps proportions)

# Mass
CHASSIS_MASS = 20.4         # kg (45 lbs)
WHEEL_MASS = 1.0            # kg per wheel
ROLLER_MASS_FRAC = 0.3
NUM_ROLLERS = 10
HUB_MASS = WHEEL_MASS * (1.0 - ROLLER_MASS_FRAC)
ROLLER_MASS = WHEEL_MASS * ROLLER_MASS_FRAC / NUM_ROLLERS

# Physics
PHYSICS_HZ = max(360, min(480, args.hz))
FRICTION_MU = 0.5
DRIVE_DAMPING = 100.0
DRIVE_MAX_FORCE = 200.0

# Drive
DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5

# ============================================================================
# Arm constants (Sim3 phase 1)
# ============================================================================
# Link lengths from inverse_kinetmatic_solver.py DH_PARAMS (Carlos).
# Row 0 d = 0.1270 (J1 vertical stack), row 1 r = 0.5761 (L2 shoulder link),
# row 2 r = 0.5000 (L3 elbow link), row 3 r = 0.1500 (L4 wrist link). The
# 0.0250 row-0 r (lateral offset) is absorbed into the J1 base body geometry.
L_J1 = 0.1270   #  5.00"  J1 vertical base stack (chassis top -> J2 pivot)
L_L2 = 0.5761   # 22.68"  shoulder link (J2 pivot -> J3 pivot)
L_L3 = 0.5000   # 19.69"  elbow link    (J3 pivot -> J4 pivot)
L_L4 = 0.1500   #  5.91"  wrist link    (J4 pivot -> camera tip)

# Arm mount on chassis: BACK edge, chassis centerline, on top.
# The arm stows flat on top of the chassis (folded/parallel to chassis top).
# L2 is longer than the chassis (22.68" vs 15.35"), so mounting J1 at the back
# edge lets L2 extend forward across the top and cantilever past the front by
# ~7.3". Mounting at the back keeps the stowed CG over the rear wheels and
# puts the J1 yaw motor out of the way of the forward-facing camera workspace.
# Chassis +X side points toward the reactor center at spawn (yaw = 90 deg),
# so chassis -X = back of chassis = away from the center column.
ARM_MOUNT_X = -CHASSIS_L / 2.0         # -7.68" = back edge of chassis
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0          # chassis top = +3.55"

# All-NEMA-17 lumped masses from arm_spec.md (CF links, Cricket MK II 25:1).
M_J1_BASE     = 0.630   # J1 motor + gearbox + vertical stack hardware
M_J2_SHOULDER = 0.652   # J2 motor + gearbox + half L2 CF link
M_J3_ELBOW    = 0.666   # J3 motor + gearbox + half L2+L3 CF links
M_L4_LINK     = 0.653   # J4 motor + gearbox + L4 CF link (camera added separately)
M_CAMERA      = 0.445   # Orbbec Gemini 2 at L4 tip

# DriveAPI (USD native units: deg for positions, Nm/deg, Nm*s/deg, Nm).
# maxForce = 12 Nm matches the Cricket Drive MK II 25:1 rated gearbox torque
# (spec doc); motor stall would give 13.9 Nm but the gearbox is the limiting
# component. All four arm joints share the same motor+gearbox, so identical.
ARM_STIFFNESS_NM_PER_DEG = 1000.0
ARM_DAMPING_NMS_PER_DEG  = 100.0
ARM_MAX_FORCE_NM         = 12.0

# Joint limits in degrees.
ARM_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),   # base yaw, full rotation
    "ArmJ2": ( -20.0, 110.0),   # shoulder pitch; upper clamp keeps J2 below horizontal
    "ArmJ3": (-170.0, 170.0),   # elbow fold
    "ArmJ4": (-180.0, 180.0),   # wrist
}

# Visual box cross-section for arm links (cosmetic; no collision in phase 1).
LINK_BOX_W = 0.040  # 40 mm wide  (Y in body frame)
LINK_BOX_T = 0.025  # 25 mm thick (cross-section thickness)
# Z stack between parallel folded links in the stowed pose. Represents the
# real mechanical offset between stacked motor brackets + CF tubes when the
# arm is folded flat on top of the chassis. Also ensures the joint pivot
# axes for J3/J4 sit above the link they rotate about (motor bracket height).
VIS_Z_STACK = 0.045  # 45 mm = 1.77"

# Chassis geometry
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

# Octagon vertices (top view, for wheel mounting positions)
OCT_XY = [
    ( SL,       -(SW - C)),
    ( SL,        (SW - C)),
    ( (SL - C),  SW      ),
    (-(SL - C),  SW      ),
    (-SL,        (SW - C)),
    (-SL,       -(SW - C)),
    (-(SL - C), -SW      ),
    ( (SL - C), -SW      ),
]

def mid(a, b):
    return ((a[0]+b[0])/2, (a[1]+b[1])/2)

WHEEL_XY = {
    "FR": mid(OCT_XY[0], OCT_XY[7]),
    "FL": mid(OCT_XY[1], OCT_XY[2]),
    "BL": mid(OCT_XY[3], OCT_XY[4]),
    "BR": mid(OCT_XY[5], OCT_XY[6]),
}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]


# ============================================================================
# Source wheel data reader
# ============================================================================

def classify_part(name):
    if "Kaya_Wheel_Hub" in name:
        return "hub"
    if "_17_4775_001_" in name or "_17_4775_003_" in name:
        return "sideplate"
    if "_17_4775_004_" in name:
        return "roller_barrel"
    if "_17_4775_005_" in name:
        return "roller_cap"
    if "_17_4775_007_" in name:
        return "roller_pin"
    if "_17_2584_007_" in name:
        return "screw_large"
    if "_17_2584_009_" in name:
        return "screw_small"
    return "unknown"


def read_source_wheel():
    """Read all mesh data from omniwheels.usd, classify parts."""
    print(f"Reading wheel source: {OMNIWHEEL_USD}", flush=True)
    src = Usd.Stage.Open(OMNIWHEEL_USD)
    xfc = UsdGeom.XformCache()

    parts = {"hub": [], "sideplate": [], "roller_barrel": [], "roller_cap": [],
             "roller_pin": [], "screw_large": [], "screw_small": []}

    for prim in src.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        raw_pts = mesh.GetPointsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        if not raw_pts or len(raw_pts) == 0:
            continue

        wxf = xfc.GetLocalToWorldTransform(prim)
        world_pts = []
        for p in raw_pts:
            wp = wxf.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
            world_pts.append([wp[0], wp[1], wp[2]])

        cat = classify_part(prim.GetName())
        if cat == "unknown":
            continue
        parts[cat].append({
            "name": prim.GetName(),
            "points": np.array(world_pts),
            "indices": list(indices) if indices else [],
            "counts": list(counts) if counts else [],
        })

    # Wheel center from roller barrel positions
    barrel_centers = [p["points"].mean(axis=0) for p in parts["roller_barrel"]]
    wheel_center = np.mean(barrel_centers, axis=0)

    for cat, lst in parts.items():
        print(f"  {cat}: {len(lst)} meshes", flush=True)
    print(f"  Wheel center: ({wheel_center[0]*1000:.2f}, {wheel_center[1]*1000:.2f}, "
          f"{wheel_center[2]*1000:.2f}) mm", flush=True)

    return parts, wheel_center


def create_mesh_prim(stage, path, part, src_center, sx, sy, sz,
                     body_offset=None, color=None):
    """Create a mesh prim with scaled, centered vertices."""
    scaled = []
    for p in part["points"]:
        x = float((p[0] - src_center[0]) * sx)
        y = float((p[1] - src_center[1]) * sy)
        z = float((p[2] - src_center[2]) * sz)
        if body_offset is not None:
            x -= body_offset[0]
            y -= body_offset[1]
            z -= body_offset[2]
        scaled.append(Gf.Vec3f(x, y, z))

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(scaled))
    if part["indices"]:
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(part["indices"]))
    if part["counts"]:
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(part["counts"]))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    if color:
        mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


# ============================================================================
# Scene builders
# ============================================================================

def build_physics_scene(stage):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateSolverTypeAttr("TGS")
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    # GPU physics unstable with 44-DOF articulation (PhysX GPU limitation).
    # Using CPU solver. Still GPU-accelerated rendering.
    print(f"Physics: {PHYSICS_HZ}Hz, CPU dynamics (44 DOF articulation), TGS solver", flush=True)


def build_ground(stage):
    gs = 10.0
    gp = UsdGeom.Mesh.Define(stage, "/World/Ground")
    gp.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-gs, -gs, 0), Gf.Vec3f(gs, -gs, 0),
        Gf.Vec3f(gs, gs, 0), Gf.Vec3f(-gs, gs, 0)]))
    gp.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    gp.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    gp.GetSubdivisionSchemeAttr().Set("none")
    gp.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.7, 0.7, 0.7)]))
    UsdPhysics.CollisionAPI.Apply(gp.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(gp.GetPrim()).GetApproximationAttr().Set("none")

    gmat = UsdShade.Material.Define(stage, "/World/GroundMat")
    gpm = UsdPhysics.MaterialAPI.Apply(gmat.GetPrim())
    gpm.CreateStaticFrictionAttr(FRICTION_MU)
    gpm.CreateDynamicFrictionAttr(FRICTION_MU)
    gpm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(gp.GetPrim()).Bind(
        gmat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # Grid lines
    for i in range(11):
        idx = i - 5
        for axis in ["x", "y"]:
            lp = f"/World/Grid/{axis}{i}"
            line = UsdGeom.Cube.Define(stage, lp)
            line.GetSizeAttr().Set(1.0)
            lxf = UsdGeom.Xformable(line.GetPrim())
            lxf.ClearXformOpOrder()
            if axis == "x":
                lxf.AddTranslateOp().Set(Gf.Vec3d(idx * 0.5, 0, 0.001))
                lxf.AddScaleOp().Set(Gf.Vec3d(0.002, 10.0, 0.001))
            else:
                lxf.AddTranslateOp().Set(Gf.Vec3d(0, idx * 0.5, 0.001))
                lxf.AddScaleOp().Set(Gf.Vec3d(10.0, 0.002, 0.001))
            line.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.5, 0.5, 0.5)]))

    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)


def build_arched_chassis(stage, path, half_h, color):
    """Octagonal chassis with recessed belly — motor mount flats at front/rear,
    raised arch center for step clearance. Matches CAD side-view profile."""
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half
    ramp_end = SL - MOTOR_MOUNT_LEN

    z_low = -half_h                    # motor mount bottom (original)
    z_high = -half_h + BELLY_HEIGHT    # raised belly center

    # X stations for bottom profile
    x_stations = sorted(set([
        -SL, -(SL - C),
        -ramp_end, -ramp_start,
        0.0,
        ramp_start, ramp_end,
        (SL - C), SL,
    ]))

    def bottom_z(x):
        ax = abs(x)
        if ax <= ramp_start:
            return z_high
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)
        else:
            return z_low

    def half_width_at(x):
        ax = abs(x)
        if ax <= (SL - C):
            return SW
        else:
            t = (ax - (SL - C)) / C
            return SW - t * C

    verts = []
    face_indices = []
    face_counts = []

    sections = []
    for x in x_stations:
        hw = half_width_at(x)
        bz = bottom_z(x)
        tl = Gf.Vec3f(x, hw, half_h)
        tr = Gf.Vec3f(x, -hw, half_h)
        br = Gf.Vec3f(x, -hw, bz)
        bl = Gf.Vec3f(x, hw, bz)
        idx_start = len(verts)
        verts.extend([tl, tr, br, bl])
        sections.append((idx_start, idx_start + 1, idx_start + 2, idx_start + 3))

    for i in range(len(sections) - 1):
        tl0, tr0, br0, bl0 = sections[i]
        tl1, tr1, br1, bl1 = sections[i + 1]
        face_indices.extend([tl0, tl1, tr1, tr0]); face_counts.append(4)
        face_indices.extend([bl0, br0, br1, bl1]); face_counts.append(4)
        face_indices.extend([tl0, bl0, bl1, tl1]); face_counts.append(4)
        face_indices.extend([tr0, tr1, br1, br0]); face_counts.append(4)

    tl, tr, br, bl = sections[-1]
    face_indices.extend([tl, tr, br, bl]); face_counts.append(4)
    tl, tr, br, bl = sections[0]
    face_indices.extend([tr, tl, bl, br]); face_counts.append(4)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def build_chassis(stage, chassis_path):
    """Arched chassis with chamfered corners and recessed belly."""
    half_h = CHASSIS_H / 2.0

    chassis_mesh = build_arched_chassis(stage, chassis_path + "/body",
                                        half_h, (0.25, 0.25, 0.35))
    UsdPhysics.CollisionAPI.Apply(chassis_mesh.GetPrim())
    # convexDecomposition preserves the belly recess (convexHull fills it in)
    UsdPhysics.MeshCollisionAPI.Apply(chassis_mesh.GetPrim()).CreateApproximationAttr("convexDecomposition")

    # Forward arrow indicator
    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim())
    axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))


def build_omniwheel(stage, wheel_path, src_parts, src_center, wheel_mat, wname):
    """
    Build one complete omni wheel from source mesh data.

    The Wheel Xform (wheel_path) already has translate + rotateZ set by caller.
    Hub is at identity (inherits wheel transform).
    Rollers are at their local offsets within the wheel frame.
    """
    hub_path = wheel_path + "/Hub"
    hub_xf = UsdGeom.Xform.Define(stage, hub_path)
    UsdPhysics.RigidBodyAPI.Apply(hub_xf.GetPrim())
    UsdPhysics.MassAPI.Apply(hub_xf.GetPrim()).CreateMassAttr(HUB_MASS)
    # Hub at identity -- inherits wheel_path transform

    # Hub visual meshes (hub flanges + sideplates + screws only — NOT caps/pins)
    hub_color = (0.3, 0.3, 0.35)
    mi = 0
    for cat in ["hub", "sideplate", "screw_large", "screw_small"]:
        for part in src_parts[cat]:
            mp = f"{hub_path}/vis_{mi}"
            create_mesh_prim(stage, mp, part, src_center,
                             SCALE_XZ, SCALE_Y, SCALE_XZ, color=hub_color)
            mi += 1

    # Pre-compute cap/pin centers for matching to rollers
    cap_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_cap"]]
    pin_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_pin"]]
    used_caps = set()
    used_pins = set()

    # Roller bodies -- each at its local offset in the wheel frame
    ENABLE_ROLLERS = True  # Set False to test without rollers
    if not ENABLE_ROLLERS:
        # Fallback: simple cylinder collider on hub
        hub_cyl = UsdGeom.Cylinder.Define(stage, hub_path + "/hub_collider")
        hub_cyl.GetRadiusAttr().Set(float(WHEEL_RADIUS))
        hub_cyl.GetHeightAttr().Set(float(WHEEL_WIDTH * 0.9))
        hub_cyl.GetAxisAttr().Set("Y")
        UsdPhysics.CollisionAPI.Apply(hub_cyl.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(hub_cyl.GetPrim()).Bind(
            wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        hub_cyl.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.5, 0.5, 0.5)]))
    # Pre-compute all roller scaled centers to find safe collision radius
    roller_color = (0.15, 0.15, 0.15)
    all_scaled_centers = []
    for barrel in src_parts["roller_barrel"]:
        bc = barrel["points"].mean(axis=0)
        sc = np.array([
            float((bc[0] - src_center[0]) * SCALE_XZ),
            float((bc[1] - src_center[1]) * SCALE_Y),
            float((bc[2] - src_center[2]) * SCALE_XZ),
        ])
        all_scaled_centers.append(sc)

    # Find minimum pairwise distance between any two rollers
    min_pair_dist = float('inf')
    for i in range(len(all_scaled_centers)):
        for j in range(i + 1, len(all_scaled_centers)):
            d = float(np.linalg.norm(all_scaled_centers[i] - all_scaled_centers[j]))
            if d < min_pair_dist:
                min_pair_dist = d
    # Safe collision sphere radius: half min distance minus gap
    safe_roller_r = min_pair_dist / 2.0 - 0.003
    # Also can't exceed distance to wheel surface
    max_surface_r = float(WHEEL_RADIUS - np.sqrt(
        all_scaled_centers[0][0]**2 + all_scaled_centers[0][2]**2))
    roller_coll_r_global = max(min(safe_roller_r, max_surface_r), 0.005)  # floor at 5mm
    print(f"  Roller collision: min_pair_dist={min_pair_dist*1000:.1f}mm, "
          f"safe_r={safe_roller_r*1000:.1f}mm, surface_r={max_surface_r*1000:.1f}mm, "
          f"final_r={roller_coll_r_global*1000:.1f}mm", flush=True)

    for ri, barrel in enumerate(src_parts["roller_barrel"]):
        if not ENABLE_ROLLERS:
            break
        bc = barrel["points"].mean(axis=0)
        scaled_center = all_scaled_centers[ri]

        # Roller as separate rigid body with revolute joint
        rp = wheel_path + f"/Roller_{ri}"
        rxf = UsdGeom.Xform.Define(stage, rp)
        rprim = rxf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(rprim)
        UsdPhysics.MassAPI.Apply(rprim).CreateMassAttr(ROLLER_MASS)
        # Solver tuning for roller stability
        prb = PhysxSchema.PhysxRigidBodyAPI.Apply(rprim)
        prb.CreateSolverPositionIterationCountAttr(16)
        prb.CreateMaxDepenetrationVelocityAttr(5.0)

        # Position roller at its center in wheel frame
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))

        # Roller collision: sphere at roller center
        # GPU PhysX limits convex hulls to 64 verts -- mesh collision fails.
        # Sphere gives correct roller physics via the revolute joint.
        # Size: must not overlap adjacent rollers AND must reach wheel surface
        roller_center_dist = float(np.sqrt(scaled_center[0]**2 + scaled_center[2]**2))
        # Max radius before adjacent rollers overlap:
        max_nonoverlap_r = roller_center_dist * math.sin(math.pi / NUM_ROLLERS) - 0.002
        # Radius to just reach wheel surface:
        surface_r = float(WHEEL_RADIUS - roller_center_dist)
        roller_coll_r = min(max_nonoverlap_r, surface_r)

        bmp = rp + "/collider"
        sphere = UsdGeom.Sphere.Define(stage, bmp)
        sphere.GetRadiusAttr().Set(roller_coll_r_global)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
            wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        UsdGeom.Imageable(sphere.GetPrim()).CreatePurposeAttr("guide")
        pcoll = PhysxSchema.PhysxCollisionAPI.Apply(sphere.GetPrim())
        pcoll.CreateContactOffsetAttr(0.001)
        pcoll.CreateRestOffsetAttr(0.0005)

        # Visual barrel mesh (no collision) — real Kaya geometry, uniform scale
        bvp = rp + "/barrel_vis"
        create_mesh_prim(stage, bvp, barrel, src_center,
                         SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                         body_offset=scaled_center, color=roller_color)

        # Match closest cap and pin to this roller and parent to roller body
        for label, data_list, used_set in [("cap", cap_data, used_caps),
                                            ("pin", pin_data, used_pins)]:
            best_idx, best_dist = -1, float('inf')
            for idx, (part, center) in enumerate(data_list):
                if idx in used_set:
                    continue
                d = np.linalg.norm(bc - center)
                if d < best_dist:
                    best_dist = d
                    best_idx = idx
            if best_idx >= 0:
                used_set.add(best_idx)
                part = data_list[best_idx][0]
                vp = rp + f"/{label}_vis"
                create_mesh_prim(stage, vp, part, src_center,
                                 SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                                 body_offset=scaled_center, color=hub_color)

        # Revolute joint: roller spins around barrel axis (tangent to wheel)
        # Kaya URDF uses: joint axis=Z, origin rpy=(0, -theta, 0)
        # where theta is the roller's angular position around the wheel.
        # This rotates the joint frame about Y by -theta, so the Z axis
        # (revolute axis) points along the tangent direction in the XZ plane.
        dx = bc[0] - src_center[0]
        dz = bc[2] - src_center[2]
        theta = math.atan2(dz, dx)

        jp = rp + "/joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.CreateBody0Rel().SetTargets([hub_path])
        joint.CreateBody1Rel().SetTargets([rp])

        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        # Rotate joint frame about Y by -theta (same as Kaya URDF rpy=(0,-theta,0))
        # This aligns the Z axis with the roller's barrel tangent direction
        half_a = float(-theta / 2.0)
        qy = Gf.Quatf(float(math.cos(half_a)), 0.0, float(math.sin(half_a)), 0.0)
        joint.CreateLocalRot0Attr().Set(qy)
        joint.CreateLocalRot1Attr().Set(qy)
        joint.CreateAxisAttr("Z")

        # Minimal bearing friction so rollers don't spin forever at rest
        # Kaya removes DriveAPI entirely, but that causes perpetual spin from settling
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.001)  # tiny bearing friction

    return hub_path


# ============================================================================
# Arm build helpers (Sim3 phase 1, vertical stowed zigzag)
# ============================================================================

def _apply_mass_box_vertical(prim, mass, com_z, length_z):
    """MassAPI for a uniform box aligned along +Z.

    Body frame: +Z is the link long axis. Box extents are
    (LINK_BOX_T, LINK_BOX_W, length_z); COM at (0, 0, com_z).
    """
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, float(com_z)))
    Lx, Ly, Lz = LINK_BOX_T, LINK_BOX_W, float(length_z)
    Ixx = mass / 12.0 * (Ly * Ly + Lz * Lz)
    Iyy = mass / 12.0 * (Lx * Lx + Lz * Lz)
    Izz = mass / 12.0 * (Lx * Lx + Ly * Ly)
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixx, Iyy, Izz))
    # Isaac 5.1 asset validator requires principalAxes.
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _apply_mass_box_horizontal_x(prim, mass, length_x, dir_sign):
    """MassAPI for a uniform box aligned along body-local +/- X axis.

    Used for the horizontal stowed arm links (L2, L3). The body box extents
    are (length_x, LINK_BOX_W, LINK_BOX_T) with the link extending from the
    body origin along +X (dir_sign = +1) or -X (dir_sign = -1). COM sits at
    the link midpoint.
    """
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    com_x = dir_sign * length_x / 2.0
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0.0, 0.0))
    Lx, Ly, Lz = float(length_x), LINK_BOX_W, LINK_BOX_T
    Ixx = mass / 12.0 * (Ly * Ly + Lz * Lz)
    Iyy = mass / 12.0 * (Lx * Lx + Lz * Lz)
    Izz = mass / 12.0 * (Lx * Lx + Ly * Ly)
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixx, Iyy, Izz))
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _apply_mass_link_plus_tip_x(prim, link_mass, tip_mass, link_length, link_dir_sign):
    """Uniform link box along +/- X with a point mass at the link tip.

    Used for the wrist body (L4 + Orbbec camera at the tip). link_dir_sign
    = +1 means the link extends in body-local +X; -1 means -X.
    """
    total = link_mass + tip_mass
    link_cx = link_dir_sign * link_length / 2.0
    tip_cx  = link_dir_sign * link_length
    com_x   = (link_mass * link_cx + tip_mass * tip_cx) / total

    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0.0, 0.0))

    Lx, Ly, Lz = float(link_length), LINK_BOX_W, LINK_BOX_T
    Ixx_link_own = link_mass / 12.0 * (Ly * Ly + Lz * Lz)
    Iyy_link_own = link_mass / 12.0 * (Lx * Lx + Lz * Lz)
    Izz_link_own = link_mass / 12.0 * (Lx * Lx + Ly * Ly)
    d_link = link_cx - com_x
    Iyy_link = Iyy_link_own + link_mass * d_link * d_link
    Izz_link = Izz_link_own + link_mass * d_link * d_link
    d_tip = tip_cx - com_x
    Iyy_tip = tip_mass * d_tip * d_tip
    Izz_tip = tip_mass * d_tip * d_tip

    Ixx_tot = Ixx_link_own  # point mass on the X axis contributes zero Ixx
    Iyy_tot = Iyy_link + Iyy_tip
    Izz_tot = Izz_link + Izz_tip
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixx_tot, Iyy_tot, Izz_tot))
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _apply_mass_link_plus_tip_z(prim, link_mass, tip_mass, link_length, link_dir_sign):
    """Uniform link box along +/- Z with a point mass at the link tip.

    link_dir_sign = +1 means the link extends in body-local +Z; -1 means -Z.
    Used for the wrist body (link + Orbbec camera at the tip).
    """
    total = link_mass + tip_mass
    link_cz = link_dir_sign * link_length / 2.0
    tip_cz  = link_dir_sign * link_length
    com_z   = (link_mass * link_cz + tip_mass * tip_cz) / total

    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, float(com_z)))

    Lx, Ly, Lz = LINK_BOX_T, LINK_BOX_W, float(link_length)
    Ixx_link_own = link_mass / 12.0 * (Ly * Ly + Lz * Lz)
    Iyy_link_own = link_mass / 12.0 * (Lx * Lx + Lz * Lz)
    Izz_link_own = link_mass / 12.0 * (Lx * Lx + Ly * Ly)
    # Parallel axis shift from (0, 0, link_cz) to composite COM at (0, 0, com_z).
    d_link = link_cz - com_z
    Ixx_link = Ixx_link_own + link_mass * d_link * d_link
    Iyy_link = Iyy_link_own + link_mass * d_link * d_link
    # Point-mass tip at (0, 0, tip_cz), parallel axis to composite COM.
    d_tip = tip_cz - com_z
    Ixx_tip = tip_mass * d_tip * d_tip
    Iyy_tip = tip_mass * d_tip * d_tip

    Ixx_tot = Ixx_link + Ixx_tip
    Iyy_tot = Iyy_link + Iyy_tip
    Izz_tot = Izz_link_own  # point mass + box rotation about z axis is just the box
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixx_tot, Iyy_tot, Izz_tot))
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _add_arm_link_visual(stage, path, center, size, color):
    """Cosmetic box (no collision) for an arm link."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def _add_arm_joint(stage, joint_path, body0, body1, localPos0, axis, limit_deg):
    """Revolute joint + position-mode DriveAPI. Child localPos1 = origin."""
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*localPos0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateAxisAttr(axis)
    lo, hi = limit_deg
    joint.CreateLowerLimitAttr(float(lo))
    joint.CreateUpperLimitAttr(float(hi))

    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateStiffnessAttr(ARM_STIFFNESS_NM_PER_DEG)
    drive.CreateDampingAttr(ARM_DAMPING_NMS_PER_DEG)
    drive.CreateMaxForceAttr(ARM_MAX_FORCE_NM)
    drive.CreateTargetPositionAttr(0.0)


def build_arm(stage, robot_path, chassis_path):
    """Build the 4-DOF arm as additional rigid bodies in the chassis articulation.

    Flat stowed pose at q = (0, 0, 0, 0): arm lays folded on top of the chassis
    with the yaw base at the back edge of the chassis and the links stacked
    parallel to the chassis top surface.

      - J1_base     : vertical 5" motor stack from chassis top  (link +Z)
      - J2_shoulder : L2 22.68" horizontal, +X toward chassis front (link +X)
      - J3_elbow    : L3 19.69" horizontal, folded -X toward back  (link -X)
      - J4_wrist+cam: L4  5.91" horizontal, folded +X back forward (link +X)

    L2/L3/L4 are co-located in Y but stacked in Z by VIS_Z_STACK (~1.8") per
    link to represent the motor bracket offsets between folded links.

    Joint pivots in CHASSIS frame (verification):
      J1:  (ARM_MOUNT_X,                         0, ARM_MOUNT_Z                    )
      J2:  (ARM_MOUNT_X,                         0, ARM_MOUNT_Z + L_J1             )   top of J1
      J3:  (ARM_MOUNT_X + L_L2,                  0, ARM_MOUNT_Z + L_J1 +   VIS     )   L2 tip + bracket
      J4:  (ARM_MOUNT_X + L_L2 - L_L3,           0, ARM_MOUNT_Z + L_J1 + 2 VIS     )   L3 tip + bracket
      tip: (ARM_MOUNT_X + L_L2 - L_L3 + L_L4,    0, ARM_MOUNT_Z + L_J1 + 2 VIS     )   camera

    Returns the list of arm joint DOF names in creation order.
    """
    j1_base = robot_path + "/ArmJ1base"
    j2_shd  = robot_path + "/ArmJ2shoulder"
    j3_elb  = robot_path + "/ArmJ3elbow"
    j4_wrs  = robot_path + "/ArmJ4wrist"

    z_top_j1  = ARM_MOUNT_Z + L_J1                  # J2 pivot height
    z_l3_body = z_top_j1 + VIS_Z_STACK              # L3 body plane (bracket above L2)
    z_l4_body = z_top_j1 + 2.0 * VIS_Z_STACK        # L4 body plane (bracket above L3)

    # ---- Body 1: J1 base (vertical 5" motor stack, link along +Z) ----
    UsdGeom.Xform.Define(stage, j1_base)
    p1 = stage.GetPrimAtPath(j1_base)
    xf1 = UsdGeom.Xformable(p1); xf1.ClearXformOpOrder()
    xf1.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z))
    UsdPhysics.RigidBodyAPI.Apply(p1)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p1).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_box_vertical(p1, mass=M_J1_BASE,
                             com_z=L_J1 / 2.0, length_z=L_J1)
    _add_arm_link_visual(stage, j1_base + "/vis",
                         center=(0.0, 0.0, L_J1 / 2.0),
                         size=(0.050, 0.050, L_J1),
                         color=(0.55, 0.55, 0.60))

    # ---- Body 2: J2 shoulder (L2 22.68", horizontal link along +X) ----
    UsdGeom.Xform.Define(stage, j2_shd)
    p2 = stage.GetPrimAtPath(j2_shd)
    xf2 = UsdGeom.Xformable(p2); xf2.ClearXformOpOrder()
    xf2.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, z_top_j1))
    UsdPhysics.RigidBodyAPI.Apply(p2)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p2).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_box_horizontal_x(p2, mass=M_J2_SHOULDER,
                                 length_x=L_L2, dir_sign=+1)
    _add_arm_link_visual(stage, j2_shd + "/vis",
                         center=(+L_L2 / 2.0, 0.0, 0.0),
                         size=(L_L2, LINK_BOX_W, LINK_BOX_T),
                         color=(0.85, 0.30, 0.25))

    # ---- Body 3: J3 elbow (L3 19.69", horizontal link along -X, stacked above L2) ----
    UsdGeom.Xform.Define(stage, j3_elb)
    p3 = stage.GetPrimAtPath(j3_elb)
    xf3 = UsdGeom.Xformable(p3); xf3.ClearXformOpOrder()
    xf3.AddTranslateOp().Set(Gf.Vec3d(
        ARM_MOUNT_X + L_L2, ARM_MOUNT_Y, z_l3_body))
    UsdPhysics.RigidBodyAPI.Apply(p3)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p3).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_box_horizontal_x(p3, mass=M_J3_ELBOW,
                                 length_x=L_L3, dir_sign=-1)
    _add_arm_link_visual(stage, j3_elb + "/vis",
                         center=(-L_L3 / 2.0, 0.0, 0.0),
                         size=(L_L3, LINK_BOX_W, LINK_BOX_T),
                         color=(0.25, 0.75, 0.30))

    # ---- Body 4: J4 wrist + camera (L4 5.91", horizontal +X, stacked above L3) ----
    UsdGeom.Xform.Define(stage, j4_wrs)
    p4 = stage.GetPrimAtPath(j4_wrs)
    xf4 = UsdGeom.Xformable(p4); xf4.ClearXformOpOrder()
    xf4.AddTranslateOp().Set(Gf.Vec3d(
        ARM_MOUNT_X + L_L2 - L_L3, ARM_MOUNT_Y, z_l4_body))
    UsdPhysics.RigidBodyAPI.Apply(p4)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p4).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_link_plus_tip_x(p4,
                                link_mass=M_L4_LINK,
                                tip_mass=M_CAMERA,
                                link_length=L_L4,
                                link_dir_sign=+1)
    _add_arm_link_visual(stage, j4_wrs + "/vis",
                         center=(+L_L4 / 2.0, 0.0, 0.0),
                         size=(L_L4, LINK_BOX_W, LINK_BOX_T),
                         color=(0.25, 0.30, 0.85))
    # Camera visual (sphere at L4 tip; mass already lumped into the wrist body).
    cam = UsdGeom.Sphere.Define(stage, j4_wrs + "/camera")
    cam.GetRadiusAttr().Set(0.030)
    cxf = UsdGeom.Xformable(cam.GetPrim()); cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(L_L4, 0.0, 0.0))
    cam.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.05, 0.05, 0.05)]))

    # ---- Joints ----
    # J1: chassis -> J1base, axis Z (yaw). Pivot at the mount point on chassis top.
    _add_arm_joint(stage, robot_path + "/ArmJ1",
                   body0=chassis_path, body1=j1_base,
                   localPos0=(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z),
                   axis="Z", limit_deg=ARM_LIMITS_DEG["ArmJ1"])

    # J2: J1base -> J2shoulder, axis Y (shoulder pitch). Pivot at top of J1.
    _add_arm_joint(stage, robot_path + "/ArmJ2",
                   body0=j1_base, body1=j2_shd,
                   localPos0=(0.0, 0.0, L_J1),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ2"])

    # J3: J2shoulder -> J3elbow, axis Y (elbow pitch). Pivot at L2 tip, offset
    # up by VIS_Z_STACK to model the motor bracket that holds L3 above L2.
    _add_arm_joint(stage, robot_path + "/ArmJ3",
                   body0=j2_shd, body1=j3_elb,
                   localPos0=(L_L2, 0.0, VIS_Z_STACK),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ3"])

    # J4: J3elbow -> J4wrist, axis Y (wrist pitch). Pivot at L3 tip, offset
    # up by VIS_Z_STACK to model the bracket that holds L4 above L3.
    _add_arm_joint(stage, robot_path + "/ArmJ4",
                   body0=j3_elb, body1=j4_wrs,
                   localPos0=(-L_L3, 0.0, VIS_Z_STACK),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ4"])

    return ["ArmJ1", "ArmJ2", "ArmJ3", "ArmJ4"]


def arm_tip_world(stage):
    """Arm-tip world position for diagnostics."""
    prim = stage.GetPrimAtPath("/World/Robot/ArmJ4wrist")
    if not prim.IsValid():
        return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # Link tip = (+L_L4, 0, 0) in wrist body local frame (horizontal stowed).
    return mat.Transform(Gf.Vec3d(L_L4, 0.0, 0.0))


def build_robot(stage, src_parts, src_center):
    robot_path = "/World/Robot"
    chassis_path = robot_path + "/Chassis"

    UsdGeom.Xform.Define(stage, robot_path)
    UsdGeom.Xform.Define(stage, chassis_path)

    cp = stage.GetPrimAtPath(chassis_path)
    UsdPhysics.RigidBodyAPI.Apply(cp)
    UsdPhysics.MassAPI.Apply(cp).CreateMassAttr(CHASSIS_MASS)
    UsdPhysics.MassAPI(cp).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(cp)
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(cp)
    # Arm links have no collision shapes in phase 1, so self-collision between
    # arm and chassis cannot fire. Keep it off explicitly anyway for clarity.
    physx_art.CreateEnabledSelfCollisionsAttr(False)

    build_chassis(stage, chassis_path)

    # Wheel material
    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    # Wheel Z position relative to chassis center
    # On flat ground: chassis center at Z = BELLY_HEIGHT + CHASSIS_H/2
    # Wheel axle at Z = WHEEL_RADIUS (above ground)
    # Wheel Z in chassis frame = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H/2)
    wheel_z = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)

    # Wheel offset: must clear roller orbit radius from chassis
    # Roller centers orbit at ~76mm from wheel hub in the rotation plane.
    # Offset must prevent inner rollers from overlapping chassis collision.
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.04  # half of 51.8mm + 40mm clearance

    drive_joint_paths = []

    for wname in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wname]
        angle_deg = AXLE_ANGLES[wname]
        angle_rad = math.radians(angle_deg)

        # Offset wheel outward along chamfer normal
        norm_len = math.sqrt(cx * cx + cy * cy)
        nx, ny = cx / norm_len, cy / norm_len
        wx = cx + nx * wheel_offset
        wy = cy + ny * wheel_offset

        # Wheel Xform: translate + rotateZ (same pattern as original xdrive_o3dyn.py)
        wp = robot_path + f"/Wheel_{wname}"
        wxf = UsdGeom.Xform.Define(stage, wp)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(float(angle_deg))

        # Build wheel meshes + roller physics (hub at identity, rollers at offsets)
        hub_path = build_omniwheel(stage, wp, src_parts, src_center, wheel_mat, wname)

        # Drive joint: chassis to hub (exact same as original)
        djp = robot_path + f"/DriveJoint_{wname}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, djp)
        joint.CreateBody0Rel().SetTargets([chassis_path])
        joint.CreateBody1Rel().SetTargets([hub_path])

        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(wx), float(wy), float(wheel_z)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        # Joint rotation: align Y axis with wheel axle in chassis frame
        half_a = angle_rad / 2.0
        rot_q = Gf.Quatf(float(math.cos(half_a)), 0.0, 0.0, float(math.sin(half_a)))
        joint.CreateLocalRot0Attr().Set(rot_q)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        joint.CreateAxisAttr("Y")

        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(djp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(DRIVE_DAMPING)
        drive.CreateMaxForceAttr(DRIVE_MAX_FORCE)
        drive.CreateTargetVelocityAttr(0.0)

        # Isaac mecanum wheel attributes (for potential HolonomicController use)
        jprim = stage.GetPrimAtPath(djp)
        jprim.CreateAttribute("isaacmecanumwheel:radius",
                              Sdf.ValueTypeNames.Float).Set(float(WHEEL_RADIUS))
        jprim.CreateAttribute("isaacmecanumwheel:angle",
                              Sdf.ValueTypeNames.Float).Set(90.0)

        drive_joint_paths.append(djp)

    # Arm (Sim3 phase 1): 4 extra rigid bodies + 4 revolute joints inside the
    # same articulation as the chassis and wheels. Must be added BEFORE the
    # spawn transform so the arm bodies inherit the robot_path xform.
    arm_joint_names = build_arm(stage, robot_path, chassis_path)

    # Spawn position (reactor straddle pose from xdrive_realwheel.py)
    global SPAWN_X, SPAWN_Y, spawn_yaw
    import sim_config as cfg
    SPAWN_X = 0.0
    SPAWN_Y = -1.63                   # 64.2" from center (nudged 1.6" outward
                                      # from the realwheel default of -1.59
                                      # to give forward-cantilevered arm
                                      # clearance from the central column)
    SPAWN_FLOOR_Z = cfg.Z_OUTER_IN * IN  # -49.3"
    spawn_z = SPAWN_FLOOR_Z + WHEEL_RADIUS + CHASSIS_H / 2.0 + 0.10
    spawn_yaw = math.degrees(math.atan2(0 - SPAWN_Y, 0 - SPAWN_X))
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, spawn_z))
    rxf.AddRotateZOp().Set(spawn_yaw)
    r_from_origin = math.sqrt(SPAWN_X**2 + SPAWN_Y**2)
    print(f"Reactor spawn: ({SPAWN_X:.4f}, {SPAWN_Y:.4f}, {spawn_z:.4f})m", flush=True)
    print(f"  = ({SPAWN_X/IN:.1f}\", {SPAWN_Y/IN:.1f}\", {spawn_z/IN:.1f}\")"
          f" R={r_from_origin/IN:.1f}\" yaw={spawn_yaw:.1f}deg", flush=True)
    print(f"  wheel_r={WHEEL_RADIUS:.4f}m, half_h={CHASSIS_H/2/IN:.2f}\"", flush=True)

    print(f"\nChassis: {CHASSIS_L/IN:.1f}x{CHASSIS_W/IN:.1f}x{CHASSIS_H/IN:.1f}\" rect, "
          f"{CHASSIS_MASS}kg", flush=True)
    print(f"Belly height: {BELLY_HEIGHT/IN:.2f}\" ({BELLY_HEIGHT*100:.1f}cm)", flush=True)
    print(f"Wheel: {TARGET_DIA_MM:.0f}mm dia, {TARGET_WIDTH_MM:.0f}mm wide, "
          f"{NUM_ROLLERS} real rollers/wheel", flush=True)
    print(f"Wheel Z in chassis frame: {wheel_z/IN:.2f}\"", flush=True)
    print(f"Spawn Z: {spawn_z:.4f}m ({spawn_z/IN:.2f}\")", flush=True)
    print(f"Total bodies: 1 chassis + 4 hubs + {4*NUM_ROLLERS} rollers + 4 arm = "
          f"{1 + 4 + 4*NUM_ROLLERS + 4}", flush=True)
    print(f"Total joints: 4 drive + {4*NUM_ROLLERS} roller + 4 arm = "
          f"{4 + 4*NUM_ROLLERS + 4}", flush=True)

    # Arm stowed pose geometry report (chassis-local, so independent of yaw).
    j1_x   = ARM_MOUNT_X
    j2_x   = ARM_MOUNT_X
    j3_x   = ARM_MOUNT_X + L_L2
    j4_x   = ARM_MOUNT_X + L_L2 - L_L3
    tip_x  = ARM_MOUNT_X + L_L2 - L_L3 + L_L4
    j2_z   = ARM_MOUNT_Z + L_J1
    j3_z   = j2_z + VIS_Z_STACK
    j4_z   = j2_z + 2.0 * VIS_Z_STACK
    chassis_front_x = +CHASSIS_L / 2.0
    l2_overhang = j3_x - chassis_front_x
    print(f"\nArm (flat stowed on chassis top, mount at chassis back edge):", flush=True)
    print(f"  ARM_MOUNT_X = {ARM_MOUNT_X/IN:+.2f}\" (chassis back edge = {-CHASSIS_L/2/IN:+.2f}\")", flush=True)
    print(f"  Chassis X extents: {-CHASSIS_L/2/IN:+.2f}\" to {chassis_front_x/IN:+.2f}\"", flush=True)
    print(f"  J1 pivot (chassis frame) x,z = {j1_x/IN:+.2f}\", {ARM_MOUNT_Z/IN:+.2f}\"", flush=True)
    print(f"  J2 pivot                 x,z = {j2_x/IN:+.2f}\", {j2_z/IN:+.2f}\"", flush=True)
    print(f"  J3 pivot (L2 tip)        x,z = {j3_x/IN:+.2f}\", {j3_z/IN:+.2f}\"", flush=True)
    print(f"  J4 pivot (L3 tip)        x,z = {j4_x/IN:+.2f}\", {j4_z/IN:+.2f}\"", flush=True)
    print(f"  L4 tip (camera)          x,z = {tip_x/IN:+.2f}\", {j4_z/IN:+.2f}\"", flush=True)
    print(f"  L2 forward overhang past chassis front = {l2_overhang/IN:+.2f}\"", flush=True)
    print(f"  Total arm mass = "
          f"{M_J1_BASE + M_J2_SHOULDER + M_J3_ELBOW + M_L4_LINK + M_CAMERA:.3f} kg", flush=True)

    # World-frame radial check (chassis yaw = 90 deg -> chassis +X = world +Y).
    # r(point) = |SPAWN_Y + chassis_x| for y-axis-aligned chassis at SPAWN_X = 0.
    def _r_world_in(cx):
        return abs(SPAWN_Y + cx) / IN
    print(f"  World radial distance from reactor center (yaw=90):", flush=True)
    print(f"    J1 base     r = {_r_world_in(j1_x):5.2f}\"", flush=True)
    print(f"    L2 tip / J3 r = {_r_world_in(j3_x):5.2f}\"  (central column r~38\")", flush=True)
    print(f"    J4 pivot    r = {_r_world_in(j4_x):5.2f}\"", flush=True)
    print(f"    L4 tip      r = {_r_world_in(tip_x):5.2f}\"", flush=True)

    return robot_path, chassis_path, drive_joint_paths, arm_joint_names, spawn_z


# ============================================================================
# X-drive IK
# ============================================================================

def xdrive_ik(vx, vy, omega):
    r = WHEEL_RADIUS
    vels = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        vwx = vx - omega * wy
        vwy = vy + omega * wx
        angle_rad = math.radians(AXLE_ANGLES[wn])
        drive_x = math.cos(angle_rad)
        drive_y = math.sin(angle_rad)
        v_drive = vwx * drive_x + vwy * drive_y
        vels.append(v_drive / r)
    return np.array(vels)


# ============================================================================
# Contact reporting
# ============================================================================

def read_chassis_contacts(cs, chassis_path, reactor_prefix="/World/Reactor"):
    """Read raw contact data for chassis touching the reactor mesh."""
    raw = cs.get_rigid_body_raw_data(chassis_path)
    contacts = []
    for c in raw:
        b0 = cs.decode_body_name(int(c["body0"]))
        b1 = cs.decode_body_name(int(c["body1"]))
        other = b1 if chassis_path in b0 else b0
        if not other.startswith(reactor_prefix):
            continue
        pos = (float(c["position"]["x"]), float(c["position"]["y"]), float(c["position"]["z"]))
        nrm = (float(c["normal"]["x"]), float(c["normal"]["y"]), float(c["normal"]["z"]))
        imp = (float(c["impulse"]["x"]), float(c["impulse"]["y"]), float(c["impulse"]["z"]))
        contacts.append({"position": pos, "normal": nrm, "impulse": imp, "other": other})
    return contacts


def get_chassis_orientation(stage, chassis_path):
    """Get yaw, pitch, roll of chassis in degrees."""
    prim = stage.GetPrimAtPath(chassis_path)
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
    pitch = math.degrees(math.atan2(-mat[2][0], math.sqrt(mat[2][1]**2 + mat[2][2]**2)))
    roll = math.degrees(math.atan2(mat[2][1], mat[2][2]))
    return yaw, pitch, roll


def world_to_chassis_local(pos_world, chassis_pos, yaw_rad):
    """Convert world contact point to chassis-local frame (2D, XY only)."""
    dx = pos_world[0] - chassis_pos[0]
    dy = pos_world[1] - chassis_pos[1]
    dz = pos_world[2] - chassis_pos[2]
    # Rotate into chassis frame (undo yaw)
    cos_y = math.cos(-yaw_rad)
    sin_y = math.sin(-yaw_rad)
    lx = dx * cos_y - dy * sin_y
    ly = dx * sin_y + dy * cos_y
    return lx, ly, dz


# ============================================================================
# State reporting
# ============================================================================

def get_state(art, stage, chassis_path):
    prim = stage.GetPrimAtPath(chassis_path)
    if not prim.IsValid():
        return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos": np.array([t[0], t[1], t[2]]), "jv": jv, "je": je}


def print_state(art, stage, chassis_path, frame):
    s = get_state(art, stage, chassis_path)
    if not s:
        print("  No state", flush=True)
        return
    p = s["pos"]
    print(f"\n--- Frame {frame} ---", flush=True)
    print(f"  Pos: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) m", flush=True)
    print(f"  Pos: ({p[0]/IN:.2f}, {p[1]/IN:.2f}, {p[2]/IN:.2f}) in", flush=True)


# ============================================================================
# Keyboard
# ============================================================================

class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.orbit_radius = 1.59  # 62.7" = straddle radius
        self.reset = self.pstate = self.stop = False
        self.orbit = 0
        self.arm_speed = 0.5    # rad/s per held arm key
        self.arm_home = False   # 'H' -> snap all arm targets to 0
        self._keys = set()
        self._inp = carb.input.acquire_input_interface()
        self._kb = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = self._inp.subscribe_to_keyboard_events(self._kb, self._on)

    def _on(self, ev, *a, **kw):
        k = ev.input
        K = carb.input.KeyboardInput
        if ev.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._keys.add(k)
            if k == K.R: self.reset = True
            elif k == K.P: self.pstate = True
            elif k == K.SPACE: self.stop = True
            elif k == K.H: self.arm_home = True
            elif k == K.EQUAL:
                self.speed = min(self.speed + 0.05, 1.0)
                print(f"Drive speed: {self.speed:.2f} m/s")
            elif k == K.MINUS:
                self.speed = max(self.speed - 0.05, 0.05)
                print(f"Drive speed: {self.speed:.2f} m/s")
            elif k == K.RIGHT_BRACKET:
                self.arm_speed = min(self.arm_speed + 0.1, 2.0)
                print(f"Arm speed: {self.arm_speed:.2f} rad/s")
            elif k == K.LEFT_BRACKET:
                self.arm_speed = max(self.arm_speed - 0.1, 0.1)
                print(f"Arm speed: {self.arm_speed:.2f} rad/s")
            elif k == K.O:
                self.orbit = (self.orbit + 2) % 3 - 1
                labels = {-1: "CW", 0: "OFF", 1: "CCW"}
                print(f"Orbit: {labels[self.orbit]}, r={self.orbit_radius:.2f}m")
            elif k == K.KEY_9:
                self.orbit_radius = max(self.orbit_radius - 0.1, 0.2)
                print(f"Orbit radius: {self.orbit_radius:.2f}m")
            elif k == K.KEY_0:
                self.orbit_radius = min(self.orbit_radius + 0.1, 5.0)
                print(f"Orbit radius: {self.orbit_radius:.2f}m")
        elif ev.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys.discard(k)
        return True

    def cmd(self):
        K = carb.input.KeyboardInput
        if self.orbit != 0:
            vy = self.speed * self.orbit
            omega = vy / self.orbit_radius
            return 0.0, vy, omega
        vx = self.speed * (int(K.UP in self._keys) - int(K.DOWN in self._keys))
        vy = self.speed * (int(K.LEFT in self._keys) - int(K.RIGHT in self._keys))
        w = self.rspeed * (int(K.Q in self._keys) - int(K.E in self._keys))
        return vx, vy, w

    def arm_delta(self, dt):
        """Per-joint angular delta (rad) for the 4 arm joints based on held keys.

        Keys: 1/2 = J1 -/+, 3/4 = J2 -/+, 5/6 = J3 -/+, 7/8 = J4 -/+.
        """
        K = carb.input.KeyboardInput
        d = np.zeros(4, dtype=np.float64)
        d[0] = int(K.KEY_2 in self._keys) - int(K.KEY_1 in self._keys)  # J1 yaw
        d[1] = int(K.KEY_4 in self._keys) - int(K.KEY_3 in self._keys)  # J2 shoulder
        d[2] = int(K.KEY_6 in self._keys) - int(K.KEY_5 in self._keys)  # J3 elbow
        d[3] = int(K.KEY_8 in self._keys) - int(K.KEY_7 in self._keys)  # J4 wrist
        return d * self.arm_speed * dt


# ============================================================================
# MAIN
# ============================================================================
print("=" * 60, flush=True)
print("FORTIS Sim3 phase 1 -- X-drive + 4-DOF arm (reactor straddle)", flush=True)
print("=" * 60, flush=True)

# Read source wheel data before setting up the scene
src_parts, src_center = read_source_wheel()

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10):
    app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_physics_scene(stage)

# Reactor environment + bright lighting so the arm is visible against the walls.
print(f"Loading reactor: {REACTOR_USD}", flush=True)
rp = stage.DefinePrim("/World/Reactor", "Xform")
rp.GetReferences().AddReference(REACTOR_USD)
dome = stage.DefinePrim("/World/DomeLight", "DomeLight")
dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)
dome.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 1, 1))
for i, (rx, ry, rz) in enumerate([(45, 0, 0), (-45, 0, 0), (0, 45, 0), (0, -45, 0)]):
    dl = stage.DefinePrim(f"/World/DirLight{i}", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(300.0)
    dl.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 1, 1))
    dlxf = UsdGeom.Xformable(dl); dlxf.ClearXformOpOrder()
    dlxf.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))

robot_path, chassis_path, drive_joint_paths, arm_joint_names, spawn_z = build_robot(
    stage, src_parts, src_center)

for _ in range(20):
    app.update()

world = World(stage_units_in_meters=1.0)

omni.timeline.get_timeline_interface().play()
for _ in range(10):
    app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10):
    world.step(render=not headless)

if not art.is_physics_handle_valid():
    print("FATAL: Articulation physics handle invalid", flush=True)
    app.close()
    sys.exit(1)

ndof = art.num_dof
dof_names = art.dof_names
print(f"\nArticulation: {ndof} DOFs", flush=True)
print(f"DOF names: {dof_names}", flush=True)

# Find drive DOF indices (the 4 wheel drive joints, not roller joints)
drive_dof_indices = []
for djp in drive_joint_paths:
    joint_name = djp.split("/")[-1]  # e.g. "DriveJoint_FR"
    for i, dn in enumerate(dof_names):
        if joint_name in dn or dn == joint_name:
            drive_dof_indices.append(i)
            break

if len(drive_dof_indices) != 4:
    # Fallback: first 4 DOFs if naming doesn't match
    print(f"WARNING: Found {len(drive_dof_indices)} drive DOFs, expected 4", flush=True)
    print(f"Trying to match by scanning DOF names...", flush=True)
    drive_dof_indices = []
    for i, dn in enumerate(dof_names):
        if "Drive" in dn or "drive" in dn:
            drive_dof_indices.append(i)
    if len(drive_dof_indices) != 4:
        print(f"Still found {len(drive_dof_indices)}, using first 4 DOFs", flush=True)
        drive_dof_indices = list(range(4))

print(f"Drive DOF indices: {drive_dof_indices}", flush=True)

# Find arm DOF indices by name (non-deterministic DOF ordering gotcha).
arm_dof_indices = []
for name in arm_joint_names:
    found = -1
    for i, dn in enumerate(dof_names):
        if dn == name or dn.endswith("/" + name) or name in dn:
            found = i
            break
    if found < 0:
        print(f"FATAL: arm joint {name} not found in DOF names {dof_names}", flush=True)
        app.close()
        sys.exit(1)
    arm_dof_indices.append(found)
arm_idx_np = np.array(arm_dof_indices, dtype=np.int64)
print(f"Arm DOF indices: {arm_dof_indices} -> {[dof_names[i] for i in arm_dof_indices]}",
      flush=True)

# Default everything to velocity mode, then flip only the arm DOFs to position.
art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
art.switch_control_mode("position", joint_indices=arm_idx_np)

# Arm position targets buffer (radians). Starts at q=0 = stowed pose.
arm_targets = np.zeros(4, dtype=np.float64)

def _apply_arm_targets():
    """Write the current arm_targets buffer to the 4 arm DOFs of the articulation."""
    if not art.is_physics_handle_valid():
        return
    # With joint_indices, the positions tensor is subset-sized: (num_envs=1, 4).
    art.set_joint_position_targets(
        arm_targets.reshape(1, -1),
        joint_indices=arm_idx_np,
    )

_apply_arm_targets()

# Settle 2s with zero drive + stowed arm.
print("Settling 2s (stowed arm)...", flush=True)
for i in range(2 * PHYSICS_HZ):
    _apply_arm_targets()
    world.step(render=not headless)
    if i % (PHYSICS_HZ // 2) == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"  [{i/PHYSICS_HZ:.1f}s] Z={p[2]:.4f}m ({p[2]/IN:.2f}\") "
                  f"pos=({p[0]/IN:.1f},{p[1]/IN:.1f},{p[2]/IN:.1f})\"", flush=True)

print_state(art, stage, chassis_path, 0)

# ============================================================================
# Spawn self-check (delta-based, 30 frames)
# ============================================================================
def _snap_state():
    s = get_state(art, stage, chassis_path)
    try:
        jp = art.get_joint_positions()
    except Exception:
        jp = None
    return s, (jp.flatten() if jp is not None else np.zeros(ndof))

print("\nSpawn self-check (30 frames delta)...", flush=True)
s0, jp0 = _snap_state()
for _ in range(30):
    _apply_arm_targets()
    world.step(render=not headless)
s1, jp1 = _snap_state()

arm_delta_check = jp1[arm_idx_np] - jp0[arm_idx_np]
wheel_delta_check = jp1[np.array(drive_dof_indices)] - jp0[np.array(drive_dof_indices)]
lat_drift = 0.0
if s0 and s1:
    lat_drift = math.sqrt((s1["pos"][0] - s0["pos"][0])**2 +
                          (s1["pos"][1] - s0["pos"][1])**2)

arm_ok = np.max(np.abs(arm_delta_check)) < 0.02    # 1.1 deg
wheel_ok = np.max(np.abs(wheel_delta_check)) < 0.05
drift_ok = lat_drift < 0.02

print(f"  arm joint delta (rad):   {arm_delta_check}  -> {'OK' if arm_ok else 'FAIL'}",
      flush=True)
print(f"  wheel joint delta (rad): {wheel_delta_check}  -> {'OK' if wheel_ok else 'FAIL'}",
      flush=True)
print(f"  chassis lateral drift:   {lat_drift*1000:.2f} mm  -> {'OK' if drift_ok else 'FAIL'}",
      flush=True)
tip = arm_tip_world(stage)
if tip is not None:
    r_tip = math.sqrt(tip[0]**2 + tip[1]**2)
    print(f"  arm tip world: ({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}) m  "
          f"r={r_tip:.3f} m ({r_tip/IN:.1f}\")", flush=True)
if not (arm_ok and wheel_ok and drift_ok):
    print("  ** SPAWN COLLISION OR DRIFT DETECTED -- continuing to GUI so you can see it **",
          flush=True)

# ============================================================================
# Headless smoke test (optional) -- just reports status and exits.
# ============================================================================
if headless:
    print("\nHeadless smoke test: 2s hold + report.", flush=True)
    for _ in range(2 * PHYSICS_HZ):
        _apply_arm_targets()
        world.step(render=False)
    print_state(art, stage, chassis_path, 0)
    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)


# ============================================================================
# GUI main loop (drive wheels in velocity mode + arm in position mode)
# ============================================================================
kb = KB()
spawn_pos = [SPAWN_X, SPAWN_Y, spawn_z]
spawn_yaw_reset = spawn_yaw

print("\nControls:", flush=True)
print("  Drive:   Arrows=translate  Q/E=rotate  +/-=drive speed", flush=True)
print("           O=orbit toggle    9/0=orbit radius", flush=True)
print("  Arm:     1/2=J1  3/4=J2  5/6=J3  7/8=J4  (hold to move)", flush=True)
print("           H=home (all arm joints -> 0)   [ / ]=arm speed", flush=True)
print("  Misc:    R=reset  P=print state  Space=stop", flush=True)

# Joint limits in radians (for clamping arm_targets inside the USD limits).
arm_lim_rad = np.array([
    [math.radians(ARM_LIMITS_DEG[n][0]), math.radians(ARM_LIMITS_DEG[n][1])]
    for n in arm_joint_names
])

dt = 1.0 / PHYSICS_HZ
frame = 0
while app.is_running():
    vx, vy, w = kb.cmd()

    # Stop: zero wheel velocity (arm holds its current targets).
    if kb.stop:
        kb.stop = False
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
        vx = vy = w = 0.0

    # Reset: move robot back to spawn pose and re-init the articulation.
    if kb.reset:
        kb.reset = False
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5): app.update()
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos))
        rxf.AddRotateZOp().Set(spawn_yaw_reset)
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(10): world.step(render=True)
        art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
        art.switch_control_mode("position", joint_indices=arm_idx_np)
        arm_targets[:] = 0.0
        _apply_arm_targets()
        print("RESET (arm to stowed)", flush=True)
        continue

    # Arm home: snap all targets to zero.
    if kb.arm_home:
        kb.arm_home = False
        arm_targets[:] = 0.0
        print("Arm HOME -> stowed", flush=True)

    # Integrate held arm keys into target positions, then clamp.
    arm_targets += kb.arm_delta(dt)
    arm_targets = np.clip(arm_targets, arm_lim_rad[:, 0], arm_lim_rad[:, 1])

    # Wheels (velocity mode): write all DOFs at once; non-drive DOFs stay 0.
    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    for ii, di in enumerate(drive_dof_indices):
        va[di] = tv[ii]
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))
        _apply_arm_targets()

    world.step(render=True)
    frame += 1

    if kb.pstate:
        kb.pstate = False
        print_state(art, stage, chassis_path, frame)
        print(f"  Drive cmd: vx={vx:.2f} vy={vy:.2f} w={w:.2f}", flush=True)
        print(f"  Arm targets (deg): "
              f"[{', '.join(f'{math.degrees(x):+.1f}' for x in arm_targets)}]", flush=True)
        tip = arm_tip_world(stage)
        if tip is not None:
            r_tip = math.sqrt(tip[0]**2 + tip[1]**2)
            print(f"  Arm tip: ({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}) m  "
                  f"r={r_tip:.3f} m ({r_tip/IN:.1f}\")", flush=True)

    # Once-per-second chassis + arm-q status line.
    if frame % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            prim = stage.GetPrimAtPath(chassis_path)
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
            try:
                jp_now = art.get_joint_positions().flatten()
                arm_q_deg = [math.degrees(jp_now[i]) for i in arm_dof_indices]
                aq_str = f" arm=[{','.join(f'{x:+.0f}' for x in arm_q_deg)}]"
            except Exception:
                aq_str = ""
            print(f"[{frame/PHYSICS_HZ:.0f}s] pos=({p[0]/IN:.1f},{p[1]/IN:.1f},{p[2]/IN:.1f})\" "
                  f"yaw={yaw:.1f}deg{aq_str}", flush=True)

app.close()

