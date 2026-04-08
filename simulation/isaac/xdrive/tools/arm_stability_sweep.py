"""
FORTIS arm stability sweep — physics-based tipping test on step geometry.

Tests whether the robot tips over on the 4.5" step for each arm pose.
Reads geometry-valid poses from the torque sweep filtered CSV, then
runs Isaac Sim with chassis collision ON to detect actual tipping.

Modes:
  --measure-tilt  Just measure the chassis tilt angle on the step and exit.
  --test          Run 10-20 borderline poses for validation.
  (default)       Full stability sweep of all borderline poses.

Usage:
  IsaacSim\\python.bat tools/arm_stability_sweep.py --36arm --armloaded --measure-tilt
  IsaacSim\\python.bat tools/arm_stability_sweep.py --36arm --armloaded --test
  IsaacSim\\python.bat tools/arm_stability_sweep.py --36arm --armloaded
"""
import os, sys, math, argparse, time, csv, json, datetime
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser(description="FORTIS arm stability sweep — step tipping test")
parser.add_argument("--36arm", action="store_true", dest="arm36", help="36\" arm (default)")
parser.add_argument("--30arm", action="store_true", dest="arm30", help="30\" arm")
parser.add_argument("--24arm", action="store_true", dest="arm24", help="24\" arm")
parser.add_argument("--armloaded", action="store_true")
parser.add_argument("--step", action="store_true", default=True)
parser.add_argument("--reactor", action="store_true")
parser.add_argument("--hz", type=int, default=360)
parser.add_argument("--belly", type=float, default=2.5)
parser.add_argument("--measure-tilt", action="store_true", dest="measure_tilt",
                    help="Just measure chassis tilt on step and exit")
parser.add_argument("--test", action="store_true", help="Run ~20 test poses")
parser.add_argument("--margin-threshold", type=float, default=30.0, dest="margin_mm",
                    help="Analytical margin threshold in mm (default 30)")
parser.add_argument("--settle-time", type=float, default=2.0, dest="settle_s",
                    help="Settle time after teleport in seconds")
parser.add_argument("--hold-time", type=float, default=1.5, dest="hold_s",
                    help="Hold time for tipping detection in seconds")
parser.add_argument("--resume", action="store_true")
args, _ = parser.parse_known_args()

# Always headless
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

# ============================================================================
# Constants
# ============================================================================
IN = 0.0254
MM = 0.001

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ASSETS_DIR = os.path.join(XDRIVE_ROOT, "assets")
RESULTS_ROOT = os.path.join(XDRIVE_ROOT, "results")
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))

OMNIWHEEL_USD = os.path.join(ASSETS_DIR, "omniwheels.usd")
REACTOR_USD = os.path.join(ASSETS_DIR, "diiid_reactor.usd")

# Chassis
CHASSIS_L = 15.354 * IN
CHASSIS_W = 9.353 * IN
CHASSIS_H = 7.1 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
MOTOR_MOUNT_LEN = 2.5 * IN
BELLY_HEIGHT = args.belly * IN
ARCH_FLAT_WIDTH = 3.0 * IN
FRICTION_MU = 0.5

# Wheels
TARGET_DIA_MM = 203.0
TARGET_WIDTH_MM = 51.8
WHEEL_RADIUS = TARGET_DIA_MM / 2.0 * MM
WHEEL_WIDTH = TARGET_WIDTH_MM * MM
SRC_DIA_MM = 82.5
SRC_WIDTH_MM = 44.87
SRC_CENTER_Y_MM = -17.43
SCALE_XZ = TARGET_DIA_MM / SRC_DIA_MM
SCALE_Y = TARGET_WIDTH_MM / SRC_WIDTH_MM
SCALE_UNIFORM = SCALE_XZ

CHASSIS_MASS = 20.4
WHEEL_MASS = 1.0
ROLLER_MASS_FRAC = 0.3
NUM_ROLLERS = 10
HUB_MASS = WHEEL_MASS * (1.0 - ROLLER_MASS_FRAC)
ROLLER_MASS = WHEEL_MASS * ROLLER_MASS_FRAC / NUM_ROLLERS

PHYSICS_HZ = max(360, min(480, args.hz))
DRIVE_DAMPING = 100.0
DRIVE_MAX_FORCE = 200.0

# Chassis geometry
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

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
# Arm constants (configurable)
# ============================================================================
J1_STACK_H = 1.5 * IN

ARM_CONFIGS = {
    36: (17.0, 15.0, 4.0),
    30: (15.0, 12.0, 3.0),
    24: (12.0, 10.0, 2.0),
}

_arm_sel = 36
if args.arm30: _arm_sel = 30
if args.arm24: _arm_sel = 24
if args.arm36: _arm_sel = 36
_l2_in, _l3_in, _l4_in = ARM_CONFIGS[_arm_sel]

L_L2 = _l2_in * IN
L_L3 = _l3_in * IN
L_L4 = _l4_in * IN
ARM_REACH_IN = _l2_in + _l3_in + _l4_in

M_JOINT = 0.629
CF_DENSITY_LB_PER_IN = 0.0053
CF_DENSITY_KG_PER_M = CF_DENSITY_LB_PER_IN * 0.453592 / IN
M_L2 = CF_DENSITY_KG_PER_M * L_L2
M_L3 = CF_DENSITY_KG_PER_M * L_L3
M_L4 = CF_DENSITY_KG_PER_M * L_L4

M_GRIPPER_BARE = 0.500
M_PAYLOAD = 1.361
if args.armloaded:
    M_GRIPPER = M_GRIPPER_BARE + M_PAYLOAD
else:
    M_GRIPPER = M_GRIPPER_BARE
ARM_LOADED = args.armloaded

M_CAMERA = 0.098
CAM_X_ON_L2 = 2.0 * IN
CF_TUBE_SIZE = 0.79 * IN

ARM_MOUNT_X = -CHASSIS_L / 2.0
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0

LINK_Y_SPACING = CF_TUBE_SIZE + 0.005
LINK_Y = [(-1.0) * LINK_Y_SPACING,
          ( 0.0) * LINK_Y_SPACING,
          (+1.0) * LINK_Y_SPACING]

# Arm drive gains — stiff enough for fast convergence during sweep
ARM_STIFFNESS = 400.0  # Nm/deg — high for fast PD tracking
ARM_DAMPING   = 60.0   # Nm*s/deg — high for damping oscillation
ARM_MAX_FORCE = 1500.0  # Nm — high to avoid saturation during dynamic ramp

ARM_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),
    "ArmJ2": (-180.0, 180.0),
    "ArmJ3": (-180.0, 180.0),
    "ArmJ4": (-180.0, 180.0),
}
ARM_JOINT_NAMES = ["ArmJ1", "ArmJ2", "ArmJ3", "ArmJ4"]

# ============================================================================
# Sweep parameters
# ============================================================================
# In USD convention:
#   J2=0: L2 horizontal (+X forward from mount)
#   J3=0: L3 folded back (-X, stowed)   J3=180/-180: L3 extended forward
#   J4=0: L4 extends +X from J4 body
# Grid steps not used (poses read from CSV), but kept for compatibility
J2_STEP, J3_STEP, J4_STEP = 15, 20, 30

J2_MIN, J2_MAX = -30, 150
J3_MIN, J3_MAX = -170, 170
J4_MIN, J4_MAX = -150, 150

# Motion timing (collision-free: teleport + settle, no ramping needed)
SETTLE_FRAMES = 150      # frames to settle after teleport (~417ms at 360Hz)
MEASURE_FRAMES = 36      # frames to measure torques (~100ms)
POSITION_WARN_THRESH = 3.0  # deg — log warning if arm didn't converge

# ============================================================================
# USD/scene builder functions (from arm_pose_sweep.py)
# ============================================================================

def classify_part(name):
    if "Kaya_Wheel_Hub" in name: return "hub"
    if "_17_4775_001_" in name or "_17_4775_003_" in name: return "sideplate"
    if "_17_4775_004_" in name: return "roller_barrel"
    if "_17_4775_005_" in name: return "roller_cap"
    if "_17_4775_007_" in name: return "roller_pin"
    if "_17_2584_007_" in name: return "screw_large"
    if "_17_2584_009_" in name: return "screw_small"
    return "unknown"


def read_source_wheel():
    print(f"Reading wheel source: {OMNIWHEEL_USD}", flush=True)
    src = Usd.Stage.Open(OMNIWHEEL_USD)
    xfc = UsdGeom.XformCache()
    parts = {"hub": [], "sideplate": [], "roller_barrel": [], "roller_cap": [],
             "roller_pin": [], "screw_large": [], "screw_small": []}
    for prim in src.Traverse():
        if prim.GetTypeName() != "Mesh": continue
        mesh = UsdGeom.Mesh(prim)
        raw_pts = mesh.GetPointsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        if not raw_pts or len(raw_pts) == 0: continue
        wxf = xfc.GetLocalToWorldTransform(prim)
        world_pts = [[float(v) for v in wxf.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))]
                     for p in raw_pts]
        cat = classify_part(prim.GetName())
        if cat == "unknown": continue
        parts[cat].append({"name": prim.GetName(), "points": np.array(world_pts),
                           "indices": list(indices) if indices else [],
                           "counts": list(counts) if counts else []})
    barrel_centers = [p["points"].mean(axis=0) for p in parts["roller_barrel"]]
    wheel_center = np.mean(barrel_centers, axis=0)
    for cat, lst in parts.items():
        print(f"  {cat}: {len(lst)} meshes", flush=True)
    return parts, wheel_center


def create_mesh_prim(stage, path, part, src_center, sx, sy, sz,
                     body_offset=None, color=None):
    scaled = []
    for p in part["points"]:
        x = float((p[0] - src_center[0]) * sx)
        y = float((p[1] - src_center[1]) * sy)
        z = float((p[2] - src_center[2]) * sz)
        if body_offset is not None:
            x -= body_offset[0]; y -= body_offset[1]; z -= body_offset[2]
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


def build_physics_scene(stage):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateSolverTypeAttr("TGS")
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    print(f"Physics: {PHYSICS_HZ}Hz, TGS solver", flush=True)


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
    PhysxSchema.PhysxCollisionAPI.Apply(gp.GetPrim())
    PhysxSchema.PhysxCollisionAPI(gp.GetPrim()).CreateContactOffsetAttr(0.002)
    PhysxSchema.PhysxCollisionAPI(gp.GetPrim()).CreateRestOffsetAttr(0.001)
    gmat = UsdShade.Material.Define(stage, "/World/GroundMat")
    gpm = UsdPhysics.MaterialAPI.Apply(gmat.GetPrim())
    gpm.CreateStaticFrictionAttr(FRICTION_MU)
    gpm.CreateDynamicFrictionAttr(FRICTION_MU)
    gpm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(gp.GetPrim()).Bind(
        gmat, UsdShade.Tokens.weakerThanDescendants, "physics")
    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)


def build_step_env(stage):
    step_h = 4.5 * IN
    half = 2.0
    outer = UsdGeom.Mesh.Define(stage, "/World/Step/Outer")
    outer.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, -half*2, 0), Gf.Vec3f(half, -half*2, 0),
        Gf.Vec3f(half, 0, 0), Gf.Vec3f(-half, 0, 0)]))
    outer.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    outer.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    outer.GetSubdivisionSchemeAttr().Set("none")
    outer.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.6, 0.6, 0.65)]))
    UsdPhysics.CollisionAPI.Apply(outer.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(outer.GetPrim()).GetApproximationAttr().Set("none")
    PhysxSchema.PhysxCollisionAPI.Apply(outer.GetPrim())
    PhysxSchema.PhysxCollisionAPI(outer.GetPrim()).CreateContactOffsetAttr(0.002)
    PhysxSchema.PhysxCollisionAPI(outer.GetPrim()).CreateRestOffsetAttr(0.001)

    inner = UsdGeom.Mesh.Define(stage, "/World/Step/Inner")
    inner.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, 0, -step_h), Gf.Vec3f(half, 0, -step_h),
        Gf.Vec3f(half, half*2, -step_h), Gf.Vec3f(-half, half*2, -step_h)]))
    inner.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    inner.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    inner.GetSubdivisionSchemeAttr().Set("none")
    inner.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.5, 0.5, 0.55)]))
    UsdPhysics.CollisionAPI.Apply(inner.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(inner.GetPrim()).GetApproximationAttr().Set("none")
    PhysxSchema.PhysxCollisionAPI.Apply(inner.GetPrim())
    PhysxSchema.PhysxCollisionAPI(inner.GetPrim()).CreateContactOffsetAttr(0.002)
    PhysxSchema.PhysxCollisionAPI(inner.GetPrim()).CreateRestOffsetAttr(0.001)

    face = UsdGeom.Mesh.Define(stage, "/World/Step/Face")
    face.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, 0, -step_h), Gf.Vec3f(half, 0, -step_h),
        Gf.Vec3f(half, 0, 0), Gf.Vec3f(-half, 0, 0)]))
    face.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    face.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    face.GetSubdivisionSchemeAttr().Set("none")
    face.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.4, 0.4, 0.45)]))
    UsdPhysics.CollisionAPI.Apply(face.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(face.GetPrim()).GetApproximationAttr().Set("none")
    PhysxSchema.PhysxCollisionAPI.Apply(face.GetPrim())
    PhysxSchema.PhysxCollisionAPI(face.GetPrim()).CreateContactOffsetAttr(0.002)
    PhysxSchema.PhysxCollisionAPI(face.GetPrim()).CreateRestOffsetAttr(0.001)

    smat = UsdShade.Material.Define(stage, "/World/StepMat")
    spm = UsdPhysics.MaterialAPI.Apply(smat.GetPrim())
    spm.CreateStaticFrictionAttr(FRICTION_MU)
    spm.CreateDynamicFrictionAttr(FRICTION_MU)
    spm.CreateRestitutionAttr(0.05)
    for p in [outer, inner, face]:
        UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(
            smat, UsdShade.Tokens.weakerThanDescendants, "physics")
    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)
    print(f"Step env: step height={step_h/IN:.1f}\"", flush=True)
    return step_h


# ============================================================================
# Arched chassis
# ============================================================================
def build_arched_chassis(stage, path, half_h, color):
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half
    ramp_end = SL - MOTOR_MOUNT_LEN
    z_low = -half_h
    z_high = -half_h + BELLY_HEIGHT

    x_stations = sorted(set([
        -SL, -(SL - C), -ramp_end, -ramp_start, 0.0,
        ramp_start, ramp_end, (SL - C), SL,
    ]))

    def bottom_z(x):
        ax = abs(x)
        if ax <= ramp_start: return z_high
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)
        else: return z_low

    def half_width_at(x):
        ax = abs(x)
        if ax <= (SL - C): return SW
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
        sections.append((idx_start, idx_start+1, idx_start+2, idx_start+3))

    for i in range(len(sections)-1):
        tl0, tr0, br0, bl0 = sections[i]
        tl1, tr1, br1, bl1 = sections[i+1]
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
    half_h = CHASSIS_H / 2.0
    chassis_mesh = build_arched_chassis(stage, chassis_path + "/body", half_h, (0.25, 0.25, 0.35))
    UsdPhysics.CollisionAPI.Apply(chassis_mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(chassis_mesh.GetPrim()).CreateApproximationAttr("convexDecomposition")
    PhysxSchema.PhysxCollisionAPI.Apply(chassis_mesh.GetPrim())
    PhysxSchema.PhysxCollisionAPI(chassis_mesh.GetPrim()).CreateContactOffsetAttr(0.001)
    PhysxSchema.PhysxCollisionAPI(chassis_mesh.GetPrim()).CreateRestOffsetAttr(0.0005)
    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim()); axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))


# ============================================================================
# Omni wheel builder
# ============================================================================
def build_omniwheel(stage, wheel_path, src_parts, src_center, wheel_mat, wname):
    hub_path = wheel_path + "/Hub"
    hub_xf = UsdGeom.Xform.Define(stage, hub_path)
    UsdPhysics.RigidBodyAPI.Apply(hub_xf.GetPrim())
    UsdPhysics.MassAPI.Apply(hub_xf.GetPrim()).CreateMassAttr(HUB_MASS)
    hub_color = (0.3, 0.3, 0.35)
    mi = 0
    for cat in ["hub", "sideplate", "screw_large", "screw_small"]:
        for part in src_parts[cat]:
            mp = f"{hub_path}/vis_{mi}"
            create_mesh_prim(stage, mp, part, src_center,
                             SCALE_XZ, SCALE_Y, SCALE_XZ, color=hub_color)
            mi += 1
    cap_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_cap"]]
    pin_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_pin"]]
    used_caps = set()
    used_pins = set()
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
    mean_radius = np.mean([np.sqrt(c[0]**2 + c[2]**2) for c in all_scaled_centers])
    roller_r = 12.8 * MM * SCALE_XZ / 2.0
    roller_w = 12.5 * MM * SCALE_Y
    for ri, barrel in enumerate(src_parts["roller_barrel"]):
        bc = barrel["points"].mean(axis=0)
        sc = all_scaled_centers[ri]
        rp = f"{wheel_path}/Roller_{ri}"
        rxf = UsdGeom.Xform.Define(stage, rp)
        UsdPhysics.RigidBodyAPI.Apply(rxf.GetPrim())
        UsdPhysics.MassAPI.Apply(rxf.GetPrim()).CreateMassAttr(ROLLER_MASS)
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(float(sc[0]), float(sc[1]), float(sc[2])))
        dx, dz = float(sc[0]), float(sc[2])
        theta = math.atan2(dz, dx)
        cyl = UsdGeom.Capsule.Define(stage, rp + "/col")
        cyl.GetRadiusAttr().Set(float(roller_r))
        cyl.GetHeightAttr().Set(float(roller_w))
        cyl.GetAxisAttr().Set("Y")
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(
            wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        cyl.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*roller_color)]))
        for pidx, (part, pc) in enumerate(cap_data):
            if pidx in used_caps: continue
            dist = np.linalg.norm(bc - pc)
            if dist < 20.0:
                mp = f"{rp}/cap_{pidx}"
                create_mesh_prim(stage, mp, part, src_center,
                                 SCALE_XZ, SCALE_Y, SCALE_XZ,
                                 body_offset=sc, color=roller_color)
                used_caps.add(pidx)
        for pidx, (part, pc) in enumerate(pin_data):
            if pidx in used_pins: continue
            dist = np.linalg.norm(bc - pc)
            if dist < 20.0:
                mp = f"{rp}/pin_{pidx}"
                create_mesh_prim(stage, mp, part, src_center,
                                 SCALE_XZ, SCALE_Y, SCALE_XZ,
                                 body_offset=sc, color=roller_color)
                used_pins.add(pidx)
        jp = rp + "/joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.CreateBody0Rel().SetTargets([hub_path])
        joint.CreateBody1Rel().SetTargets([rp])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(sc[0]), float(sc[1]), float(sc[2])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        half_a = float(-theta / 2.0)
        qy = Gf.Quatf(float(math.cos(half_a)), 0.0, float(math.sin(half_a)), 0.0)
        joint.CreateLocalRot0Attr().Set(qy)
        joint.CreateLocalRot1Attr().Set(qy)
        joint.CreateAxisAttr("Z")
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.001)
    return hub_path


# ============================================================================
# Arm build helpers
# ============================================================================
def _apply_arm_body_mass_x(prim, joint_mass, link_mass, link_length, link_dir,
                           extra_mass=0.0, extra_x=0.0):
    total = joint_mass + link_mass + extra_mass
    link_cx = link_dir * link_length / 2.0
    com_x = (joint_mass * 0.0 + link_mass * link_cx + extra_mass * extra_x) / total
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0.0, 0.0))
    Lx = max(float(link_length), 0.01)
    Ly = CF_TUBE_SIZE; Lz = CF_TUBE_SIZE
    Ixx_link = link_mass / 12.0 * (Ly**2 + Lz**2)
    Iyy_link = link_mass / 12.0 * (Lx**2 + Lz**2)
    Izz_link = link_mass / 12.0 * (Lx**2 + Ly**2)
    d_link = link_cx - com_x
    Iyy_link += link_mass * d_link**2
    Izz_link += link_mass * d_link**2
    d_joint = -com_x
    Iyy_joint = joint_mass * d_joint**2
    Izz_joint = joint_mass * d_joint**2
    d_extra = extra_x - com_x
    Iyy_extra = extra_mass * d_extra**2
    Izz_extra = extra_mass * d_extra**2
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(
        float(Ixx_link),
        float(Iyy_link + Iyy_joint + Iyy_extra),
        float(Izz_link + Izz_joint + Izz_extra)))
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _apply_arm_j1_mass(prim, mass, stack_h):
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, float(stack_h / 2.0)))
    Lx = 0.057; Ly = 0.057; Lz = float(stack_h)
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(
        mass/12.0*(Ly**2+Lz**2), mass/12.0*(Lx**2+Lz**2), mass/12.0*(Lx**2+Ly**2)))
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _add_arm_link_visual(stage, path, center, size, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim()); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def _add_arm_collision_box(stage, path, center, size):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim()); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.GetPurposeAttr().Set(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim()).GetApproximationAttr().Set("boundingCube")
    PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    PhysxSchema.PhysxCollisionAPI(cube.GetPrim()).CreateContactOffsetAttr(0.001)
    PhysxSchema.PhysxCollisionAPI(cube.GetPrim()).CreateRestOffsetAttr(0.0005)


def _add_arm_joint(stage, joint_path, body0, body1, localPos0, axis, limit_deg):
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
    drive.CreateStiffnessAttr(ARM_STIFFNESS)
    drive.CreateDampingAttr(ARM_DAMPING)
    drive.CreateMaxForceAttr(ARM_MAX_FORCE)
    drive.CreateTargetPositionAttr(0.0)


def build_arm(stage, robot_path, chassis_path):
    j1_path = robot_path + "/ArmJ1base"
    j2_path = robot_path + "/ArmJ2shoulder"
    j3_path = robot_path + "/ArmJ3elbow"
    j4_path = robot_path + "/ArmJ4wrist"
    z_arm = ARM_MOUNT_Z + J1_STACK_H
    y_l2, y_l3, y_l4 = LINK_Y
    C_J1 = (0.55, 0.55, 0.60); C_L2 = (0.85, 0.30, 0.25)
    C_L3 = (0.25, 0.75, 0.30); C_L4 = (0.25, 0.30, 0.85)

    # J1 base
    UsdGeom.Xform.Define(stage, j1_path)
    p1 = stage.GetPrimAtPath(j1_path)
    xf1 = UsdGeom.Xformable(p1); xf1.ClearXformOpOrder()
    xf1.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z))
    UsdPhysics.RigidBodyAPI.Apply(p1)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p1).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_j1_mass(p1, mass=M_JOINT, stack_h=J1_STACK_H)
    _add_arm_link_visual(stage, j1_path + "/vis",
                         center=(0.0, 0.0, J1_STACK_H / 2.0),
                         size=(0.057, 0.057, J1_STACK_H), color=C_J1)
    # Collision disabled — raw sweep, filtered in post-processing

    # J2 shoulder (L2, +X)
    UsdGeom.Xform.Define(stage, j2_path)
    p2 = stage.GetPrimAtPath(j2_path)
    xf2 = UsdGeom.Xformable(p2); xf2.ClearXformOpOrder()
    xf2.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, y_l2, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p2)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p2).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p2, joint_mass=M_JOINT, link_mass=M_L2,
                           link_length=L_L2, link_dir=+1,
                           extra_mass=M_CAMERA, extra_x=CAM_X_ON_L2)
    _add_arm_link_visual(stage, j2_path + "/vis",
                         center=(+L_L2/2.0, 0.0, 0.0),
                         size=(L_L2, CF_TUBE_SIZE, CF_TUBE_SIZE), color=C_L2)
    jsph = UsdGeom.Sphere.Define(stage, j2_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L2)]))

    # J3 elbow (L3, -X zigzag)
    UsdGeom.Xform.Define(stage, j3_path)
    p3 = stage.GetPrimAtPath(j3_path)
    xf3 = UsdGeom.Xformable(p3); xf3.ClearXformOpOrder()
    xf3.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2, y_l3, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p3)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p3).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p3, joint_mass=M_JOINT, link_mass=M_L3,
                           link_length=L_L3, link_dir=-1)
    _add_arm_link_visual(stage, j3_path + "/vis",
                         center=(-L_L3/2.0, 0.0, 0.0),
                         size=(L_L3, CF_TUBE_SIZE, CF_TUBE_SIZE), color=C_L3)
    jsph = UsdGeom.Sphere.Define(stage, j3_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L3)]))

    # J4 wrist (L4, +X)
    UsdGeom.Xform.Define(stage, j4_path)
    p4 = stage.GetPrimAtPath(j4_path)
    xf4 = UsdGeom.Xformable(p4); xf4.ClearXformOpOrder()
    xf4.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2 - L_L3, y_l4, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p4)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p4).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p4, joint_mass=M_JOINT, link_mass=M_L4,
                           link_length=L_L4, link_dir=+1,
                           extra_mass=M_GRIPPER, extra_x=L_L4)
    _add_arm_link_visual(stage, j4_path + "/vis",
                         center=(+L_L4/2.0, 0.0, 0.0),
                         size=(L_L4, CF_TUBE_SIZE, CF_TUBE_SIZE), color=C_L4)
    grip = UsdGeom.Cube.Define(stage, j4_path + "/gripper")
    grip.GetSizeAttr().Set(1.0)
    gxf = UsdGeom.Xformable(grip.GetPrim()); gxf.ClearXformOpOrder()
    gxf.AddTranslateOp().Set(Gf.Vec3d(L_L4, 0.0, 0.0))
    gxf.AddScaleOp().Set(Gf.Vec3d(0.030, 0.040, 0.025))
    grip.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.15, 0.15, 0.15)]))
    tip = UsdGeom.Sphere.Define(stage, j4_path + "/tip")
    tip.GetRadiusAttr().Set(0.012)
    txf = UsdGeom.Xformable(tip.GetPrim()); txf.ClearXformOpOrder()
    txf.AddTranslateOp().Set(Gf.Vec3d(L_L4 + 0.015, 0.0, 0.0))
    tip.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.0, 0.0)]))

    # Joints
    _add_arm_joint(stage, robot_path + "/ArmJ1",
                   body0=chassis_path, body1=j1_path,
                   localPos0=(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z),
                   axis="Z", limit_deg=ARM_LIMITS_DEG["ArmJ1"])
    _add_arm_joint(stage, robot_path + "/ArmJ2",
                   body0=j1_path, body1=j2_path,
                   localPos0=(0.0, y_l2, J1_STACK_H),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ2"])
    _add_arm_joint(stage, robot_path + "/ArmJ3",
                   body0=j2_path, body1=j3_path,
                   localPos0=(L_L2, y_l3 - y_l2, 0.0),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ3"])
    _add_arm_joint(stage, robot_path + "/ArmJ4",
                   body0=j3_path, body1=j4_path,
                   localPos0=(-L_L3, y_l4 - y_l3, 0.0),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ4"])

    total_arm_mass = 4 * M_JOINT + M_L2 + M_L3 + M_L4 + M_GRIPPER + M_CAMERA
    load_label = "LOADED" if ARM_LOADED else "UNLOADED"
    print(f"\nARM: {ARM_REACH_IN:.0f}\" reach, {load_label}, "
          f"total mass={total_arm_mass:.3f}kg", flush=True)
    return ARM_JOINT_NAMES


# ============================================================================
# Build full robot
# ============================================================================
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
    physx_art.CreateEnabledSelfCollisionsAttr(False)  # collision-free sweep

    build_chassis(stage, chassis_path)

    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    wheel_z = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.04
    drive_joint_paths = []

    for wname in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wname]
        angle_deg = AXLE_ANGLES[wname]
        angle_rad = math.radians(angle_deg)
        norm_len = math.sqrt(cx**2 + cy**2)
        nx, ny = cx / norm_len, cy / norm_len
        wx = cx + nx * wheel_offset
        wy = cy + ny * wheel_offset
        wp = robot_path + f"/Wheel_{wname}"
        wxf = UsdGeom.Xform.Define(stage, wp)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(float(angle_deg))
        hub_path = build_omniwheel(stage, wp, src_parts, src_center, wheel_mat, wname)
        djp = robot_path + f"/DriveJoint_{wname}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, djp)
        joint.CreateBody0Rel().SetTargets([chassis_path])
        joint.CreateBody1Rel().SetTargets([hub_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(wx), float(wy), float(wheel_z)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
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
        drive_joint_paths.append(djp)

    arm_joint_names = build_arm(stage, robot_path, chassis_path)

    # Spawn height
    spawn_z = BELLY_HEIGHT + CHASSIS_H / 2.0
    p = stage.GetPrimAtPath(robot_path)
    UsdGeom.Xformable(p).ClearXformOpOrder()
    UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    return robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names



# ============================================================================
# Stability-specific helpers
# ============================================================================
SETTLE_FRAMES = 150  # for arm PD convergence during sweep (not used in measure-tilt)
STEP_H = 4.5 * IN

def get_chassis_orientation(stage, chassis_path):
    """Get chassis roll/pitch from rotation matrix."""
    try:
        cprim = stage.GetPrimAtPath(chassis_path)
        cmat = UsdGeom.Xformable(cprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        r = cmat.ExtractRotationMatrix()
        roll_deg = math.degrees(math.atan2(float(r[2][1]), float(r[2][2])))
        pitch_deg = math.degrees(math.asin(max(-1, min(1, -float(r[2][0])))))
        return roll_deg, pitch_deg
    except Exception:
        return 0.0, 0.0


def get_chassis_quaternion(stage, chassis_path):
    """Get chassis orientation as quaternion [w, x, y, z]. Avoids Euler gimbal."""
    try:
        cprim = stage.GetPrimAtPath(chassis_path)
        cmat = UsdGeom.Xformable(cprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        rot = cmat.ExtractRotation()
        q = rot.GetQuat()
        im = q.GetImaginary()
        return np.array([q.GetReal(), im[0], im[1], im[2]], dtype=np.float64)
    except Exception:
        return np.array([1.0, 0.0, 0.0, 0.0])


def quaternion_angle_deg(q1, q2):
    """Angular distance between two unit quaternions, in degrees."""
    dot = np.clip(abs(np.dot(q1, q2)), 0.0, 1.0)
    return 2.0 * math.degrees(math.acos(dot))


def get_wheel_hub_z(stage):
    """Get world-frame Z position of each wheel hub."""
    zs = {}
    for wname in WHEEL_ORDER:
        path = f"/World/Robot/Wheel_{wname}/Hub"
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = mat.ExtractTranslation()
            zs[wname] = float(t[2])
        else:
            zs[wname] = float('nan')
    return zs


def usd_to_fk_angles(j2_usd, j3_usd, j4_usd):
    j2_fk = j2_usd
    j3_fk = j3_usd + 180.0
    if j3_fk > 180.0: j3_fk -= 360.0
    if j3_fk < -180.0: j3_fk += 360.0
    j4_fk = j4_usd - 180.0
    if j4_fk > 180.0: j4_fk -= 360.0
    if j4_fk < -180.0: j4_fk += 360.0
    return j2_fk, j3_fk, j4_fk


# ============================================================================
# Main
# ============================================================================
_arm_sel = 36
if args.arm30: _arm_sel = 30
if args.arm24: _arm_sel = 24
if args.arm36: _arm_sel = 36

env_label = "step"
load_label = "loaded" if args.armloaded else "bare"
config_name = f"{_arm_sel}in_{load_label}_step"

RESULTS_DIR = os.path.join(RESULTS_ROOT, "arm_stability_sweep")
PHYSICS_DIR = os.path.join(RESULTS_DIR, "physics")
os.makedirs(PHYSICS_DIR, exist_ok=True)

csv_path = os.path.join(PHYSICS_DIR, f"{config_name}_stability.csv")
json_path = os.path.join(PHYSICS_DIR, f"{config_name}_stability_summary.json")

# Input: filtered CSV from torque sweep
INPUT_CSV = os.path.join(RESULTS_ROOT, "arm_continuous_sweep", f"{config_name}_filtered.csv")

print(f"\n{'='*60}", flush=True)
print(f"FORTIS Arm Stability Sweep", flush=True)
print(f"Config: {config_name}", flush=True)
print(f"Input: {INPUT_CSV}", flush=True)
print(f"Output: {csv_path}", flush=True)
print(f"{'='*60}\n", flush=True)

# ============================================================================
# Build scene
# ============================================================================
src_parts, src_center = read_source_wheel()
stage = omni.usd.get_context().get_stage()
build_physics_scene(stage)
build_step_env(stage)

robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names = build_robot(
    stage, src_parts, src_center)

print("Arm collision DISABLED — stability via chassis physics", flush=True)

# Initialize simulation
for _ in range(20):
    app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10):
    app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10):
    world.step(render=False)

if not art.is_physics_handle_valid():
    print("FATAL: Articulation physics handle invalid", flush=True)
    app.close(); sys.exit(1)

ndof = art.num_dof
dof_names = art.dof_names

# Map arm DOFs
arm_dof_indices = []
for ajn in arm_joint_names:
    for i, dn in enumerate(dof_names):
        if ajn in dn or dn == ajn:
            arm_dof_indices.append(i); break
if len(arm_dof_indices) != 4:
    arm_dof_indices = [i for i, dn in enumerate(dof_names) if "Arm" in dn]
print(f"Arm DOF indices: {arm_dof_indices}", flush=True)

# Lock wheels
non_arm = np.array([i for i in range(ndof) if i not in arm_dof_indices])
art.switch_control_mode("velocity", joint_indices=non_arm)
va_lock = np.zeros(ndof)
art.set_joint_velocity_targets(va_lock.reshape(1, -1))

# The omni wheel rollers allow the robot to slide off the step in Y.
# In deployment, the tether cable prevents this. We model this with
# physical guide walls at Y = ±guide_y that keep the wheels on the step.
# The walls are thin collision boxes straddling the step edge.
guide_y = 0.15  # slightly wider than wheel span (~0.09m from center)
wall_h = 0.3    # tall enough to block chassis
wall_t = 0.01   # thin

for side, ypos in [("inner", guide_y), ("outer", -guide_y)]:
    wp = UsdGeom.Cube.Define(stage, f"/World/Step/Guide_{side}")
    wp.GetSizeAttr().Set(1.0)
    wxf = UsdGeom.Xformable(wp.GetPrim()); wxf.ClearXformOpOrder()
    wxf.AddTranslateOp().Set(Gf.Vec3d(0, ypos, wall_h / 2.0 - 0.05))
    wxf.AddScaleOp().Set(Gf.Vec3d(4.0, wall_t, wall_h))
    UsdPhysics.CollisionAPI.Apply(wp.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(wp.GetPrim()).GetApproximationAttr().Set("boundingCube")
    PhysxSchema.PhysxCollisionAPI.Apply(wp.GetPrim())
    wp.GetPurposeAttr().Set(UsdGeom.Tokens.guide)  # invisible
print(f"Guide walls at Y=+/-{guide_y:.3f}m (models tether lateral constraint)", flush=True)

# Settle on step (3 seconds for proper settling)
print("Settling on step (3s)...", flush=True)
for _ in range(3 * PHYSICS_HZ):
    world.step(render=False)

init_jp = art.get_joint_positions().copy()

# ============================================================================
# Measure baseline chassis state
# ============================================================================
baseline_quat = get_chassis_quaternion(stage, chassis_path)
baseline_roll, baseline_pitch = get_chassis_orientation(stage, chassis_path)
baseline_wheel_z = get_wheel_hub_z(stage)

# Check where the chassis actually ended up
chassis_prim = stage.GetPrimAtPath(chassis_path)
chassis_mat = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
chassis_world_pos = chassis_mat.ExtractTranslation()
print(f"\nChassis world position: ({chassis_world_pos[0]:.4f}, {chassis_world_pos[1]:.4f}, {chassis_world_pos[2]:.4f})", flush=True)

# Also check individual wheel world XYZ
print("Wheel hub world positions:", flush=True)
for wname in WHEEL_ORDER:
    path = f"/World/Robot/Wheel_{wname}/Hub"
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = mat.ExtractTranslation()
        print(f"  {wname}: ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})", flush=True)

print(f"\nBaseline chassis state on step:", flush=True)
print(f"  Quaternion: [{baseline_quat[0]:.6f}, {baseline_quat[1]:.6f}, "
      f"{baseline_quat[2]:.6f}, {baseline_quat[3]:.6f}]", flush=True)
print(f"  Roll={baseline_roll:.2f} deg, Pitch={baseline_pitch:.2f} deg", flush=True)
print(f"  Wheel Z: FR={baseline_wheel_z['FR']:.4f} FL={baseline_wheel_z['FL']:.4f} "
      f"BL={baseline_wheel_z['BL']:.4f} BR={baseline_wheel_z['BR']:.4f}", flush=True)

# Compute actual tilt from wheel heights
z_right = (baseline_wheel_z["FR"] + baseline_wheel_z["BR"]) / 2.0
z_left = (baseline_wheel_z["FL"] + baseline_wheel_z["BL"]) / 2.0
wheel_y_span = abs(WHEEL_XY["FL"][1] - WHEEL_XY["FR"][1])
measured_tilt_rad = math.atan2(abs(z_right - z_left), wheel_y_span)
measured_tilt_deg = math.degrees(measured_tilt_rad)
print(f"\n  Z_right={z_right:.4f}m, Z_left={z_left:.4f}m", flush=True)
print(f"  Delta Z = {abs(z_right - z_left)*1000:.1f}mm over {wheel_y_span*1000:.1f}mm span", flush=True)
print(f"  MEASURED CHASSIS TILT = {measured_tilt_deg:.2f} degrees", flush=True)

# ============================================================================
# Measure-tilt mode: just report and exit
# ============================================================================
if args.measure_tilt:
    # Also measure tilt stability over 2 more seconds
    print("\nMonitoring tilt stability (2s)...", flush=True)
    quat_samples = []
    for i in range(2 * PHYSICS_HZ):
        world.step(render=False)
        if i % 72 == 0:  # every 0.2s
            q = get_chassis_quaternion(stage, chassis_path)
            angle = quaternion_angle_deg(baseline_quat, q)
            quat_samples.append(angle)
    print(f"  Quaternion drift over 2s: max={max(quat_samples):.3f} deg, "
          f"mean={sum(quat_samples)/len(quat_samples):.3f} deg", flush=True)

    # Save result
    tilt_result = {
        "config": config_name,
        "measured_tilt_deg": measured_tilt_deg,
        "measured_tilt_rad": measured_tilt_rad,
        "baseline_roll_deg": baseline_roll,
        "baseline_pitch_deg": baseline_pitch,
        "baseline_quaternion": baseline_quat.tolist(),
        "wheel_z": baseline_wheel_z,
        "z_right_avg": z_right,
        "z_left_avg": z_left,
        "quat_drift_max_deg": max(quat_samples),
        "quat_drift_mean_deg": sum(quat_samples) / len(quat_samples),
    }
    tilt_path = os.path.join(RESULTS_DIR, f"{config_name}_tilt.json")
    with open(tilt_path, "w") as f:
        json.dump(tilt_result, f, indent=2)
    print(f"\nTilt data saved to {tilt_path}", flush=True)
    app.close()
    sys.exit(0)

# ============================================================================
# Load poses from filtered CSV
# ============================================================================
if not os.path.exists(INPUT_CSV):
    print(f"FATAL: Input CSV not found: {INPUT_CSV}", flush=True)
    app.close(); sys.exit(1)

import arm_ik as _arm_ik_mod
_arm_params = _arm_ik_mod.get_arm_params(_arm_sel, loaded=args.armloaded)

print(f"\nLoading poses from {INPUT_CSV}...", flush=True)
all_rows = []
with open(INPUT_CSV, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        all_rows.append(row)
print(f"  Total rows: {len(all_rows)}", flush=True)

# Select geometry-valid poses (passes chassis/self/floor checks)
# Include both "valid" and "tipping" (tipping-only rejection = geometry is fine)
geom_valid_rows = []
for row in all_rows:
    status = row.get("filter_status", "")
    # Accept if status is "valid" or if the ONLY rejection is "tipping"
    if status == "valid":
        geom_valid_rows.append(row)
    elif "tipping" in status:
        # Check that no geometry filters are set
        if (row.get("filter_chassis") != "True" and
            row.get("filter_self_L4_L2") != "True" and
            row.get("filter_self_L4_J1") != "True" and
            row.get("filter_floor") != "True"):
            geom_valid_rows.append(row)
print(f"  Geometry-valid poses: {len(geom_valid_rows)}", flush=True)

# Compute corrected analytical tipping with measured tilt
print(f"  Computing tilted tipping margins (tilt={measured_tilt_deg:.2f} deg)...", flush=True)
margin_threshold_m = args.margin_mm / 1000.0

test_poses = []
for row in geom_valid_rows:
    j2_usd = float(row["j2_deg"])
    j3_usd = float(row["j3_deg"])
    j4_usd = float(row["j4_deg"])
    j2_fk, j3_fk, j4_fk = usd_to_fk_angles(j2_usd, j3_usd, j4_usd)

    tip = _arm_ik_mod.tipping_sweep_j1_tilted(
        math.radians(j2_fk), math.radians(j3_fk), math.radians(j4_fk),
        _arm_params, tilt_rad=measured_tilt_rad)

    if tip["worst_margin"] < margin_threshold_m:
        test_poses.append({
            "j2_deg": j2_usd, "j3_deg": j3_usd, "j4_deg": j4_usd,
            "analytical_margin_m": tip["worst_margin"],
            "worst_j1_deg": tip["worst_j1_deg"],
            "best_j1_deg": tip["best_j1_deg"],
            "n_stable": tip["n_stable"],
            "n_total": tip["n_total"],
        })

print(f"  Poses below {args.margin_mm:.0f}mm threshold: {len(test_poses)}", flush=True)

if args.test:
    # Take up to 20 evenly spaced poses sorted by margin
    test_poses.sort(key=lambda p: p["analytical_margin_m"])
    step = max(1, len(test_poses) // 20)
    test_poses = test_poses[::step][:20]
    print(f"  Test mode: selected {len(test_poses)} poses", flush=True)

if len(test_poses) == 0:
    print("\nNo poses to test! All margins above threshold.", flush=True)
    summary = {
        "config": config_name,
        "measured_tilt_deg": measured_tilt_deg,
        "margin_threshold_mm": args.margin_mm,
        "total_geom_valid": len(geom_valid_rows),
        "total_below_threshold": 0,
        "total_tested": 0,
        "stable": 0, "tipping": 0, "borderline": 0,
    }
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Summary saved to {json_path}", flush=True)
    app.close(); sys.exit(0)

# ============================================================================
# Teleport arm to launch (J2=90, straight up)
# ============================================================================
launch_jp = init_jp.flatten().copy()
launch_j2_deg = 90.0
launch_jp[arm_dof_indices[1]] = math.radians(launch_j2_deg)
art.set_joint_positions(launch_jp.reshape(1, -1))
art.set_joint_velocities(np.zeros_like(launch_jp).reshape(1, -1))
for i in range(4):
    jp_path = f"/World/Robot/{ARM_JOINT_NAMES[i]}"
    jprim = stage.GetPrimAtPath(jp_path)
    if jprim.IsValid():
        target_deg = [0.0, launch_j2_deg, 0.0, 0.0][i]
        UsdPhysics.DriveAPI(jprim, "angular").GetTargetPositionAttr().Set(float(target_deg))
art.set_joint_position_targets(launch_jp.reshape(1, -1))
art.set_joint_velocity_targets(va_lock.reshape(1, -1))

print("Launch: arm to J2=90 (straight up)", flush=True)
for _ in range(int(1.5 * PHYSICS_HZ)):
    world.step(render=False)

# Re-capture baseline after launch (arm up changes CG slightly)
baseline_quat = get_chassis_quaternion(stage, chassis_path)
baseline_wheel_z = get_wheel_hub_z(stage)
print("Baseline re-captured after launch", flush=True)

# ============================================================================
# Stability sweep loop
# ============================================================================
SETTLE_FRAMES = int(args.settle_s * PHYSICS_HZ)
HOLD_FRAMES = int(args.hold_s * PHYSICS_HZ)
SAMPLE_INTERVAL = 36  # sample every 0.1s

# Tipping thresholds
STABLE_QUAT_THRESH = 3.0    # degrees
TIPPING_QUAT_THRESH = 8.0   # degrees
STABLE_LIFT_THRESH = 3.0    # mm
TIPPING_LIFT_THRESH = 10.0  # mm

CSV_FIELDS = [
    "j2_deg", "j3_deg", "j4_deg", "j1_deg",
    "analytical_margin_m", "analytical_worst_j1_deg",
    "hold_quat_angle_deg", "hold_max_wheel_lift_mm",
    "settle_quat_angle_deg",
    "wheel_FR_lift_mm", "wheel_FL_lift_mm", "wheel_BL_lift_mm", "wheel_BR_lift_mm",
    "classification", "tipped_reset", "test_duration_s",
]

# Resume support
completed_keys = set()
if args.resume and os.path.exists(csv_path):
    with open(csv_path, "r") as f:
        for row in csv.DictReader(f):
            key = (float(row["j2_deg"]), float(row["j3_deg"]),
                   float(row["j4_deg"]), float(row["j1_deg"]))
            completed_keys.add(key)
    print(f"Resume: {len(completed_keys)} tests already done", flush=True)

csv_mode = "a" if (args.resume and len(completed_keys) > 0) else "w"
csv_file = open(csv_path, csv_mode, newline="", encoding="utf-8")
writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
if csv_mode == "w":
    writer.writeheader()
    csv_file.flush()

# For each pose, test 3 J1 angles: worst, worst+15, worst-15
n_total = 0
n_stable = 0
n_tipping = 0
n_borderline = 0
n_reset = 0
start_time = time.time()

total_tests = sum(3 for _ in test_poses)  # 3 J1 angles per pose
print(f"\nStarting stability sweep: {len(test_poses)} poses x 3 J1 angles = {total_tests} tests", flush=True)
print(f"Settle={args.settle_s:.1f}s, Hold={args.hold_s:.1f}s", flush=True)

for pi, pose in enumerate(test_poses):
    j2_deg = pose["j2_deg"]
    j3_deg = pose["j3_deg"]
    j4_deg = pose["j4_deg"]
    worst_j1 = pose["worst_j1_deg"]

    # Test 3 J1 angles: worst, worst-15, worst+15
    j1_angles = [worst_j1, (worst_j1 - 15) % 360, (worst_j1 + 15) % 360]

    for j1_deg in j1_angles:
        key = (j2_deg, j3_deg, j4_deg, j1_deg)
        if key in completed_keys:
            continue

        t0 = time.time()
        target_angles_deg = [j1_deg, j2_deg, j3_deg, j4_deg]

        # Build target joint position array
        target_jp = init_jp.flatten().copy()
        for i in range(4):
            target_jp[arm_dof_indices[i]] = math.radians(target_angles_deg[i])

        # Teleport
        art.set_joint_positions(target_jp.reshape(1, -1))
        art.set_joint_velocities(np.zeros_like(target_jp).reshape(1, -1))
        for i in range(4):
            jp_path = f"/World/Robot/{ARM_JOINT_NAMES[i]}"
            jprim = stage.GetPrimAtPath(jp_path)
            if jprim.IsValid():
                UsdPhysics.DriveAPI(jprim, "angular").GetTargetPositionAttr().Set(
                    float(target_angles_deg[i]))
        art.set_joint_position_targets(target_jp.reshape(1, -1))
        art.set_joint_velocity_targets(va_lock.reshape(1, -1))

        # Settle
        for _ in range(SETTLE_FRAMES):
            world.step(render=False)

        # NaN check
        raw_jp = art.get_joint_positions()
        if raw_jp is None or np.any(np.isnan(raw_jp)):
            # NaN — record and reset
            row = {f: "" for f in CSV_FIELDS}
            row["j2_deg"] = j2_deg; row["j3_deg"] = j3_deg; row["j4_deg"] = j4_deg
            row["j1_deg"] = j1_deg
            row["analytical_margin_m"] = f"{pose['analytical_margin_m']:.6f}"
            row["analytical_worst_j1_deg"] = worst_j1
            row["classification"] = "NaN"
            row["tipped_reset"] = "True"
            row["test_duration_s"] = f"{time.time()-t0:.2f}"
            writer.writerow(row); csv_file.flush()
            n_reset += 1
            # Reset
            art.set_joint_positions(launch_jp.reshape(1, -1))
            art.set_joint_velocities(np.zeros_like(launch_jp).reshape(1, -1))
            art.set_joint_position_targets(launch_jp.reshape(1, -1))
            for _ in range(2 * PHYSICS_HZ):
                world.step(render=False)
            baseline_quat = get_chassis_quaternion(stage, chassis_path)
            baseline_wheel_z = get_wheel_hub_z(stage)
            continue

        # Check orientation after settle
        settle_quat = get_chassis_quaternion(stage, chassis_path)
        settle_angle = quaternion_angle_deg(baseline_quat, settle_quat)

        # Hold and detect tipping
        max_quat_angle = settle_angle
        max_wheel_lift = 0.0
        wheel_lifts = {wn: 0.0 for wn in WHEEL_ORDER}

        for frame in range(HOLD_FRAMES):
            world.step(render=False)
            if frame % SAMPLE_INTERVAL == 0:
                q = get_chassis_quaternion(stage, chassis_path)
                angle = quaternion_angle_deg(baseline_quat, q)
                if angle > max_quat_angle:
                    max_quat_angle = angle

                wz = get_wheel_hub_z(stage)
                for wn in WHEEL_ORDER:
                    lift = (wz[wn] - baseline_wheel_z[wn]) * 1000.0  # mm, positive = up
                    if lift > wheel_lifts[wn]:
                        wheel_lifts[wn] = lift
                    if lift > max_wheel_lift:
                        max_wheel_lift = lift

        # Classify
        if max_quat_angle > TIPPING_QUAT_THRESH or max_wheel_lift > TIPPING_LIFT_THRESH:
            classification = "TIPPING"
            n_tipping += 1
        elif max_quat_angle < STABLE_QUAT_THRESH and max_wheel_lift < STABLE_LIFT_THRESH:
            classification = "STABLE"
            n_stable += 1
        else:
            classification = "BORDERLINE"
            n_borderline += 1

        tipped_reset = classification == "TIPPING"
        n_total += 1

        row = {
            "j2_deg": j2_deg, "j3_deg": j3_deg, "j4_deg": j4_deg,
            "j1_deg": j1_deg,
            "analytical_margin_m": f"{pose['analytical_margin_m']:.6f}",
            "analytical_worst_j1_deg": worst_j1,
            "hold_quat_angle_deg": f"{max_quat_angle:.3f}",
            "hold_max_wheel_lift_mm": f"{max_wheel_lift:.2f}",
            "settle_quat_angle_deg": f"{settle_angle:.3f}",
            "wheel_FR_lift_mm": f"{wheel_lifts['FR']:.2f}",
            "wheel_FL_lift_mm": f"{wheel_lifts['FL']:.2f}",
            "wheel_BL_lift_mm": f"{wheel_lifts['BL']:.2f}",
            "wheel_BR_lift_mm": f"{wheel_lifts['BR']:.2f}",
            "classification": classification,
            "tipped_reset": str(tipped_reset),
            "test_duration_s": f"{time.time()-t0:.2f}",
        }
        writer.writerow(row); csv_file.flush()

        # Reset if tipped
        if tipped_reset:
            n_reset += 1
            art.set_joint_positions(launch_jp.reshape(1, -1))
            art.set_joint_velocities(np.zeros_like(launch_jp).reshape(1, -1))
            art.set_joint_position_targets(launch_jp.reshape(1, -1))
            for _ in range(2 * PHYSICS_HZ):
                world.step(render=False)
            baseline_quat = get_chassis_quaternion(stage, chassis_path)
            baseline_wheel_z = get_wheel_hub_z(stage)

    # Progress
    if (pi + 1) % 10 == 0 or pi == 0:
        elapsed = time.time() - start_time
        rate = n_total / elapsed if elapsed > 0 else 0
        print(f"  [{pi+1}/{len(test_poses)}] tests={n_total} "
              f"stable={n_stable} tipping={n_tipping} borderline={n_borderline} "
              f"resets={n_reset} ({rate:.1f} tests/s)", flush=True)

csv_file.close()
elapsed = time.time() - start_time

print(f"\n{'='*60}", flush=True)
print(f"Stability sweep complete: {config_name}", flush=True)
print(f"  Total tests: {n_total}", flush=True)
print(f"  STABLE: {n_stable} ({100*n_stable/max(1,n_total):.1f}%)", flush=True)
print(f"  TIPPING: {n_tipping} ({100*n_tipping/max(1,n_total):.1f}%)", flush=True)
print(f"  BORDERLINE: {n_borderline} ({100*n_borderline/max(1,n_total):.1f}%)", flush=True)
print(f"  Resets: {n_reset}", flush=True)
print(f"  Time: {elapsed:.0f}s ({elapsed/60:.1f}min)", flush=True)
print(f"  Measured tilt: {measured_tilt_deg:.2f} deg", flush=True)
print(f"{'='*60}", flush=True)

# Save summary
summary = {
    "config": config_name,
    "measured_tilt_deg": measured_tilt_deg,
    "measured_tilt_rad": measured_tilt_rad,
    "margin_threshold_mm": args.margin_mm,
    "settle_s": args.settle_s,
    "hold_s": args.hold_s,
    "total_geom_valid": len(geom_valid_rows),
    "total_below_threshold": len(test_poses),
    "total_tested": n_total,
    "stable": n_stable,
    "tipping": n_tipping,
    "borderline": n_borderline,
    "resets": n_reset,
    "elapsed_s": elapsed,
    "thresholds": {
        "stable_quat_deg": STABLE_QUAT_THRESH,
        "tipping_quat_deg": TIPPING_QUAT_THRESH,
        "stable_lift_mm": STABLE_LIFT_THRESH,
        "tipping_lift_mm": TIPPING_LIFT_THRESH,
    },
}
with open(json_path, "w") as f:
    json.dump(summary, f, indent=2)
print(f"Summary: {json_path}", flush=True)
print(f"CSV: {csv_path}", flush=True)

app.close()
