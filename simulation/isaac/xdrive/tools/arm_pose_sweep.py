"""
FORTIS arm pose sweep — headless Isaac Sim script.

Systematically sweeps through a grid of joint angles for the FORTIS robot arm,
measuring torques, tipping stability, and end-effector contact at each pose.
Runs headless, outputs CSV + JSON.

Built on canonical/xdrive_reactor_arm.py (4-DOF parallel-link arm on arched
chassis with real Kaya omni-wheel meshes).

Usage:
  IsaacSim\\python.bat tools/arm_pose_sweep.py --36arm
  IsaacSim\\python.bat tools/arm_pose_sweep.py --30arm --armloaded --step
  IsaacSim\\python.bat tools/arm_pose_sweep.py --24arm --reactor --resume
"""
import os, sys, math, argparse, time, csv, json, datetime
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser(description="FORTIS arm pose sweep — headless")
parser.add_argument("--36arm", action="store_true", dest="arm36", help="36\" arm (default)")
parser.add_argument("--30arm", action="store_true", dest="arm30", help="30\" arm")
parser.add_argument("--24arm", action="store_true", dest="arm24", help="24\" arm")
parser.add_argument("--armloaded", action="store_true")
parser.add_argument("--step", action="store_true")
parser.add_argument("--reactor", action="store_true")
parser.add_argument("--hz", type=int, default=360)
parser.add_argument("--belly", type=float, default=2.5)
parser.add_argument("--j2-range", default="-30,150,10", help="start,stop,step")
parser.add_argument("--j3-range", default="-150,150,10")
parser.add_argument("--j4-range", default="-90,90,15")
parser.add_argument("--j1-tipping", default="0,360,15", help="J1 yaw angles for analytical tipping")
parser.add_argument("--resume", action="store_true", help="Resume from existing CSV")
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
# Constants — chassis & wheels (identical to canonical/xdrive_reactor_arm.py)
# ============================================================================
IN = 0.0254
MM = 0.001
# --- xdrive path bootstrap (script lives in tools/) ---
XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ASSETS_DIR  = os.path.join(XDRIVE_ROOT, "assets")
RESULTS_ROOT = os.path.join(XDRIVE_ROOT, "results")
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
# --- end bootstrap ---
OMNIWHEEL_USD = os.path.join(ASSETS_DIR, "omniwheels.usd")

# Chassis
CHASSIS_L = 15.354 * IN
CHASSIS_W = 9.353 * IN
CHASSIS_H = 7.1 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
MOTOR_MOUNT_LEN = 2.5 * IN
BELLY_HEIGHT = args.belly * IN
ARCH_FLAT_WIDTH = 3.0 * IN
REACTOR_USD = os.path.join(ASSETS_DIR, "diiid_reactor.usd")

# Wheels (AndyMark 8" Dualie)
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

# Mass
CHASSIS_MASS = 20.4
WHEEL_MASS = 1.0
ROLLER_MASS_FRAC = 0.3
NUM_ROLLERS = 10
HUB_MASS = WHEEL_MASS * (1.0 - ROLLER_MASS_FRAC)
ROLLER_MASS = WHEEL_MASS * ROLLER_MASS_FRAC / NUM_ROLLERS

# Physics
PHYSICS_HZ = max(360, min(480, args.hz))
FRICTION_MU = 0.5
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
# Constants — 4-DOF arm (configurable via --36arm / --30arm / --24arm)
# ============================================================================
J1_STACK_H = 1.5 * IN  # 38.1mm (motor+gearbox recessed into chassis)

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

ARM_STIFFNESS = 1000.0
ARM_DAMPING   = 100.0
ARM_MAX_FORCE = 100.0

ARM_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),
    "ArmJ2": (-180.0, 180.0),
    "ArmJ3": (-180.0, 180.0),
    "ArmJ4": (-180.0, 180.0),
}

ARM_JOINT_NAMES = ["ArmJ1", "ArmJ2", "ArmJ3", "ArmJ4"]


# ============================================================================
# Source wheel data reader (from canonical/xdrive_reactor_arm.py)
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
# Scene builders (from canonical/xdrive_reactor_arm.py)
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
    print(f"Physics: {PHYSICS_HZ}Hz, CPU dynamics, TGS solver", flush=True)


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


def build_step_env(stage):
    """Two flat planes with a 4.5" step between them."""
    step_h = 4.5 * IN
    half = 2.0

    outer = UsdGeom.Mesh.Define(stage, "/World/Step/Outer")
    outer.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, -half * 2, 0),  Gf.Vec3f(half, -half * 2, 0),
        Gf.Vec3f(half, 0, 0),           Gf.Vec3f(-half, 0, 0)]))
    outer.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    outer.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    outer.GetSubdivisionSchemeAttr().Set("none")
    outer.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.6, 0.6, 0.65)]))
    UsdPhysics.CollisionAPI.Apply(outer.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(outer.GetPrim()).GetApproximationAttr().Set("none")

    inner = UsdGeom.Mesh.Define(stage, "/World/Step/Inner")
    inner.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, 0, -step_h),    Gf.Vec3f(half, 0, -step_h),
        Gf.Vec3f(half, half * 2, -step_h), Gf.Vec3f(-half, half * 2, -step_h)]))
    inner.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    inner.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    inner.GetSubdivisionSchemeAttr().Set("none")
    inner.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.5, 0.5, 0.55)]))
    UsdPhysics.CollisionAPI.Apply(inner.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(inner.GetPrim()).GetApproximationAttr().Set("none")

    face = UsdGeom.Mesh.Define(stage, "/World/Step/Face")
    face.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, 0, -step_h),  Gf.Vec3f(half, 0, -step_h),
        Gf.Vec3f(half, 0, 0),         Gf.Vec3f(-half, 0, 0)]))
    face.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    face.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    face.GetSubdivisionSchemeAttr().Set("none")
    face.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.4, 0.4, 0.45)]))
    UsdPhysics.CollisionAPI.Apply(face.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(face.GetPrim()).GetApproximationAttr().Set("none")

    smat = UsdShade.Material.Define(stage, "/World/StepMat")
    spm = UsdPhysics.MaterialAPI.Apply(smat.GetPrim())
    spm.CreateStaticFrictionAttr(FRICTION_MU)
    spm.CreateDynamicFrictionAttr(FRICTION_MU)
    spm.CreateRestitutionAttr(0.05)
    for p in [outer, inner, face]:
        UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(
            smat, UsdShade.Tokens.weakerThanDescendants, "physics")

    edge = UsdGeom.Cube.Define(stage, "/World/Step/EdgeLine")
    edge.GetSizeAttr().Set(1.0)
    exf = UsdGeom.Xformable(edge.GetPrim()); exf.ClearXformOpOrder()
    exf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -step_h / 2.0))
    exf.AddScaleOp().Set(Gf.Vec3d(half * 2, 0.005, step_h))
    edge.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.3, 0.3)]))

    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

    print(f"Step env: outer floor Z=0, inner floor Z={-step_h/IN:.1f}\", "
          f"step height={step_h/IN:.1f}\", edge at Y=0", flush=True)
    return step_h


# ============================================================================
# Chassis builder (from canonical/xdrive_reactor_arm.py)
# ============================================================================

def build_arched_chassis(stage, path, half_h, color):
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half
    ramp_end = SL - MOTOR_MOUNT_LEN

    z_low = -half_h
    z_high = -half_h + BELLY_HEIGHT

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
    half_h = CHASSIS_H / 2.0

    chassis_mesh = build_arched_chassis(stage, chassis_path + "/body",
                                        half_h, (0.25, 0.25, 0.35))
    UsdPhysics.CollisionAPI.Apply(chassis_mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(chassis_mesh.GetPrim()).CreateApproximationAttr("convexDecomposition")

    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim())
    axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))


# ============================================================================
# Omni wheel builder (from canonical/xdrive_reactor_arm.py)
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

    min_pair_dist = float('inf')
    for i in range(len(all_scaled_centers)):
        for j in range(i + 1, len(all_scaled_centers)):
            d = float(np.linalg.norm(all_scaled_centers[i] - all_scaled_centers[j]))
            if d < min_pair_dist:
                min_pair_dist = d
    safe_roller_r = min_pair_dist / 2.0 - 0.003
    max_surface_r = float(WHEEL_RADIUS - np.sqrt(
        all_scaled_centers[0][0]**2 + all_scaled_centers[0][2]**2))
    roller_coll_r_global = max(min(safe_roller_r, max_surface_r), 0.005)

    for ri, barrel in enumerate(src_parts["roller_barrel"]):
        bc = barrel["points"].mean(axis=0)
        scaled_center = all_scaled_centers[ri]

        rp = wheel_path + f"/Roller_{ri}"
        rxf = UsdGeom.Xform.Define(stage, rp)
        rprim = rxf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(rprim)
        UsdPhysics.MassAPI.Apply(rprim).CreateMassAttr(ROLLER_MASS)
        prb = PhysxSchema.PhysxRigidBodyAPI.Apply(rprim)
        prb.CreateSolverPositionIterationCountAttr(16)
        prb.CreateMaxDepenetrationVelocityAttr(5.0)

        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))

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

        bvp = rp + "/barrel_vis"
        create_mesh_prim(stage, bvp, barrel, src_center,
                         SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                         body_offset=scaled_center, color=roller_color)

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
# Arm build helpers (from canonical/xdrive_reactor_arm.py)
# ============================================================================

def _apply_arm_body_mass_x(prim, joint_mass, link_mass, link_length, link_dir,
                           extra_mass=0.0, extra_x=0.0):
    """Set mass, COM, and inertia for an arm body with link along X."""
    total = joint_mass + link_mass + extra_mass
    link_cx = link_dir * link_length / 2.0
    com_x = (joint_mass * 0.0 + link_mass * link_cx + extra_mass * extra_x) / total

    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0.0, 0.0))

    Lx = max(float(link_length), 0.01)
    Ly = CF_TUBE_SIZE
    Lz = CF_TUBE_SIZE
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
    """Mass for J1 base: motor stack along +Z."""
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, float(stack_h / 2.0)))
    Lx = 0.057
    Ly = 0.057
    Lz = float(stack_h)
    Ixx = mass / 12.0 * (Ly**2 + Lz**2)
    Iyy = mass / 12.0 * (Lx**2 + Lz**2)
    Izz = mass / 12.0 * (Lx**2 + Ly**2)
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixx, Iyy, Izz))
    mapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _add_arm_link_visual(stage, path, center, size, color):
    """Cosmetic box for an arm link (no collision)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def _add_arm_collision_box(stage, path, center, size):
    """Invisible collision box for an arm link body."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.GetPurposeAttr().Set(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim()).GetApproximationAttr().Set("boundingCube")


def _add_arm_joint(stage, joint_path, body0, body1, localPos0, axis, limit_deg):
    """Revolute joint with position-mode DriveAPI."""
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


# ============================================================================
# build_arm — 4-DOF parallel-link arm, flat stowed zigzag
# ============================================================================

def build_arm(stage, robot_path, chassis_path):
    """Build the 4-DOF arm as rigid bodies in the chassis articulation."""
    j1_path = robot_path + "/ArmJ1base"
    j2_path = robot_path + "/ArmJ2shoulder"
    j3_path = robot_path + "/ArmJ3elbow"
    j4_path = robot_path + "/ArmJ4wrist"

    z_arm = ARM_MOUNT_Z + J1_STACK_H
    y_l2, y_l3, y_l4 = LINK_Y

    C_J1 = (0.55, 0.55, 0.60)
    C_L2 = (0.85, 0.30, 0.25)
    C_L3 = (0.25, 0.75, 0.30)
    C_L4 = (0.25, 0.30, 0.85)

    # ---- Body 1: J1 base (vertical motor stack, +Z) ----
    UsdGeom.Xform.Define(stage, j1_path)
    p1 = stage.GetPrimAtPath(j1_path)
    xf1 = UsdGeom.Xformable(p1); xf1.ClearXformOpOrder()
    xf1.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z))
    UsdPhysics.RigidBodyAPI.Apply(p1)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p1).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_j1_mass(p1, mass=M_JOINT, stack_h=J1_STACK_H)
    _add_arm_link_visual(stage, j1_path + "/vis",
                         center=(0.0, 0.0, J1_STACK_H / 2.0),
                         size=(0.057, 0.057, J1_STACK_H),
                         color=C_J1)
    _add_arm_collision_box(stage, j1_path + "/col",
                           center=(0.0, 0.0, J1_STACK_H / 2.0),
                           size=(0.057, 0.057, J1_STACK_H))

    # ---- Body 2: J2 shoulder (L2, +X) ----
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
                         center=(+L_L2 / 2.0, 0.0, 0.0),
                         size=(L_L2, CF_TUBE_SIZE, CF_TUBE_SIZE),
                         color=C_L2)
    _add_arm_collision_box(stage, j2_path + "/col",
                           center=(+L_L2 / 2.0, 0.0, 0.0),
                           size=(L_L2, CF_TUBE_SIZE, CF_TUBE_SIZE))
    cam = UsdGeom.Cube.Define(stage, j2_path + "/camera")
    cam.GetSizeAttr().Set(1.0)
    cxf = UsdGeom.Xformable(cam.GetPrim()); cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(CAM_X_ON_L2, 0.0, CF_TUBE_SIZE / 2.0 + 0.015))
    cxf.AddScaleOp().Set(Gf.Vec3d(0.060, 0.025, 0.025))
    cam.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.05, 0.05, 0.05)]))
    jsph = UsdGeom.Sphere.Define(stage, j2_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L2)]))

    # ---- Body 3: J3 elbow (L3, -X) ----
    UsdGeom.Xform.Define(stage, j3_path)
    p3 = stage.GetPrimAtPath(j3_path)
    xf3 = UsdGeom.Xformable(p3); xf3.ClearXformOpOrder()
    xf3.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2, y_l3, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p3)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p3).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p3, joint_mass=M_JOINT, link_mass=M_L3,
                           link_length=L_L3, link_dir=-1)
    _add_arm_link_visual(stage, j3_path + "/vis",
                         center=(-L_L3 / 2.0, 0.0, 0.0),
                         size=(L_L3, CF_TUBE_SIZE, CF_TUBE_SIZE),
                         color=C_L3)
    _add_arm_collision_box(stage, j3_path + "/col",
                           center=(-L_L3 / 2.0, 0.0, 0.0),
                           size=(L_L3, CF_TUBE_SIZE, CF_TUBE_SIZE))
    jsph = UsdGeom.Sphere.Define(stage, j3_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L3)]))

    # ---- Body 4: J4 wrist (L4, +X, gripper at tip) ----
    UsdGeom.Xform.Define(stage, j4_path)
    p4 = stage.GetPrimAtPath(j4_path)
    xf4 = UsdGeom.Xformable(p4); xf4.ClearXformOpOrder()
    xf4.AddTranslateOp().Set(Gf.Vec3d(
        ARM_MOUNT_X + L_L2 - L_L3, y_l4, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p4)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p4).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p4, joint_mass=M_JOINT, link_mass=M_L4,
                           link_length=L_L4, link_dir=+1,
                           extra_mass=M_GRIPPER, extra_x=L_L4)
    _add_arm_link_visual(stage, j4_path + "/vis",
                         center=(+L_L4 / 2.0, 0.0, 0.0),
                         size=(L_L4, CF_TUBE_SIZE, CF_TUBE_SIZE),
                         color=C_L4)
    grip_col_len = L_L4 + 0.030
    _add_arm_collision_box(stage, j4_path + "/col",
                           center=(grip_col_len / 2.0, 0.0, 0.0),
                           size=(grip_col_len, 0.040, CF_TUBE_SIZE))
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
    jsph = UsdGeom.Sphere.Define(stage, j4_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L4)]))

    # ---- Joints ----
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
    print(f"\n{'='*50}", flush=True)
    print(f"ARM BUILD SUMMARY ({ARM_REACH_IN:.0f}\" reach, {load_label})", flush=True)
    print(f"{'='*50}", flush=True)
    print(f"  J1 stack height: {J1_STACK_H*1000:.0f}mm ({J1_STACK_H/IN:.2f}\")", flush=True)
    print(f"  L2={L_L2/IN:.0f}\"  L3={L_L3/IN:.0f}\"  L4={L_L4/IN:.0f}\"", flush=True)
    print(f"  Total reach: {(L_L2+L_L3+L_L4)/IN:.1f}\"", flush=True)
    print(f"  Joint mass: {M_JOINT:.3f} kg x 4 = {4*M_JOINT:.3f} kg", flush=True)
    print(f"  Link masses: L2={M_L2*1000:.1f}g  L3={M_L3*1000:.1f}g  "
          f"L4={M_L4*1000:.1f}g", flush=True)
    if ARM_LOADED:
        print(f"  EE tip: {M_GRIPPER*1000:.0f}g at L4 tip "
              f"(gripper {M_GRIPPER_BARE*1000:.0f}g + payload {M_PAYLOAD*1000:.0f}g)",
              flush=True)
    else:
        print(f"  EE tip: {M_GRIPPER*1000:.0f}g at L4 tip (gripper only)", flush=True)
    print(f"  Camera: {M_CAMERA*1000:.0f}g on J2 shoulder ({CAM_X_ON_L2/IN:.1f}\" from J2)", flush=True)
    m_j1 = M_JOINT
    m_j2 = M_JOINT + M_L2 + M_CAMERA
    m_j3 = M_JOINT + M_L3
    m_j4 = M_JOINT + M_L4 + M_GRIPPER
    print(f"\n  Per-body masses (joint + link + extras):", flush=True)
    print(f"    J1_base:     {m_j1:.3f} kg ({m_j1*2.205:.2f} lb)  [motor+gearbox]", flush=True)
    print(f"    J2_shoulder: {m_j2:.3f} kg ({m_j2*2.205:.2f} lb)  [joint + L2 {M_L2*1000:.1f}g + camera {M_CAMERA*1000:.0f}g]", flush=True)
    print(f"    J3_elbow:    {m_j3:.3f} kg ({m_j3*2.205:.2f} lb)  [joint + L3 {M_L3*1000:.1f}g]", flush=True)
    print(f"    J4_wrist:    {m_j4:.3f} kg ({m_j4*2.205:.2f} lb)  [joint + L4 {M_L4*1000:.1f}g + EE {M_GRIPPER*1000:.0f}g]", flush=True)
    print(f"\n  Total arm mass: {total_arm_mass:.3f} kg ({total_arm_mass*2.205:.2f} lb)", flush=True)
    print(f"  Mount: back edge x={ARM_MOUNT_X/IN:+.2f}\" z={ARM_MOUNT_Z/IN:+.2f}\"", flush=True)
    y_l2_, y_l3_, y_l4_ = LINK_Y
    print(f"  Link Y lanes (side-by-side): "
          f"L2={y_l2_/IN:+.2f}\"  L3={y_l3_/IN:+.2f}\"  "
          f"L4={y_l4_/IN:+.2f}\"", flush=True)
    print(f"  Stowed positions (chassis frame X):", flush=True)
    j3x = ARM_MOUNT_X + L_L2
    j4x = ARM_MOUNT_X + L_L2 - L_L3
    tipx = j4x + L_L4
    print(f"    J1={ARM_MOUNT_X/IN:+.2f}\"  J2={ARM_MOUNT_X/IN:+.2f}\"  "
          f"J3={j3x/IN:+.2f}\"  J4={j4x/IN:+.2f}\"  "
          f"tip={tipx/IN:+.2f}\"", flush=True)
    print(f"{'='*50}\n", flush=True)

    return ARM_JOINT_NAMES


# ============================================================================
# Arm world position helpers
# ============================================================================

def arm_tip_world(stage):
    """Arm tip world position (gripper at L4 tip)."""
    prim = stage.GetPrimAtPath("/World/Robot/ArmJ4wrist")
    if not prim.IsValid():
        return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return mat.Transform(Gf.Vec3d(L_L4, 0.0, 0.0))


def arm_body_world_pos(stage, body_path):
    """World position of an arm body origin."""
    prim = stage.GetPrimAtPath(body_path)
    if not prim.IsValid():
        return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


# ============================================================================
# Contact & tipping (from canonical/xdrive_reactor_arm.py)
# ============================================================================

def read_ee_contacts(cs, ee_path):
    """Read raw contact data for end-effector touching anything."""
    raw = cs.get_rigid_body_raw_data(ee_path)
    contacts = []
    for c in raw:
        b0 = cs.decode_body_name(int(c["body0"]))
        b1 = cs.decode_body_name(int(c["body1"]))
        other = b1 if ee_path in b0 else b0
        imp = np.array([float(c["impulse"]["x"]),
                        float(c["impulse"]["y"]),
                        float(c["impulse"]["z"])])
        pos = np.array([float(c["position"]["x"]),
                        float(c["position"]["y"]),
                        float(c["position"]["z"])])
        contacts.append({"other": other, "position": pos,
                         "impulse": imp, "force_mag": np.linalg.norm(imp)})
    return contacts


def read_chassis_contacts(cs, chassis_path_local):
    """Read contacts on chassis body, return list of arm-body contacts."""
    raw = cs.get_rigid_body_raw_data(chassis_path_local)
    arm_contacts = []
    for c in raw:
        b0 = cs.decode_body_name(int(c["body0"]))
        b1 = cs.decode_body_name(int(c["body1"]))
        other = b1 if chassis_path_local in b0 else b0
        # Only care about arm bodies hitting chassis
        if "Arm" in other:
            imp = np.array([float(c["impulse"]["x"]),
                            float(c["impulse"]["y"]),
                            float(c["impulse"]["z"])])
            arm_contacts.append({"body": other, "force_mag": np.linalg.norm(imp)})
    return arm_contacts


def compute_tipping_margins(stage, chassis_path, arm_joint_names_local):
    """Compute tipping stability: arm CG vs wheel support polygon."""
    GRAV = 9.81
    arm_bodies = [
        ("/World/Robot/ArmJ1base",     M_JOINT),
        ("/World/Robot/ArmJ2shoulder", M_JOINT + M_L2 + M_CAMERA),
        ("/World/Robot/ArmJ3elbow",    M_JOINT + M_L3),
        ("/World/Robot/ArmJ4wrist",    M_JOINT + M_L4 + M_GRIPPER),
    ]
    total_m = 0.0
    cg = np.zeros(3)
    for path, mass in arm_bodies:
        p = arm_body_world_pos(stage, path)
        if p is not None:
            cg += mass * p
            total_m += mass
    if total_m > 0:
        cg /= total_m

    cprim = stage.GetPrimAtPath(chassis_path)
    cmat = UsdGeom.Xformable(cprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    wheel_world = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        wz = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
        wp = cmat.Transform(Gf.Vec3d(wx, wy, wz))
        wheel_world.append(np.array([wp[0], wp[1]]))

    wxs = [w[0] for w in wheel_world]
    wys = [w[1] for w in wheel_world]
    x_min, x_max = min(wxs), max(wxs)
    y_min, y_max = min(wys), max(wys)

    margin_front = x_max - cg[0]
    margin_back  = cg[0] - x_min
    margin_left  = y_max - cg[1]
    margin_right = cg[1] - y_min

    worst_overhang = -min(margin_front, margin_back, margin_left, margin_right)
    if worst_overhang < 0:
        worst_overhang = 0.0
    tipping_moment = total_m * GRAV * worst_overhang

    return {
        "arm_cg": cg,
        "arm_mass": total_m,
        "wheel_xy": wheel_world,
        "margin_front": margin_front,
        "margin_back": margin_back,
        "margin_left": margin_left,
        "margin_right": margin_right,
        "tipping_moment_Nm": tipping_moment,
        "stable": min(margin_front, margin_back, margin_left, margin_right) > 0,
    }


# ============================================================================
# Analytical J1 tipping
# ============================================================================

def analytical_j1_tipping(stage, chassis_path, j1_angles_deg):
    """Compute tipping at multiple J1 yaw angles from current arm positions.

    Reads arm body world positions at current J1=0 pose, then analytically
    rotates them around the J1 axis (Z at arm mount point) to compute
    tipping margins at each J1 angle. Returns (worst_j1_deg, worst_margin_m).
    """
    mount_world = arm_body_world_pos(stage, "/World/Robot/ArmJ1base")
    if mount_world is None:
        return 0.0, 0.0

    bodies = [
        ("/World/Robot/ArmJ1base",     M_JOINT),
        ("/World/Robot/ArmJ2shoulder", M_JOINT + M_L2 + M_CAMERA),
        ("/World/Robot/ArmJ3elbow",    M_JOINT + M_L3),
        ("/World/Robot/ArmJ4wrist",    M_JOINT + M_L4 + M_GRIPPER),
    ]

    rel_positions = []
    masses = []
    for path, mass in bodies:
        p = arm_body_world_pos(stage, path)
        if p is None:
            return 0.0, 0.0
        rel_positions.append(p - mount_world)
        masses.append(mass)

    cprim = stage.GetPrimAtPath(chassis_path)
    cmat = UsdGeom.Xformable(cprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    wheel_world = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        wz = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
        wp = cmat.Transform(Gf.Vec3d(wx, wy, wz))
        wheel_world.append(np.array([wp[0], wp[1]]))
    wxs = [w[0] for w in wheel_world]
    wys = [w[1] for w in wheel_world]
    x_min, x_max = min(wxs), max(wxs)
    y_min, y_max = min(wys), max(wys)

    worst_margin = float('inf')
    worst_j1 = 0.0

    for j1_deg in j1_angles_deg:
        j1_rad = math.radians(j1_deg)
        cos_j1 = math.cos(j1_rad)
        sin_j1 = math.sin(j1_rad)

        cg = np.zeros(3)
        total_m = 0.0
        for rel_p, mass in zip(rel_positions, masses):
            rx = rel_p[0] * cos_j1 - rel_p[1] * sin_j1
            ry = rel_p[0] * sin_j1 + rel_p[1] * cos_j1
            world_p = mount_world + np.array([rx, ry, rel_p[2]])
            cg += mass * world_p
            total_m += mass
        cg /= total_m

        mf = x_max - cg[0]
        mb = cg[0] - x_min
        ml = y_max - cg[1]
        mr = cg[1] - y_min
        min_margin = min(mf, mb, ml, mr)

        if min_margin < worst_margin:
            worst_margin = min_margin
            worst_j1 = j1_deg

    return worst_j1, worst_margin


# ============================================================================
# build_robot (chassis + wheels + arm)
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
    # Self-collisions ON: arm collides with chassis, PhysX auto-filters
    # parent-child (joint-connected) pairs so adjacent links don't explode.
    physx_art.CreateEnabledSelfCollisionsAttr(True)

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

        norm_len = math.sqrt(cx * cx + cy * cy)
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

        jprim = stage.GetPrimAtPath(djp)
        jprim.CreateAttribute("isaacmecanumwheel:radius",
                              Sdf.ValueTypeNames.Float).Set(float(WHEEL_RADIUS))
        jprim.CreateAttribute("isaacmecanumwheel:angle",
                              Sdf.ValueTypeNames.Float).Set(90.0)

        drive_joint_paths.append(djp)

    arm_joint_names = build_arm(stage, robot_path, chassis_path)

    # Spawn position
    if args.reactor:
        import sim_config as cfg
        SPAWN_X = 0.0
        SPAWN_Y = -1.75
        SPAWN_FLOOR_Z = cfg.Z_OUTER_IN * IN
        spawn_z = SPAWN_FLOOR_Z + WHEEL_RADIUS + CHASSIS_H / 2.0 + 0.10
        spawn_yaw = math.degrees(math.atan2(0 - SPAWN_Y, 0 - SPAWN_X))
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, spawn_z))
        rxf.AddRotateZOp().Set(spawn_yaw)
        print(f"Reactor spawn: ({SPAWN_X:.4f}, {SPAWN_Y:.4f}, {spawn_z:.4f})m "
              f"yaw={spawn_yaw:.1f}deg", flush=True)
    elif args.step:
        spawn_z = WHEEL_RADIUS + CHASSIS_H / 2.0 + 0.02
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, spawn_z))
        rxf.AddRotateZOp().Set(90.0)
        print(f"Step spawn: center on step edge, yaw=90deg (front faces lower floor), "
              f"Z={spawn_z:.4f}m", flush=True)
    else:
        spawn_z = BELLY_HEIGHT + CHASSIS_H / 2.0 + 0.01
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    print(f"Chassis: {CHASSIS_L/IN:.1f}x{CHASSIS_W/IN:.1f}x{CHASSIS_H/IN:.1f}\" "
          f"{CHASSIS_MASS}kg", flush=True)
    print(f"Belly height: {BELLY_HEIGHT/IN:.2f}\"", flush=True)
    print(f"Wheel: {TARGET_DIA_MM:.0f}mm dia, {NUM_ROLLERS} real rollers/wheel", flush=True)
    n_bodies = 1 + 4 + 4*NUM_ROLLERS + 4
    n_joints = 4 + 4*NUM_ROLLERS + 4
    print(f"Total bodies: {n_bodies}  Total joints: {n_joints}", flush=True)

    return robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names


# ============================================================================
# State reporting
# ============================================================================

def get_state(art, stage, chassis_path):
    prim = stage.GetPrimAtPath(chassis_path)
    if not prim.IsValid():
        return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    jv, je, jp = None, None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    try: jp = art.get_joint_positions()
    except: pass
    return {"pos": np.array([t[0], t[1], t[2]]), "jv": jv, "je": je, "jp": jp}


# ============================================================================
# Grid parsing helpers
# ============================================================================

def parse_range(s):
    """Parse 'start,stop,step' string into np.arange (inclusive of stop)."""
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 3:
        raise ValueError(f"Expected 'start,stop,step', got '{s}'")
    start, stop, step = parts
    return np.arange(start, stop + step * 0.5, step)


# ============================================================================
# MAIN
# ============================================================================
print("=" * 60, flush=True)
_load_str = " LOADED (gripper+payload)" if ARM_LOADED else ""
print(f"FORTIS Arm Pose Sweep ({ARM_REACH_IN:.0f}\" reach){_load_str} — HEADLESS",
      flush=True)
print("=" * 60, flush=True)

# Parse grid ranges
j2_grid = parse_range(args.j2_range)
j3_grid = parse_range(args.j3_range)
j4_grid = parse_range(args.j4_range)
j1_tipping_angles = parse_range(args.j1_tipping)

total_poses = len(j2_grid) * len(j3_grid) * len(j4_grid)
print(f"Grid: J2={args.j2_range} ({len(j2_grid)} pts), "
      f"J3={args.j3_range} ({len(j3_grid)} pts), "
      f"J4={args.j4_range} ({len(j4_grid)} pts)", flush=True)
print(f"Total grid poses: {total_poses}", flush=True)
print(f"J1 tipping angles: {args.j1_tipping} ({len(j1_tipping_angles)} pts)", flush=True)

# Output directory
arm_label = f"{int(ARM_REACH_IN)}arm"
load_label = "loaded" if ARM_LOADED else "unloaded"
env_label = "reactor" if args.reactor else ("step" if args.step else "flat")
out_dir = os.path.join(RESULTS_ROOT, "arm_pose_sweep", f"{arm_label}_{load_label}_{env_label}")
os.makedirs(out_dir, exist_ok=True)
csv_path = os.path.join(out_dir, "poses.csv")
json_path = os.path.join(out_dir, "summary.json")
print(f"Output: {out_dir}", flush=True)

# Pre-compute above-ground mask using analytical FK.
# J2 height above lowest possible floor:
# On flat ground: chassis center at BELLY_HEIGHT + CHASSIS_H/2 above floor.
# On step: chassis center at WHEEL_RADIUS + CHASSIS_H/2 above higher floor,
#   lower floor is 4.5" below. Use lower floor as reference for safety.
step_h_val = 4.5 * IN if args.step else 0.0
if args.step:
    # Robot center is at WHEEL_RADIUS + CHASSIS_H/2 above outer floor (Z=0).
    # Inner floor is at Z=-step_h. So distance from center to lowest floor
    # = WHEEL_RADIUS + CHASSIS_H/2 + step_h.
    chassis_center_above_lowest = WHEEL_RADIUS + CHASSIS_H / 2.0 + step_h_val
else:
    # Flat ground or reactor: belly sits on floor.
    chassis_center_above_lowest = BELLY_HEIGHT + CHASSIS_H / 2.0

Z_J2_ABOVE_FLOOR = chassis_center_above_lowest + ARM_MOUNT_Z + J1_STACK_H
# ARM_MOUNT_Z = CHASSIS_H / 2.0, so:
# Z_J2 = chassis_center_above_lowest + CHASSIS_H/2 + J1_STACK_H

print(f"J2 height above lowest floor: {Z_J2_ABOVE_FLOOR/IN:.2f}\" "
      f"({Z_J2_ABOVE_FLOOR*1000:.1f}mm)", flush=True)

SETTLE_FRAMES = 360   # 1s at 360Hz
MEASURE_FRAMES = 90   # 0.25s at 360Hz
SETTLE_TOL_DEG = 0.5
SETTLE_CONSEC = 10

above_ground_count = 0
underground_count = 0

# Pre-compute above-ground mask
above_ground_mask = {}
for j2_deg in j2_grid:
    for j3_deg in j3_grid:
        for j4_deg in j4_grid:
            j2_rad = math.radians(j2_deg)
            j3_rad = math.radians(j3_deg)
            j4_rad = math.radians(j4_deg)
            # In the arm's local XZ plane (J1=0), cumulative pitch angles.
            # L2 extends from J2 in +X direction; pitch rotates in XZ plane.
            # Z component: positive pitch = arm goes up.
            A2 = j2_rad
            A23 = j2_rad + j3_rad
            A234 = j2_rad + j3_rad + j4_rad
            z_j3 = L_L2 * math.sin(A2)
            z_j4 = z_j3 + L_L3 * math.sin(A23)
            z_ee = z_j4 + L_L4 * math.sin(A234)
            min_z = min(z_j3, z_j4, z_ee)
            above = (min_z + Z_J2_ABOVE_FLOOR) > 0
            above_ground_mask[(j2_deg, j3_deg, j4_deg)] = above
            if above:
                above_ground_count += 1
            else:
                underground_count += 1

physics_pose_count = above_ground_count
print(f"Above-ground poses: {above_ground_count} / {total_poses} "
      f"(underground skipped: {underground_count})", flush=True)

# CSV columns
CSV_FIELDS = [
    "pose_idx", "j2_deg", "j3_deg", "j4_deg", "status",
    "tau_j1_Nm", "tau_j2_Nm", "tau_j3_Nm", "tau_j4_Nm",
    "tau_j1_peak_Nm", "tau_j2_peak_Nm", "tau_j3_peak_Nm", "tau_j4_peak_Nm",
    "ee_x_m", "ee_y_m", "ee_z_m",
    "margin_front_m", "margin_back_m", "margin_left_m", "margin_right_m",
    "tipping_moment_Nm", "stable",
    "worst_j1_margin_m", "worst_j1_deg",
    "ee_contact", "reactor_contact", "contact_force_N", "contact_bodies",
    "chassis_drift_m", "arm_chassis_contact",
]

# Resume logic
completed_poses = set()
if args.resume and os.path.exists(csv_path):
    with open(csv_path, "r", newline="") as rf:
        reader = csv.DictReader(rf)
        for row in reader:
            try:
                key = (float(row["j2_deg"]), float(row["j3_deg"]), float(row["j4_deg"]))
                completed_poses.add(key)
            except (KeyError, ValueError):
                pass
    print(f"Resume: {len(completed_poses)} poses already completed", flush=True)

# ============================================================================
# Build scene
# ============================================================================
src_parts, src_center = read_source_wheel()

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10):
    app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_physics_scene(stage)

step_height = 0.0
if args.reactor:
    print(f"Loading reactor: {REACTOR_USD}", flush=True)
    rp = stage.DefinePrim("/World/Reactor", "Xform")
    rp.GetReferences().AddReference(REACTOR_USD)
    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)
elif args.step:
    step_height = build_step_env(stage)
else:
    build_ground(stage)

robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names = build_robot(
    stage, src_parts, src_center)

# --- End-effector contact reporting setup (before timeline play) ---
from isaacsim.sensors.physics import _sensor as _contact_sensor
ee_path = "/World/Robot/ArmJ4wrist"
ee_prim = stage.GetPrimAtPath(ee_path)
if not ee_prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
    cr_api = PhysxSchema.PhysxContactReportAPI.Apply(ee_prim)
else:
    cr_api = PhysxSchema.PhysxContactReportAPI(ee_prim)
cr_api.CreateThresholdAttr().Set(0)
PhysxSchema.PhysxRigidBodyAPI(ee_prim).CreateSleepThresholdAttr(0.0)
contact_sensor = _contact_sensor.acquire_contact_sensor_interface()
print("Contact reporting ENABLED on end effector (ArmJ4wrist)", flush=True)

# Contact reporting on chassis body (detect arm-to-chassis collisions)
chassis_prim = stage.GetPrimAtPath(chassis_path)
if not chassis_prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
    cr_chassis = PhysxSchema.PhysxContactReportAPI.Apply(chassis_prim)
else:
    cr_chassis = PhysxSchema.PhysxContactReportAPI(chassis_prim)
cr_chassis.CreateThresholdAttr().Set(0)
print("Contact reporting ENABLED on chassis (arm collision detection)", flush=True)

CHASSIS_DRIFT_THRESH = 0.003  # 3mm — if chassis moves this much, pose is invalid

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
    app.close()
    sys.exit(1)

ndof = art.num_dof
dof_names = art.dof_names
print(f"\nArticulation: {ndof} DOFs", flush=True)
print(f"DOF names: {dof_names}", flush=True)

# Map drive DOFs
drive_dof_indices = []
for djp in drive_joint_paths:
    joint_name = djp.split("/")[-1]
    for i, dn in enumerate(dof_names):
        if joint_name in dn or dn == joint_name:
            drive_dof_indices.append(i)
            break

if len(drive_dof_indices) != 4:
    drive_dof_indices = []
    for i, dn in enumerate(dof_names):
        if "Drive" in dn or "drive" in dn:
            drive_dof_indices.append(i)
    if len(drive_dof_indices) != 4:
        print(f"WARNING: Found {len(drive_dof_indices)} drive DOFs, using first 4", flush=True)
        drive_dof_indices = list(range(4))
print(f"Drive DOF indices: {drive_dof_indices}", flush=True)

# Map arm DOFs
arm_dof_indices = []
for ajn in arm_joint_names:
    for i, dn in enumerate(dof_names):
        if ajn in dn or dn == ajn:
            arm_dof_indices.append(i)
            break

if len(arm_dof_indices) != 4:
    print(f"WARNING: Found {len(arm_dof_indices)} arm DOFs, expected 4", flush=True)
    arm_dof_indices = []
    for i, dn in enumerate(dof_names):
        if "Arm" in dn:
            arm_dof_indices.append(i)
    if len(arm_dof_indices) != 4:
        print(f"FATAL: Cannot map arm DOFs, found {len(arm_dof_indices)}", flush=True)

print(f"Arm DOF indices: {arm_dof_indices}", flush=True)
for i, idx in enumerate(arm_dof_indices):
    print(f"  {arm_joint_names[i]}: DOF[{idx}] = \"{dof_names[idx]}\"", flush=True)

# Switch non-arm DOFs to velocity mode, lock wheels at zero
non_arm_dof_indices = np.array([i for i in range(ndof) if i not in arm_dof_indices])
art.switch_control_mode("velocity", joint_indices=non_arm_dof_indices)

# Lock all wheels at zero velocity
va_lock = np.zeros(ndof)
if art.is_physics_handle_valid():
    art.set_joint_velocity_targets(va_lock.reshape(1, -1))

# Settle 2s
print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    world.step(render=False)
    if i % (PHYSICS_HZ // 2) == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"  [{i/PHYSICS_HZ:.1f}s] Z={p[2]:.4f}m ({p[2]/IN:.2f}\")", flush=True)

print("\n=== INITIAL STATE (stowed) ===", flush=True)
s = get_state(art, stage, chassis_path)
if s:
    p = s["pos"]
    print(f"  Chassis: ({p[0]/IN:.1f}, {p[1]/IN:.1f}, {p[2]/IN:.1f})\"", flush=True)

# Save initial joint positions and chassis pose for reset between poses
init_jp = art.get_joint_positions().copy()
init_jv = np.zeros_like(init_jp)
init_chassis_pos = s["pos"].copy() if s else np.zeros(3)
print(f"  Saved initial state for inter-pose reset", flush=True)

# ============================================================================
# Sweep loop
# ============================================================================
start_time = time.time()
start_utc = datetime.datetime.utcnow().isoformat() + "Z"

# Open CSV (append if resuming, write if fresh)
csv_mode = "a" if (args.resume and os.path.exists(csv_path) and len(completed_poses) > 0) else "w"
csv_file = open(csv_path, csv_mode, newline="")
writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
if csv_mode == "w":
    writer.writeheader()
    csv_file.flush()

# Tracking for JSON summary
all_torques = {f"j{k}": [] for k in range(1, 5)}
all_peak_torques = {f"j{k}": [] for k in range(1, 5)}
peak_torque_poses = {f"j{k}": (0.0, (0, 0, 0)) for k in range(1, 5)}
n_tipping_j1_0 = 0
n_tipping_any_j1 = 0
n_contact = 0
n_reactor_contact = 0
all_contact_forces = []
all_contacted_bodies = set()
all_ee_z = []
all_ee_radius = []
pose_times = []
n_collision = 0

pose_idx = 0
physics_done = 0

print(f"\nStarting sweep: {total_poses} total, {physics_pose_count} physics poses", flush=True)

for j2_deg in j2_grid:
    for j3_deg in j3_grid:
        for j4_deg in j4_grid:
            pose_idx += 1
            key = (float(j2_deg), float(j3_deg), float(j4_deg))

            # Skip already completed (resume)
            if key in completed_poses:
                continue

            # Check underground
            if not above_ground_mask.get(key, False):
                row = {f: float('nan') for f in CSV_FIELDS}
                row["pose_idx"] = pose_idx
                row["j2_deg"] = j2_deg
                row["j3_deg"] = j3_deg
                row["j4_deg"] = j4_deg
                row["status"] = "underground"
                row["stable"] = ""
                row["ee_contact"] = ""
                row["reactor_contact"] = ""
                row["contact_bodies"] = ""
                row["chassis_drift_m"] = ""
                row["arm_chassis_contact"] = ""
                writer.writerow(row)
                csv_file.flush()
                continue

            pose_t0 = time.time()
            physics_done += 1

            # Snapshot chassis position before commanding arm
            s_pre = get_state(art, stage, chassis_path)
            chassis_pos_before = s_pre["pos"].copy() if s_pre is not None else np.zeros(3)

            # Command pose: J1=0, J2=j2, J3=j3, J4=j4
            target_angles = [0.0, float(j2_deg), float(j3_deg), float(j4_deg)]
            for i, ajn in enumerate(ARM_JOINT_NAMES):
                jp = f"/World/Robot/{ajn}"
                jprim = stage.GetPrimAtPath(jp)
                if jprim.IsValid():
                    UsdPhysics.DriveAPI(jprim, "angular").GetTargetPositionAttr().Set(
                        float(target_angles[i]))

            # Keep wheels locked
            if art.is_physics_handle_valid():
                art.set_joint_velocity_targets(va_lock.reshape(1, -1))

            # Settle: step until arm joints within tolerance or timeout
            # Also detect early collision (chassis drift) and bail out fast
            consec_ok = 0
            settled = False
            early_collision = False
            for sf in range(SETTLE_FRAMES):
                world.step(render=False)
                # Check for chassis drift every 60 frames (~0.17s)
                if sf > 0 and sf % 60 == 0:
                    s_check = get_state(art, stage, chassis_path)
                    if s_check is not None:
                        drift = np.linalg.norm(s_check["pos"] - chassis_pos_before)
                        if drift > CHASSIS_DRIFT_THRESH * 3:  # 9mm = clearly colliding
                            early_collision = True
                            break
                try:
                    jp_now = art.get_joint_positions()
                    if jp_now is not None:
                        jp_flat = jp_now.flatten()
                        all_ok = True
                        for ai, aidx in enumerate(arm_dof_indices):
                            actual_deg = math.degrees(jp_flat[aidx])
                            if abs(actual_deg - target_angles[ai]) > SETTLE_TOL_DEG:
                                all_ok = False
                                break
                        if all_ok:
                            consec_ok += 1
                            if consec_ok >= SETTLE_CONSEC:
                                settled = True
                                break
                        else:
                            consec_ok = 0
                except:
                    pass

            # If early collision detected, skip measurement entirely
            if early_collision:
                n_collision += 1
                row = {
                    "pose_idx": pose_idx,
                    "j2_deg": j2_deg, "j3_deg": j3_deg, "j4_deg": j4_deg,
                    "status": "collision",
                }
                for f in CSV_FIELDS:
                    if f not in row:
                        row[f] = "nan"
                row["chassis_drift_m"] = f"{float(np.linalg.norm(get_state(art, stage, chassis_path)['pos'] - chassis_pos_before)):.6f}"
                row["arm_chassis_contact"] = "1"
                row["stable"] = ""
                row["ee_contact"] = ""
                row["reactor_contact"] = ""
                row["contact_bodies"] = ""
                writer.writerow(row)
                csv_file.flush()
                # Reset and continue
                try:
                    art.set_joint_positions(init_jp)
                    art.set_joint_velocities(init_jv)
                    art.set_joint_velocity_targets(va_lock.reshape(1, -1))
                    for _rf in range(10):
                        world.step(render=False)
                except Exception:
                    pass
                pose_dt = time.time() - pose_t0
                pose_times.append(pose_dt)
                if physics_done % 50 == 0 or physics_done == physics_pose_count:
                    elapsed = time.time() - start_time
                    mean_pt = np.mean(pose_times) if pose_times else 0
                    remaining = (physics_pose_count - physics_done) * mean_pt
                    eta_min = remaining / 60.0
                    print(f"  [{physics_done}/{physics_pose_count}] "
                          f"j2={j2_deg:+.0f} j3={j3_deg:+.0f} j4={j4_deg:+.0f} "
                          f"status=collision(early) "
                          f"elapsed={elapsed/60:.1f}min ETA={eta_min:.1f}min "
                          f"({mean_pt:.2f}s/pose)", flush=True)
                continue

            # Measure: collect torques and contacts over MEASURE_FRAMES
            torque_samples = [[] for _ in range(4)]
            contact_any = False
            contact_reactor = False
            contact_force_sum = 0.0
            contact_count = 0
            contact_bodies_set = set()
            arm_chassis_hit = False

            for mf in range(MEASURE_FRAMES):
                world.step(render=False)

                s = get_state(art, stage, chassis_path)
                if s and s["je"] is not None:
                    je = s["je"].flatten()
                    for ai, aidx in enumerate(arm_dof_indices):
                        torque_samples[ai].append(je[aidx])

                contacts = read_ee_contacts(contact_sensor, ee_path)
                if contacts:
                    contact_any = True
                    for c in contacts:
                        contact_bodies_set.add(c["other"])
                        contact_force_sum += c["force_mag"]
                        contact_count += 1
                        # Check if contact is with reactor/floor (not arm parts)
                        other = c["other"]
                        if ("Robot" not in other and
                            ("Reactor" in other or "Ground" in other or
                             "Step" in other or "Floor" in other)):
                            contact_reactor = True

                # Check arm-to-chassis contact
                chassis_hits = read_chassis_contacts(contact_sensor, chassis_path)
                if chassis_hits:
                    arm_chassis_hit = True

            # Chassis drift: did the arm push the robot?
            s_post = get_state(art, stage, chassis_path)
            chassis_pos_after = s_post["pos"] if s_post is not None else chassis_pos_before
            chassis_drift = float(np.linalg.norm(chassis_pos_after - chassis_pos_before))

            # Compute metrics
            mean_torques = []
            peak_torques = []
            for ai in range(4):
                if torque_samples[ai]:
                    arr = np.array(torque_samples[ai])
                    mean_torques.append(float(np.mean(arr)))
                    peak_torques.append(float(np.max(np.abs(arr))))
                else:
                    mean_torques.append(float('nan'))
                    peak_torques.append(float('nan'))

            # EE position
            ee_pos = arm_tip_world(stage)
            if ee_pos is not None:
                ee_x, ee_y, ee_z = float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])
                all_ee_z.append(ee_z)
                ee_r = math.sqrt(ee_x**2 + ee_y**2)
                all_ee_radius.append(ee_r)
            else:
                ee_x, ee_y, ee_z = float('nan'), float('nan'), float('nan')

            # Tipping margins at J1=0
            tip_data = compute_tipping_margins(stage, chassis_path, arm_joint_names)
            stable_j1_0 = tip_data["stable"]
            if not stable_j1_0:
                n_tipping_j1_0 += 1

            # Analytical J1 tipping
            worst_j1_deg, worst_j1_margin = analytical_j1_tipping(
                stage, chassis_path, j1_tipping_angles)
            if worst_j1_margin <= 0:
                n_tipping_any_j1 += 1

            # Contact stats
            if contact_any:
                n_contact += 1
                all_contacted_bodies.update(contact_bodies_set)
            if contact_reactor:
                n_reactor_contact += 1
            mean_contact_force = contact_force_sum / max(contact_count, 1)
            if contact_any:
                all_contact_forces.append(mean_contact_force)

            # Track torque peaks for summary
            for ai in range(4):
                jk = f"j{ai+1}"
                if not math.isnan(mean_torques[ai]):
                    all_torques[jk].append(abs(mean_torques[ai]))
                if not math.isnan(peak_torques[ai]):
                    all_peak_torques[jk].append(peak_torques[ai])
                    prev_peak = peak_torque_poses[jk][0]
                    if peak_torques[ai] > prev_peak:
                        peak_torque_poses[jk] = (peak_torques[ai],
                                                 (j2_deg, j3_deg, j4_deg))

            # Status: collision if arm hit chassis or pushed the robot
            if arm_chassis_hit or chassis_drift > CHASSIS_DRIFT_THRESH:
                status = "collision"
                n_collision += 1
            elif settled:
                status = "settled"
            else:
                status = "timeout"

            # Write CSV row
            row = {
                "pose_idx": pose_idx,
                "j2_deg": j2_deg,
                "j3_deg": j3_deg,
                "j4_deg": j4_deg,
                "status": status,
                "tau_j1_Nm": f"{mean_torques[0]:.4f}",
                "tau_j2_Nm": f"{mean_torques[1]:.4f}",
                "tau_j3_Nm": f"{mean_torques[2]:.4f}",
                "tau_j4_Nm": f"{mean_torques[3]:.4f}",
                "tau_j1_peak_Nm": f"{peak_torques[0]:.4f}",
                "tau_j2_peak_Nm": f"{peak_torques[1]:.4f}",
                "tau_j3_peak_Nm": f"{peak_torques[2]:.4f}",
                "tau_j4_peak_Nm": f"{peak_torques[3]:.4f}",
                "ee_x_m": f"{ee_x:.6f}",
                "ee_y_m": f"{ee_y:.6f}",
                "ee_z_m": f"{ee_z:.6f}",
                "margin_front_m": f"{tip_data['margin_front']:.6f}",
                "margin_back_m": f"{tip_data['margin_back']:.6f}",
                "margin_left_m": f"{tip_data['margin_left']:.6f}",
                "margin_right_m": f"{tip_data['margin_right']:.6f}",
                "tipping_moment_Nm": f"{tip_data['tipping_moment_Nm']:.4f}",
                "stable": "1" if stable_j1_0 else "0",
                "worst_j1_margin_m": f"{worst_j1_margin:.6f}",
                "worst_j1_deg": f"{worst_j1_deg:.1f}",
                "ee_contact": "1" if contact_any else "0",
                "reactor_contact": "1" if contact_reactor else "0",
                "contact_force_N": f"{mean_contact_force:.4f}" if contact_any else "0.0",
                "contact_bodies": ";".join(sorted(contact_bodies_set)) if contact_bodies_set else "",
                "chassis_drift_m": f"{chassis_drift:.6f}",
                "arm_chassis_contact": "1" if arm_chassis_hit else "0",
            }
            writer.writerow(row)
            csv_file.flush()

            # Reset robot to initial state: teleport chassis back, zero velocities,
            # return arm to stowed. Prevents collision drift from cascading.
            try:
                art.set_joint_positions(init_jp)
                art.set_joint_velocities(init_jv)
                # Re-lock wheels
                art.set_joint_velocity_targets(va_lock.reshape(1, -1))
                # Let physics re-settle briefly (10 frames = ~28ms)
                for _rf in range(10):
                    world.step(render=False)
            except Exception:
                pass

            pose_dt = time.time() - pose_t0
            pose_times.append(pose_dt)

            # Progress every 50 physics poses
            if physics_done % 50 == 0 or physics_done == physics_pose_count:
                elapsed = time.time() - start_time
                mean_pt = np.mean(pose_times) if pose_times else 0
                remaining = (physics_pose_count - physics_done) * mean_pt
                eta_min = remaining / 60.0
                print(f"  [{physics_done}/{physics_pose_count}] "
                      f"j2={j2_deg:+.0f} j3={j3_deg:+.0f} j4={j4_deg:+.0f} "
                      f"status={status} "
                      f"elapsed={elapsed/60:.1f}min ETA={eta_min:.1f}min "
                      f"({mean_pt:.2f}s/pose)", flush=True)

csv_file.close()

end_time = time.time()
end_utc = datetime.datetime.utcnow().isoformat() + "Z"
elapsed_s = end_time - start_time
mean_pose_time = np.mean(pose_times) if pose_times else 0.0

print(f"\nSweep complete: {physics_done} physics poses in {elapsed_s/60:.1f} min",
      flush=True)
print(f"CSV: {csv_path}", flush=True)

# ============================================================================
# JSON summary
# ============================================================================
def safe_list(lst):
    return [float(x) for x in lst]

torque_summary = {}
for jk in ["j1", "j2", "j3", "j4"]:
    t_list = all_torques[jk]
    p_list = all_peak_torques[jk]
    torque_summary[jk] = {
        "mean_abs_Nm": float(np.mean(t_list)) if t_list else 0.0,
        "peak_abs_Nm": float(np.max(p_list)) if p_list else 0.0,
        "peak_pose_deg": list(peak_torque_poses[jk][1]) if p_list else [0, 0, 0],
    }

n_stable_all_j1 = physics_done - n_tipping_any_j1
pct_stable = (n_stable_all_j1 / max(physics_done, 1)) * 100.0

# Worst tipping pose: find from CSV (we didn't track it directly, use peak torque proxy)
# Actually track worst margin separately
worst_tipping_margin = float('inf')
worst_tipping_pose = [0, 0, 0]
worst_tipping_j1 = 0.0

# Re-read CSV to find worst tipping
try:
    with open(csv_path, "r", newline="") as rf:
        reader = csv.DictReader(rf)
        for row in reader:
            if row["status"] == "underground":
                continue
            try:
                wm = float(row["worst_j1_margin_m"])
                if wm < worst_tipping_margin:
                    worst_tipping_margin = wm
                    worst_tipping_pose = [float(row["j2_deg"]),
                                          float(row["j3_deg"]),
                                          float(row["j4_deg"])]
                    worst_tipping_j1 = float(row["worst_j1_deg"])
            except (KeyError, ValueError):
                pass
except:
    pass

summary = {
    "config": {
        "arm_inches": int(ARM_REACH_IN),
        "arm_segments_in": [_l2_in, _l3_in, _l4_in],
        "loaded": ARM_LOADED,
        "gripper_mass_kg": M_GRIPPER,
        "environment": env_label,
        "physics_hz": PHYSICS_HZ,
        "belly_height_in": args.belly,
        "settle_frames": SETTLE_FRAMES,
        "measure_frames": MEASURE_FRAMES,
        "chassis_mass_kg": CHASSIS_MASS,
    },
    "grid": {
        "j2_range_deg": args.j2_range,
        "j3_range_deg": args.j3_range,
        "j4_range_deg": args.j4_range,
        "j1_tipping_deg": args.j1_tipping,
        "total_poses": total_poses,
        "above_ground_poses": above_ground_count,
        "physics_poses": physics_done,
    },
    "timing": {
        "start_utc": start_utc,
        "end_utc": end_utc,
        "elapsed_s": round(elapsed_s, 1),
        "mean_pose_time_s": round(mean_pose_time, 3),
    },
    "torques": torque_summary,
    "tipping": {
        "n_tipping_at_j1_0": n_tipping_j1_0,
        "n_tipping_any_j1": n_tipping_any_j1,
        "worst_margin_m": float(worst_tipping_margin) if worst_tipping_margin != float('inf') else 0.0,
        "worst_pose_deg": worst_tipping_pose,
        "worst_j1_deg": worst_tipping_j1,
        "pct_stable_all_j1": round(pct_stable, 1),
    },
    "collision": {
        "n_collision_poses": n_collision,
        "pct_collision": round((n_collision / max(physics_done, 1)) * 100.0, 1),
        "chassis_drift_thresh_m": CHASSIS_DRIFT_THRESH,
    },
    "contact": {
        "n_contact_poses": n_contact,
        "n_reactor_contact_poses": n_reactor_contact,
        "pct_contact": round((n_contact / max(physics_done, 1)) * 100.0, 1),
        "mean_contact_force_N": round(float(np.mean(all_contact_forces)), 3) if all_contact_forces else 0.0,
        "peak_contact_force_N": round(float(np.max(all_contact_forces)), 3) if all_contact_forces else 0.0,
        "contacted_bodies": sorted(list(all_contacted_bodies)),
    },
    "reach": {
        "max_ee_z_m": round(float(np.max(all_ee_z)), 4) if all_ee_z else 0.0,
        "min_ee_z_m": round(float(np.min(all_ee_z)), 4) if all_ee_z else 0.0,
        "max_ee_radius_m": round(float(np.max(all_ee_radius)), 4) if all_ee_radius else 0.0,
    },
}

with open(json_path, "w") as jf:
    json.dump(summary, jf, indent=2)
print(f"JSON: {json_path}", flush=True)

# Print summary
print(f"\n{'='*60}", flush=True)
print("SWEEP SUMMARY", flush=True)
print(f"{'='*60}", flush=True)
print(f"  Config: {int(ARM_REACH_IN)}\" arm, {'loaded' if ARM_LOADED else 'unloaded'}, "
      f"{env_label}", flush=True)
print(f"  Grid: {total_poses} total, {above_ground_count} above ground, "
      f"{physics_done} physics", flush=True)
print(f"  Time: {elapsed_s/60:.1f} min ({mean_pose_time:.2f}s/pose)", flush=True)
print(f"\n  Torques (mean|peak Nm):", flush=True)
for jk in ["j1", "j2", "j3", "j4"]:
    ts = torque_summary[jk]
    print(f"    {jk.upper()}: {ts['mean_abs_Nm']:.3f} | {ts['peak_abs_Nm']:.3f} Nm "
          f"@ j2={ts['peak_pose_deg'][0]:.0f} j3={ts['peak_pose_deg'][1]:.0f} "
          f"j4={ts['peak_pose_deg'][2]:.0f}", flush=True)
print(f"\n  Tipping:", flush=True)
print(f"    At J1=0: {n_tipping_j1_0} unstable poses", flush=True)
print(f"    Any J1:  {n_tipping_any_j1} unstable poses", flush=True)
print(f"    Stable (all J1): {pct_stable:.1f}%", flush=True)
print(f"    Worst margin: {worst_tipping_margin*1000:.1f}mm "
      f"@ j2={worst_tipping_pose[0]:.0f} j3={worst_tipping_pose[1]:.0f} "
      f"j4={worst_tipping_pose[2]:.0f} j1={worst_tipping_j1:.0f}", flush=True)
print(f"\n  Arm-chassis collision:", flush=True)
print(f"    {n_collision} poses ({(n_collision/max(physics_done,1))*100:.1f}%) — arm hit chassis or pushed robot", flush=True)
print(f"\n  Contact:", flush=True)
print(f"    EE contact: {n_contact} poses ({(n_contact/max(physics_done,1))*100:.1f}%)", flush=True)
print(f"    Reactor contact: {n_reactor_contact} poses", flush=True)
if all_contact_forces:
    print(f"    Force: mean={np.mean(all_contact_forces):.2f} N, "
          f"peak={np.max(all_contact_forces):.2f} N", flush=True)
print(f"\n  Reach:", flush=True)
if all_ee_z:
    print(f"    EE Z: {np.min(all_ee_z)/IN:.1f}\" to {np.max(all_ee_z)/IN:.1f}\"", flush=True)
if all_ee_radius:
    print(f"    EE radius: max {np.max(all_ee_radius)/IN:.1f}\"", flush=True)
print(f"{'='*60}", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
