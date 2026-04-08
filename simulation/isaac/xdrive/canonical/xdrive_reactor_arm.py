"""
FORTIS X-drive chassis + 4-DOF parallel-link arm (configurable reach).

Built on xdrive_realwheel.py (canonical arched chassis + real Kaya omni wheel
meshes). Adds a 4-DOF arm (J1 yaw + J2-J4 pitch) with flat-stowed zigzag
layout, mounted at chassis back edge.

Arm configs (--36arm / --30arm / --24arm):
  36": L2=17" L3=15" L4=4"   (default)
  30": L2=15" L3=12" L4=3"
  24": L2=12" L3=10" L4=2"

--armloaded adds max payload (3 lb / 1361g) to gripper (500g) at EE tip.

Links: CF square tube 0.79"x0.79", 0.0053 lb/in linear density (STD).
Joints: 0.629 kg each (NEMA 17 0.80Nm + Cricket MK II 25:1 + hardware).
Camera: Orbbec Gemini 2, 98g, on J2 shoulder.

Usage: IsaacSim\\python.bat canonical/xdrive_reactor_arm.py --gui --36arm
       IsaacSim\\python.bat canonical/xdrive_reactor_arm.py --gui --reactor --30arm
       IsaacSim\\python.bat canonical/xdrive_reactor_arm.py --gui --24arm --armloaded
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
parser.add_argument("--reactor", action="store_true",
                    help="Load diiid_reactor.usd and spawn on outer floor")
parser.add_argument("--step", action="store_true",
                    help="Flat step env (two planes with 4.5\" step, no reactor)")
parser.add_argument("--36arm", action="store_true", dest="arm36",
                    help="36\" arm: 17+15+4 (default)")
parser.add_argument("--30arm", action="store_true", dest="arm30",
                    help="30\" arm: 15+12+3")
parser.add_argument("--24arm", action="store_true", dest="arm24",
                    help="24\" arm: 12+10+2")
parser.add_argument("--armloaded", action="store_true",
                    help="Loaded EE: gripper 500g + max payload 3lb (1361g) at tip")
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
# Constants — chassis & wheels (identical to xdrive_realwheel.py)
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

# Drive
DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5

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
# J1 motor stack height — flush mount with ~1.5" clearance above chassis top
J1_STACK_H = 1.5 * IN  # 38.1mm (motor+gearbox recessed into chassis)

# Arm configurations: (L2_in, L3_in, L4_in)
ARM_CONFIGS = {
    36: (17.0, 15.0, 4.0),
    30: (15.0, 12.0, 3.0),
    24: (12.0, 10.0, 2.0),
}

# Select arm config from CLI (default 36")
_arm_sel = 36
if args.arm30: _arm_sel = 30
if args.arm24: _arm_sel = 24
if args.arm36: _arm_sel = 36  # explicit override
_l2_in, _l3_in, _l4_in = ARM_CONFIGS[_arm_sel]

# Link lengths
L_L2 = _l2_in * IN
L_L3 = _l3_in * IN
L_L4 = _l4_in * IN
ARM_REACH_IN = _l2_in + _l3_in + _l4_in

# Joint mass: motor (0.58 kg) + hardware (49g) = 0.629 kg each
M_JOINT = 0.629

# CF tube: square 0.79"x0.79", linear density 0.0053 lb/in (STD)
CF_DENSITY_LB_PER_IN = 0.0053
CF_DENSITY_KG_PER_M = CF_DENSITY_LB_PER_IN * 0.453592 / IN
M_L2 = CF_DENSITY_KG_PER_M * L_L2
M_L3 = CF_DENSITY_KG_PER_M * L_L3
M_L4 = CF_DENSITY_KG_PER_M * L_L4

# End-effector tip mass
M_GRIPPER_BARE = 0.500   # gripper hardware: 500g
M_PAYLOAD = 1.361         # max payload: 3 lb = 1361g
if args.armloaded:
    M_GRIPPER = M_GRIPPER_BARE + M_PAYLOAD  # 1.861 kg
else:
    M_GRIPPER = M_GRIPPER_BARE              # 0.500 kg
ARM_LOADED = args.armloaded

# Depth camera on J2 shoulder (near joint, looking down the arm)
M_CAMERA = 0.098  # Orbbec Gemini 2
CAM_X_ON_L2 = 2.0 * IN  # 2" from J2 joint along L2 (not too close to joint pivot)

# CF tube cross-section (for visual boxes)
CF_TUBE_SIZE = 0.79 * IN  # 20.07mm

# Arm mount: back edge of chassis, centerline, top surface
ARM_MOUNT_X = -CHASSIS_L / 2.0
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0

# Parallel layout: links side-by-side in Y on chassis top, all at same Z.
# Each link is CF_TUBE_SIZE wide (~20mm), add 5mm gap between tubes.
LINK_Y_SPACING = CF_TUBE_SIZE + 0.005  # ~25mm center-to-center
# Center 3 links around Y=0: offsets at -1, 0, +1 * spacing
LINK_Y = [(-1.0) * LINK_Y_SPACING,   # L2
          ( 0.0) * LINK_Y_SPACING,   # L3
          (+1.0) * LINK_Y_SPACING]   # L4

# DriveAPI for arm joints (position mode, Nm units via USD deg convention)
ARM_STIFFNESS = 50.0     # Nm/deg — converges with <0.5° error, no oscillation
ARM_DAMPING   = 10.0     # Nm*s/deg — critically damped for arm inertia
ARM_MAX_FORCE = 500.0    # Nm — high so we measure what's NEEDED, not clamp it

# Joint limits: all +-180 for testing. Real limits applied in sweep later.
ARM_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),
    "ArmJ2": (-180.0, 180.0),
    "ArmJ3": (-180.0, 180.0),
    "ArmJ4": (-180.0, 180.0),
}

ARM_JOINT_NAMES = ["ArmJ1", "ArmJ2", "ArmJ3", "ArmJ4"]


# ============================================================================
# Source wheel data reader (from xdrive_realwheel.py, unchanged)
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
# Scene builders (from xdrive_realwheel.py, unchanged)
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
    """Two flat planes with a 4.5" step between them — simplified reactor floor.

    Layout (top view, centered at origin):
      Outer floor (higher, Z=0):  X from -2m to +2m, Y from -2m to 0
      Inner floor (lower, Z=-step): X from -2m to +2m, Y from 0 to +2m
      Step face (vertical):       along Y=0, height = step_height

    Robot spawns straddling the step edge (Y~0).
    """
    step_h = 4.5 * IN   # 0.1143m
    half = 2.0           # 2m half-size for each plane

    # Outer floor (higher) at Z=0, Y<0
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

    # Inner floor (lower) at Z=-step_h, Y>0
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

    # Step face (vertical wall at Y=0 from Z=-step_h to Z=0)
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

    # Friction material (graphite)
    smat = UsdShade.Material.Define(stage, "/World/StepMat")
    spm = UsdPhysics.MaterialAPI.Apply(smat.GetPrim())
    spm.CreateStaticFrictionAttr(FRICTION_MU)
    spm.CreateDynamicFrictionAttr(FRICTION_MU)
    spm.CreateRestitutionAttr(0.05)
    for p in [outer, inner, face]:
        UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(
            smat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # Step edge line (visual indicator)
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
# Chassis builder (from xdrive_realwheel.py, unchanged)
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
# Omni wheel builder (from xdrive_realwheel.py, unchanged)
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
# Arm build helpers
# ============================================================================

def _apply_arm_body_mass_x(prim, joint_mass, link_mass, link_length, link_dir,
                           extra_mass=0.0, extra_x=0.0):
    """Set mass, COM, and inertia for an arm body with link along X.

    joint_mass: concentrated at body origin (joint pivot).
    link_mass:  uniform box (CF_TUBE_SIZE x CF_TUBE_SIZE x link_length) along
                link_dir * X.
    extra_mass: point mass at extra_x (e.g. camera).
    """
    total = joint_mass + link_mass + extra_mass
    link_cx = link_dir * link_length / 2.0
    com_x = (joint_mass * 0.0 + link_mass * link_cx + extra_mass * extra_x) / total

    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0.0, 0.0))

    # Link box inertia about its own center
    Lx = max(float(link_length), 0.01)
    Ly = CF_TUBE_SIZE
    Lz = CF_TUBE_SIZE
    Ixx_link = link_mass / 12.0 * (Ly**2 + Lz**2)
    Iyy_link = link_mass / 12.0 * (Lx**2 + Lz**2)
    Izz_link = link_mass / 12.0 * (Lx**2 + Ly**2)

    # Parallel-axis shift: link center to composite COM
    d_link = link_cx - com_x
    Iyy_link += link_mass * d_link**2
    Izz_link += link_mass * d_link**2

    # Joint point mass at origin -> composite COM
    d_joint = -com_x
    Iyy_joint = joint_mass * d_joint**2
    Izz_joint = joint_mass * d_joint**2

    # Extra point mass at extra_x -> composite COM
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
    Lx = 0.057  # NEMA 17 face width
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
    cube.GetPurposeAttr().Set(UsdGeom.Tokens.guide)  # invisible in render
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
# build_arm — 5-DOF parallel-link arm, flat stowed zigzag
# ============================================================================

def build_arm(stage, robot_path, chassis_path):
    """Build the 4-DOF arm as rigid bodies in the chassis articulation.

    Flat stowed pose at q = (0,0,0,0): arm lays folded on top of the chassis
    with links side-by-side in Y (parallel layout). Link lengths set by
    --36arm / --30arm / --24arm CLI flag.

      J1_base      : motor stack from chassis top                        (+Z)
      J2_shoulder  : L2 along +X at Y=LINK_Y[0], camera near joint
      J3_elbow     : L3 along -X at Y=LINK_Y[1]
      J4_wrist     : L4 along +X at Y=LINK_Y[2], gripper at tip

    All links at same Z, offset in Y so they lay side-by-side.
    """
    j1_path = robot_path + "/ArmJ1base"
    j2_path = robot_path + "/ArmJ2shoulder"
    j3_path = robot_path + "/ArmJ3elbow"
    j4_path = robot_path + "/ArmJ4wrist"

    z_arm = ARM_MOUNT_Z + J1_STACK_H  # all links at this Z
    y_l2, y_l3, y_l4 = LINK_Y

    # Colors for each link (distinct for easy identification)
    C_J1 = (0.55, 0.55, 0.60)  # gray (motor stack)
    C_L2 = (0.85, 0.30, 0.25)  # red
    C_L3 = (0.25, 0.75, 0.30)  # green
    C_L4 = (0.25, 0.30, 0.85)  # blue

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

    # ---- Body 2: J2 shoulder (L2 17", +X) at Y=y_l2, depth camera near joint ----
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
    # Depth camera visual (dark box on top of L2, near J2 joint)
    cam = UsdGeom.Cube.Define(stage, j2_path + "/camera")
    cam.GetSizeAttr().Set(1.0)
    cxf = UsdGeom.Xformable(cam.GetPrim()); cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(CAM_X_ON_L2, 0.0, CF_TUBE_SIZE / 2.0 + 0.015))
    cxf.AddScaleOp().Set(Gf.Vec3d(0.060, 0.025, 0.025))  # ~60x25x25mm camera body
    cam.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.05, 0.05, 0.05)]))
    jsph = UsdGeom.Sphere.Define(stage, j2_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L2)]))

    # ---- Body 3: J3 elbow (L3 15", -X) at Y=y_l3 ----
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

    # ---- Body 4: J4 wrist (L4 4", +X, gripper at tip) at Y=y_l4 ----
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
    # Collision for L4 link + gripper (one box covering both)
    grip_col_len = L_L4 + 0.030  # link + gripper extent
    _add_arm_collision_box(stage, j4_path + "/col",
                           center=(grip_col_len / 2.0, 0.0, 0.0),
                           size=(grip_col_len, 0.040, CF_TUBE_SIZE))
    # Gripper visual (small box at tip)
    grip = UsdGeom.Cube.Define(stage, j4_path + "/gripper")
    grip.GetSizeAttr().Set(1.0)
    gxf = UsdGeom.Xformable(grip.GetPrim()); gxf.ClearXformOpOrder()
    gxf.AddTranslateOp().Set(Gf.Vec3d(L_L4, 0.0, 0.0))
    gxf.AddScaleOp().Set(Gf.Vec3d(0.030, 0.040, 0.025))
    grip.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.15, 0.15, 0.15)]))
    # Tip marker (red sphere at gripper)
    tip = UsdGeom.Sphere.Define(stage, j4_path + "/tip")
    tip.GetRadiusAttr().Set(0.012)
    txf = UsdGeom.Xformable(tip.GetPrim()); txf.ClearXformOpOrder()
    txf.AddTranslateOp().Set(Gf.Vec3d(L_L4 + 0.015, 0.0, 0.0))
    tip.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.0, 0.0)]))
    jsph = UsdGeom.Sphere.Define(stage, j4_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L4)]))

    # ---- Joints ----
    # J1: chassis -> J1base, Z yaw
    _add_arm_joint(stage, robot_path + "/ArmJ1",
                   body0=chassis_path, body1=j1_path,
                   localPos0=(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z),
                   axis="Z", limit_deg=ARM_LIMITS_DEG["ArmJ1"])

    # J2: J1base -> J2shoulder, Y pitch (step from J1 top to L2 Y lane)
    _add_arm_joint(stage, robot_path + "/ArmJ2",
                   body0=j1_path, body1=j2_path,
                   localPos0=(0.0, y_l2, J1_STACK_H),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ2"])

    # J3: J2shoulder -> J3elbow (at L2 tip, shift to L3 Y lane)
    _add_arm_joint(stage, robot_path + "/ArmJ3",
                   body0=j2_path, body1=j3_path,
                   localPos0=(L_L2, y_l3 - y_l2, 0.0),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ3"])

    # J4: J3elbow -> J4wrist (at L3 tip, shift to L4 Y lane)
    _add_arm_joint(stage, robot_path + "/ArmJ4",
                   body0=j3_path, body1=j4_path,
                   localPos0=(-L_L3, y_l4 - y_l3, 0.0),
                   axis="Y", limit_deg=ARM_LIMITS_DEG["ArmJ4"])

    # Print arm summary
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
    print(f"  Link Y lanes (side-by-side): "
          f"L2={y_l2/IN:+.2f}\"  L3={y_l3/IN:+.2f}\"  "
          f"L4={y_l4/IN:+.2f}\"", flush=True)
    print(f"  Stowed positions (chassis frame X):", flush=True)
    j3x = ARM_MOUNT_X + L_L2
    j4x = ARM_MOUNT_X + L_L2 - L_L3
    tipx = j4x + L_L4
    print(f"    J1={ARM_MOUNT_X/IN:+.2f}\"  J2={ARM_MOUNT_X/IN:+.2f}\"  "
          f"J3={j3x/IN:+.2f}\"  J4={j4x/IN:+.2f}\"  "
          f"tip={tipx/IN:+.2f}\"", flush=True)
    print(f"{'='*50}\n", flush=True)

    return ARM_JOINT_NAMES


def arm_tip_world(stage):
    """Arm tip world position for diagnostics (gripper at L4 tip)."""
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


def compute_tipping_margins(stage, chassis_path, arm_joint_names_local):
    """Compute tipping stability: arm CG vs wheel support polygon.

    Returns dict with:
      arm_cg_world:  [x,y,z] of lumped arm CG in world
      wheel_xy:      list of 4 wheel ground-contact [x,y] in world
      margin_front, margin_back, margin_left, margin_right:
          signed distance from arm CG projection to support edge (m).
          Positive = stable, negative = tipping.
      tipping_moment_Nm:  arm_mass * g * max_overhang (worst-case edge)
    """
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

    # Wheel ground contact points (world frame) — use chassis transform
    cprim = stage.GetPrimAtPath(chassis_path)
    cmat = UsdGeom.Xformable(cprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    wheel_world = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        wz = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
        wp = cmat.Transform(Gf.Vec3d(wx, wy, wz))
        wheel_world.append(np.array([wp[0], wp[1]]))

    # Support polygon: axis-aligned bounding box of wheel XY projections
    wxs = [w[0] for w in wheel_world]
    wys = [w[1] for w in wheel_world]
    x_min, x_max = min(wxs), max(wxs)
    y_min, y_max = min(wys), max(wys)

    # Margins: positive = CG inside polygon, negative = outside (tipping)
    margin_front = x_max - cg[0]   # front edge
    margin_back  = cg[0] - x_min   # back edge
    margin_left  = y_max - cg[1]   # left edge
    margin_right = cg[1] - y_min   # right edge

    worst_overhang = -min(margin_front, margin_back, margin_left, margin_right)
    if worst_overhang < 0:
        worst_overhang = 0.0  # all margins positive = no overhang
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


def log_comprehensive(frame, hz, art, stage, chassis_path, arm_joint_names_local,
                      arm_dof_indices, contact_sensor, ee_path):
    """Print comprehensive arm status: torques, tipping, EE contact."""
    s = get_state(art, stage, chassis_path)
    if not s:
        return
    p = s["pos"]
    je = s["je"].flatten() if s["je"] is not None else None
    jp = s["jp"].flatten() if s["jp"] is not None else None
    t_sec = frame / hz

    print(f"\n{'─'*60}", flush=True)
    print(f"[{t_sec:.1f}s] frame={frame}  chassis=({p[0]/IN:.1f}, "
          f"{p[1]/IN:.1f}, {p[2]/IN:.1f})\"", flush=True)

    # Joint torques
    if je is not None and jp is not None and arm_dof_indices:
        print(f"  {'Joint':<8} {'Angle':>8} {'Torque':>10} {'|Torque|':>10}", flush=True)
        for i, idx in enumerate(arm_dof_indices):
            ang = math.degrees(jp[idx])
            tau = je[idx]
            print(f"  {arm_joint_names_local[i]:<8} {ang:>+7.1f}° {tau:>+9.3f} Nm "
                  f"{abs(tau):>9.3f} Nm", flush=True)

    # Arm tip position
    tip = arm_tip_world(stage)
    if tip:
        print(f"  EE tip: ({tip[0]/IN:+.2f}, {tip[1]/IN:+.2f}, {tip[2]/IN:+.2f})\"",
              flush=True)

    # Tipping analysis
    tip_data = compute_tipping_margins(stage, chassis_path, arm_joint_names_local)
    cg = tip_data["arm_cg"]
    status = "STABLE" if tip_data["stable"] else "TIPPING"
    print(f"  Arm CG: ({cg[0]/IN:+.2f}, {cg[1]/IN:+.2f}, {cg[2]/IN:+.2f})\"  "
          f"mass={tip_data['arm_mass']:.2f}kg", flush=True)
    print(f"  Margins: F={tip_data['margin_front']/IN:+.2f}\"  "
          f"B={tip_data['margin_back']/IN:+.2f}\"  "
          f"L={tip_data['margin_left']/IN:+.2f}\"  "
          f"R={tip_data['margin_right']/IN:+.2f}\"  "
          f"[{status}]", flush=True)
    if tip_data["tipping_moment_Nm"] > 0:
        print(f"  Tipping moment: {tip_data['tipping_moment_Nm']:.2f} Nm", flush=True)

    # End-effector contact
    contacts = read_ee_contacts(contact_sensor, ee_path)
    if contacts:
        total_f = sum(c["force_mag"] for c in contacts)
        bodies = set(c["other"] for c in contacts)
        print(f"  EE CONTACT: {len(contacts)} points, "
              f"total force={total_f:.2f} N, bodies={bodies}", flush=True)
    else:
        print(f"  EE contact: none", flush=True)
    print(f"{'─'*60}", flush=True)


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
    # Self-collisions ON: non-adjacent bodies in the articulation collide
    # (e.g. arm links vs chassis, J2 vs J4). PhysX automatically filters
    # parent-child pairs (bodies connected by a joint) so adjacent links
    # won't explode at hinges.
    physx_art.CreateEnabledSelfCollisionsAttr(True)

    build_chassis(stage, chassis_path)

    # Wheel material
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

    # Build arm
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
        # Straddle the step edge at Y=0. Outer floor (higher) is Y<0, inner is Y>0.
        # Yaw 90 deg so robot front (+X) faces +Y (toward lower inner floor).
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
    n_bodies = 1 + 4 + 4*NUM_ROLLERS + 4  # chassis + hubs + rollers + arm
    n_joints = 4 + 4*NUM_ROLLERS + 4      # drive + roller + arm
    print(f"Total bodies: {n_bodies}  Total joints: {n_joints}", flush=True)

    return robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names


# ============================================================================
# X-drive IK (from xdrive_realwheel.py)
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
# Keyboard (drive + arm control)
# ============================================================================

# Arm speed: degrees per second while key is held
ARM_SPEED_DEG_S = 45.0

class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.reset = self.pstate = self.stop = False
        self.print_torque = False
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
            elif k == K.T: self.print_torque = True
        elif ev.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys.discard(k)
        return True

    def cmd(self):
        K = carb.input.KeyboardInput
        vx = self.speed * (int(K.UP in self._keys) - int(K.DOWN in self._keys))
        vy = self.speed * (int(K.LEFT in self._keys) - int(K.RIGHT in self._keys))
        w = self.rspeed * (int(K.Q in self._keys) - int(K.E in self._keys))
        return vx, vy, w

    def arm_deltas(self, dt):
        """Return list of (joint_index, delta_deg) for held arm keys."""
        K = carb.input.KeyboardInput
        step = ARM_SPEED_DEG_S * dt
        deltas = []
        # Adjacent pairs: 1/2=J1, 3/4=J2, 5/6=J3, 7/8=J4
        if K.KEY_1 in self._keys: deltas.append((0, +step))
        if K.KEY_2 in self._keys: deltas.append((0, -step))
        if K.KEY_3 in self._keys: deltas.append((1, +step))
        if K.KEY_4 in self._keys: deltas.append((1, -step))
        if K.KEY_5 in self._keys: deltas.append((2, +step))
        if K.KEY_6 in self._keys: deltas.append((2, -step))
        if K.KEY_7 in self._keys: deltas.append((3, +step))
        if K.KEY_8 in self._keys: deltas.append((3, -step))
        return deltas


# ============================================================================
# MAIN
# ============================================================================
print("=" * 60, flush=True)
_load_str = " LOADED (gripper+payload)" if ARM_LOADED else ""
print(f"FORTIS X-Drive + 4-DOF Parallel Link Arm "
      f"({ARM_REACH_IN:.0f}\" reach){_load_str}", flush=True)
print("=" * 60, flush=True)

src_parts, src_center = read_source_wheel()

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10):
    app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_physics_scene(stage)

step_height = 0.0  # set if --step
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
    # Try fallback
    arm_dof_indices = []
    for i, dn in enumerate(dof_names):
        if "Arm" in dn:
            arm_dof_indices.append(i)
    if len(arm_dof_indices) != 4:
        print(f"FATAL: Cannot map arm DOFs, found {len(arm_dof_indices)}", flush=True)

print(f"Arm DOF indices: {arm_dof_indices}", flush=True)
for i, idx in enumerate(arm_dof_indices):
    print(f"  {arm_joint_names[i]}: DOF[{idx}] = \"{dof_names[idx]}\"", flush=True)

# Arm joint target angles (degrees, start at 0 = stowed)
arm_targets_deg = [0.0] * 4

# Only set velocity mode on non-arm DOFs (drive + rollers).
# Arm DOFs stay in position mode from DriveAPI stiffness.
non_arm_dof_indices = np.array([i for i in range(ndof) if i not in arm_dof_indices])
art.switch_control_mode("velocity", joint_indices=non_arm_dof_indices)

# Settle
print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    world.step(render=not headless)
    if i % (PHYSICS_HZ // 2) == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"  [{i/PHYSICS_HZ:.1f}s] Z={p[2]:.4f}m ({p[2]/IN:.2f}\")", flush=True)

# Print initial comprehensive report (stowed)
print("\n=== INITIAL STATE (stowed) ===", flush=True)
log_comprehensive(0, PHYSICS_HZ, art, stage, chassis_path,
                  arm_joint_names, arm_dof_indices, contact_sensor, ee_path)

if headless:
    # Headless: just report stowed state and exit
    print("\nHeadless mode: arm stowed, reporting state.", flush=True)
    tip = arm_tip_world(stage)
    if tip:
        print(f"Arm tip: ({tip[0]/IN:.2f}, {tip[1]/IN:.2f}, {tip[2]/IN:.2f})\"", flush=True)
    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# GUI mode
kb = KB()
if args.reactor:
    spawn_pos = [0.0, -1.75, spawn_z]
    spawn_yaw_reset = math.degrees(math.atan2(0 - (-1.75), 0 - 0.0))
elif args.step:
    spawn_pos = [0.0, 0.0, spawn_z]
    spawn_yaw_reset = 90.0
else:
    spawn_pos = [0, 0, spawn_z]
    spawn_yaw_reset = 0.0

print("\nControls:", flush=True)
print("  DRIVE:  Arrows = translate  |  Q/E = rotate", flush=True)
print("  ARM:  1/2=J1  3/4=J2  5/6=J3  7/8=J4  (odd=+, even=-)", flush=True)
print("  T/P = full status report  |  R = reset  |  Space = stop", flush=True)

frame = 0
while app.is_running():
    vx, vy, w = kb.cmd()

    if kb.stop:
        kb.stop = False
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
        vx = vy = w = 0.0

    elif kb.reset:
        kb.reset = False
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5): app.update()
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos))
        if spawn_yaw_reset != 0.0:
            rxf.AddRotateZOp().Set(spawn_yaw_reset)
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(10): world.step(render=True)
        non_arm_dof_indices = np.array([i for i in range(ndof) if i not in arm_dof_indices])
        art.switch_control_mode("velocity", joint_indices=non_arm_dof_indices)
        arm_targets_deg = [0.0] * 4
        print("RESET", flush=True)
        continue

    # Arm joint control (continuous while held)
    dt = 1.0 / PHYSICS_HZ
    for ji, delta in kb.arm_deltas(dt):
        if arm_dof_indices:
            jname = arm_joint_names[ji]
            lo, hi = ARM_LIMITS_DEG[jname]
            arm_targets_deg[ji] = max(lo, min(hi, arm_targets_deg[ji] + delta))

    # Set arm joint position targets via DriveAPI
    # We need to set the target position on each arm joint prim
    for i, ajn in enumerate(arm_joint_names):
        jp = f"/World/Robot/{ajn}"
        jprim = stage.GetPrimAtPath(jp)
        if jprim.IsValid():
            drive = UsdPhysics.DriveAPI(jprim, "angular")
            drive.GetTargetPositionAttr().Set(float(arm_targets_deg[i]))

    # Drive wheels
    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    for ii, di in enumerate(drive_dof_indices):
        va[di] = tv[ii]
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))
    world.step(render=True)
    frame += 1

    # Full status on T or P
    if kb.print_torque or kb.pstate:
        kb.print_torque = False
        kb.pstate = False
        log_comprehensive(frame, PHYSICS_HZ, art, stage, chassis_path,
                          arm_joint_names, arm_dof_indices,
                          contact_sensor, ee_path)

    # Periodic comprehensive log (every 1 second)
    if frame % PHYSICS_HZ == 0:
        log_comprehensive(frame, PHYSICS_HZ, art, stage, chassis_path,
                          arm_joint_names, arm_dof_indices,
                          contact_sensor, ee_path)

app.close()
