"""
FORTIS arm Monte Carlo torque + tipping sweep v4 -- headless Isaac Sim.

Differences from v3 (continuous_sweep_v3):
  - **Monte Carlo sampling** of 5000 poses uniformly drawn over the joint
    space, instead of a structured grid. Reproducible via --seed.
  - **J1 is sampled too** (not fixed at 0), because stability is J1-dependent.
  - **Chassis mass override** = 3.629 kg so total robot = 14.144 kg
    (chassis 3.629 + arm 6.515 + wheels 4.0).
  - **Physics tipping detection**: capture chassis (roll, pitch, Z) and
    per-wheel hub Z at sweep start (arm in launch pose), then per pose
    record the after-settle deltas. Post-filter flags physics_tipped if
    deltas exceed thresholds.
  - Collisions: arm-self and arm-chassis already disabled in v3 build path
    (no collision shapes on arm bodies, articulation self-collisions off).
    Wheel-floor stays on so chassis can fall over in physics if it tips.

Pipeline:
  Stage 1 (this script): MC sweep -> raw CSV with every pose, including
    chassis state and per-wheel Z deltas for physics tipping check.
  Stage 2 (arm_mc_filter_v4.py): apply geometric + reactor-envelope
    filters and tipping (analytical at sampled J1 + physics from CSV).
    Does NOT drop unstable poses.
  Stage 3 (arm_mc_plot_v4.py): histogram, torque table, poloidal PNG.

Usage:
  IsaacSim\\python.bat sim/isaac/xdrive/tools/arm_mc_sweep_v4.py --step
  IsaacSim\\python.bat sim/isaac/xdrive/tools/arm_mc_sweep_v4.py --step --n 1000 --seed 7
"""
import os, sys, math, argparse, time, csv, json, datetime
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser(description="FORTIS arm MC sweep v4 -- headless")
parser.add_argument("--step", action="store_true",
                    help="Use the 4.5\" step environment (recommended)")
parser.add_argument("--reactor", action="store_true",
                    help="Use the full reactor USD (slower)")
parser.add_argument("--hz", type=int, default=360)
parser.add_argument("--n", type=int, default=5000,
                    help="Number of MC samples (default 5000)")
parser.add_argument("--seed", type=int, default=42,
                    help="RNG seed for reproducibility")
parser.add_argument("--chassis-mass-kg", type=float, default=3.629,
                    help="Chassis mass override (kg). Default 3.629 = 14.144 kg "
                         "total budget minus arm (6.515) and wheels (4.0).")
parser.add_argument("--resume", action="store_true", help="Resume from existing CSV")
parser.add_argument("--test", action="store_true", help="20-pose test run")
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
# Constants (matched to v3 build)
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
CHASSIS_L = 13.082 * IN
CHASSIS_W = 8.54 * IN
CHASSIS_H = 6.0 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
MOTOR_MOUNT_LEN = 1.272 * IN
BELLY_HEIGHT = 2.0 * IN
ARCH_FLAT_WIDTH = 2.059 * IN
FRICTION_MU = 0.5

# Wheels
TARGET_DIA_MM = 203.0
TARGET_WIDTH_MM = 51.8
WHEEL_RADIUS = TARGET_DIA_MM / 2.0 * MM
WHEEL_WIDTH = TARGET_WIDTH_MM * MM
SRC_DIA_MM = 82.5
SRC_WIDTH_MM = 44.87
SCALE_XZ = TARGET_DIA_MM / SRC_DIA_MM
SCALE_Y = TARGET_WIDTH_MM / SRC_WIDTH_MM
SCALE_UNIFORM = SCALE_XZ

# *** v4 mass override ***
CHASSIS_MASS = float(args.chassis_mass_kg)
WHEEL_MASS = 1.0
ROLLER_MASS_FRAC = 0.3
NUM_ROLLERS = 10
HUB_MASS = WHEEL_MASS * (1.0 - ROLLER_MASS_FRAC)
ROLLER_MASS = WHEEL_MASS * ROLLER_MASS_FRAC / NUM_ROLLERS

PHYSICS_HZ = max(360, min(480, args.hz))
DRIVE_DAMPING = 100.0
DRIVE_MAX_FORCE = 200.0

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

# Arm constants (V3 build, unchanged)
J1_STACK_H = 1.5 * IN
L_L2_IN = 14.168
L_L3_IN = 11.222
L_L4_IN = 4.970
L_L2 = L_L2_IN * IN
L_L3 = L_L3_IN * IN
L_L4 = L_L4_IN * IN
ARM_REACH_IN = L_L2_IN + L_L3_IN + L_L4_IN

M_J1 = 0.580
M_J2 = 2.665
M_J3 = 0.939
M_J4 = 0.5486
M_JOINT = M_J1

CF_DENSITY_LB_PER_IN = 1.03 / 72.0
CF_DENSITY_KG_PER_M = CF_DENSITY_LB_PER_IN * 0.453592 / IN
M_L2 = CF_DENSITY_KG_PER_M * L_L2
M_L3 = CF_DENSITY_KG_PER_M * L_L3
M_L4 = CF_DENSITY_KG_PER_M * L_L4

M_GRIPPER_BARE = 0.133
M_PAYLOAD = 1.361
M_GRIPPER = M_GRIPPER_BARE + M_PAYLOAD
ARM_LOADED = True

M_CAMERA = 0.091
CAM_X_ON_L4 = 1.5 * IN

CF_TUBE_OD = 1.128 * IN
CF_TUBE_Y = CF_TUBE_OD
CF_TUBE_Z = CF_TUBE_OD

ARM_MOUNT_X = -CHASSIS_L / 2.0 + 3.0 * IN
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0

LINK_Y_SPACING = CF_TUBE_Y + 0.005
LINK_Y = [(-1.0) * LINK_Y_SPACING,
          ( 0.0) * LINK_Y_SPACING,
          (+1.0) * LINK_Y_SPACING]

ARM_STIFFNESS = 400.0
ARM_DAMPING   = 60.0
ARM_MAX_FORCE = 1500.0

# v4: J1 is now sampled (full +/-180 range)
ARM_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),
    "ArmJ2": (-180.0, 180.0),
    "ArmJ3": (-180.0, 180.0),
    "ArmJ4": (-101.0, 101.0),
}
ARM_JOINT_NAMES = ["ArmJ1", "ArmJ2", "ArmJ3", "ArmJ4"]

# MC sampling ranges
J1_MIN, J1_MAX = -180.0, 180.0
J2_MIN, J2_MAX = -180.0, 180.0
J3_MIN, J3_MAX = -180.0, 180.0
J4_MIN, J4_MAX = -100.0, 100.0

SETTLE_FRAMES = 150
MEASURE_FRAMES = 36
POSITION_WARN_THRESH = 3.0


# ============================================================================
# USD/scene builders (same as v3)
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
# Arched chassis (same as v3)
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
        if ax <= ramp_start: return z_high
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)
        else: return z_low

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


def _apply_arm_body_mass_x(prim, joint_mass, link_mass, link_length, link_dir,
                           extra_mass=0.0, extra_x=0.0):
    total = joint_mass + link_mass + extra_mass
    link_cx = link_dir * link_length / 2.0
    com_x = (joint_mass * 0.0 + link_mass * link_cx + extra_mass * extra_x) / total
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0.0, 0.0))
    Lx = max(float(link_length), 0.01)
    Ly = CF_TUBE_Y; Lz = CF_TUBE_Z
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

    UsdGeom.Xform.Define(stage, j1_path)
    p1 = stage.GetPrimAtPath(j1_path)
    xf1 = UsdGeom.Xformable(p1); xf1.ClearXformOpOrder()
    xf1.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z))
    UsdPhysics.RigidBodyAPI.Apply(p1)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p1).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_j1_mass(p1, mass=M_J1, stack_h=J1_STACK_H)
    _add_arm_link_visual(stage, j1_path + "/vis",
                         center=(0.0, 0.0, J1_STACK_H / 2.0),
                         size=(0.057, 0.057, J1_STACK_H), color=C_J1)

    UsdGeom.Xform.Define(stage, j2_path)
    p2 = stage.GetPrimAtPath(j2_path)
    xf2 = UsdGeom.Xformable(p2); xf2.ClearXformOpOrder()
    xf2.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, y_l2, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p2)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p2).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p2, joint_mass=M_J2, link_mass=M_L2,
                           link_length=L_L2, link_dir=+1)
    _add_arm_link_visual(stage, j2_path + "/vis",
                         center=(+L_L2/2.0, 0.0, 0.0),
                         size=(L_L2, CF_TUBE_Y, CF_TUBE_Z), color=C_L2)
    jsph = UsdGeom.Sphere.Define(stage, j2_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L2)]))

    UsdGeom.Xform.Define(stage, j3_path)
    p3 = stage.GetPrimAtPath(j3_path)
    xf3 = UsdGeom.Xformable(p3); xf3.ClearXformOpOrder()
    xf3.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2, y_l3, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p3)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p3).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_arm_body_mass_x(p3, joint_mass=M_J3, link_mass=M_L3,
                           link_length=L_L3, link_dir=-1)
    _add_arm_link_visual(stage, j3_path + "/vis",
                         center=(-L_L3/2.0, 0.0, 0.0),
                         size=(L_L3, CF_TUBE_Y, CF_TUBE_Z), color=C_L3)
    jsph = UsdGeom.Sphere.Define(stage, j3_path + "/joint_vis")
    jsph.GetRadiusAttr().Set(0.020)
    jsph.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*C_L3)]))

    UsdGeom.Xform.Define(stage, j4_path)
    p4 = stage.GetPrimAtPath(j4_path)
    xf4 = UsdGeom.Xformable(p4); xf4.ClearXformOpOrder()
    xf4.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2 - L_L3, y_l4, z_arm))
    UsdPhysics.RigidBodyAPI.Apply(p4)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p4).CreateMaxDepenetrationVelocityAttr(1.0)
    _m_extra = M_GRIPPER + M_CAMERA
    _x_extra = (M_GRIPPER * L_L4 + M_CAMERA * CAM_X_ON_L4) / _m_extra
    _apply_arm_body_mass_x(p4, joint_mass=M_J4, link_mass=M_L4,
                           link_length=L_L4, link_dir=+1,
                           extra_mass=_m_extra, extra_x=_x_extra)
    _add_arm_link_visual(stage, j4_path + "/vis",
                         center=(+L_L4/2.0, 0.0, 0.0),
                         size=(L_L4, CF_TUBE_Y, CF_TUBE_Z), color=C_L4)
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

    total_arm_mass = (M_J1 + M_J2 + M_J3 + M_J4
                      + M_L2 + M_L3 + M_L4 + M_GRIPPER + M_CAMERA)
    print(f"\nARM v3 build: 30\" CF, LOADED, total={total_arm_mass:.3f}kg", flush=True)
    print(f"  per-joint: J1={M_J1:.3f} J2={M_J2:.3f} J3={M_J3:.3f} J4={M_J4:.3f} kg", flush=True)
    print(f"  links CF square {CF_TUBE_OD/IN:.3f}\": "
          f"L2={M_L2*1000:.1f}g L3={M_L3*1000:.1f}g L4={M_L4*1000:.1f}g", flush=True)
    return ARM_JOINT_NAMES


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
    physx_art.CreateEnabledSelfCollisionsAttr(False)  # arm <-> chassis off

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
    wheel_hub_paths = {}

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
        wheel_hub_paths[wname] = hub_path
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

    spawn_z = BELLY_HEIGHT + CHASSIS_H / 2.0
    p = stage.GetPrimAtPath(robot_path)
    UsdGeom.Xformable(p).ClearXformOpOrder()
    UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    return robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names, wheel_hub_paths


# ============================================================================
# Helpers
# ============================================================================
def arm_tip_world(stage):
    prim = stage.GetPrimAtPath("/World/Robot/ArmJ4wrist")
    if not prim.IsValid(): return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return mat.Transform(Gf.Vec3d(L_L4, 0.0, 0.0))


def get_chassis_state(stage, chassis_path):
    """Return (roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m) for the chassis."""
    try:
        cprim = stage.GetPrimAtPath(chassis_path)
        cmat = UsdGeom.Xformable(cprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        r = cmat.ExtractRotationMatrix()
        roll_deg = math.degrees(math.atan2(float(r[2][1]), float(r[2][2])))
        pitch_deg = math.degrees(math.asin(max(-1, min(1, -float(r[2][0])))))
        yaw_deg = math.degrees(math.atan2(float(r[1][0]), float(r[0][0])))
        t = cmat.ExtractTranslation()
        return roll_deg, pitch_deg, yaw_deg, float(t[0]), float(t[1]), float(t[2])
    except Exception:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


def get_hub_z(stage, hub_path):
    try:
        prim = stage.GetPrimAtPath(hub_path)
        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = mat.ExtractTranslation()
        return float(t[2])
    except Exception:
        return float("nan")


def usd_to_fk_angles(j2_usd, j3_usd, j4_usd):
    """Convert USD joint angles to standard FK convention."""
    j2_fk = j2_usd
    j3_fk = j3_usd + 180.0
    if j3_fk > 180.0: j3_fk -= 360.0
    if j3_fk < -180.0: j3_fk += 360.0
    j4_fk = j4_usd - 180.0
    if j4_fk > 180.0: j4_fk -= 360.0
    if j4_fk < -180.0: j4_fk += 360.0
    return j2_fk, j3_fk, j4_fk


def compute_analytical_torques(j2_usd_deg, j3_usd_deg, j4_usd_deg):
    import arm_ik_v3 as arm_ik
    j2_fk, j3_fk, j4_fk = usd_to_fk_angles(j2_usd_deg, j3_usd_deg, j4_usd_deg)
    params = arm_ik.get_arm_params()
    return arm_ik.gravity_torques(
        math.radians(j2_fk), math.radians(j3_fk), math.radians(j4_fk), params)


# ============================================================================
# Generate MC sample
# ============================================================================
def generate_mc_poses(n, seed):
    rng = np.random.default_rng(seed)
    j1 = rng.uniform(J1_MIN, J1_MAX, size=n)
    j2 = rng.uniform(J2_MIN, J2_MAX, size=n)
    j3 = rng.uniform(J3_MIN, J3_MAX, size=n)
    j4 = rng.uniform(J4_MIN, J4_MAX, size=n)
    return list(zip(j1.tolist(), j2.tolist(), j3.tolist(), j4.tolist()))


# ============================================================================
# Output setup
# ============================================================================
env_label = "reactor" if args.reactor else ("step" if args.step else "flat")
n_samples = 20 if args.test else int(args.n)
config_tag = f"30in_loaded_{env_label}_v4_mc{n_samples}"

out_dir = os.path.join(RESULTS_ROOT, "arm_mc_sweep")
os.makedirs(out_dir, exist_ok=True)
csv_path = os.path.join(out_dir, f"{config_tag}.csv")
json_path = os.path.join(out_dir, f"{config_tag}_summary.json")

CSV_FIELDS = [
    "pose_idx",
    "j1_deg", "j2_deg", "j3_deg", "j4_deg",
    "j2_fk_deg", "j3_fk_deg", "j4_fk_deg",
    "j1_actual_deg", "j2_actual_deg", "j3_actual_deg", "j4_actual_deg",
    "position_error_deg",
    "tau_j1_Nm", "tau_j2_Nm", "tau_j3_Nm", "tau_j4_Nm",
    "tau_j2_analytical", "tau_j3_analytical", "tau_j4_analytical",
    "ee_x_m", "ee_y_m", "ee_z_m",
    "j3_local_x_m", "j3_local_z_m",
    "j4_local_x_m", "j4_local_z_m",
    "ee_local_x_m", "ee_local_z_m",
    "chassis_roll_deg", "chassis_pitch_deg",
    # Physics-tipping inputs
    "chassis_z_m",
    "droll_deg", "dpitch_deg", "dz_chassis_m",
    "max_hub_lift_m",
    "physics_tipped",
    "converged",
]

completed_poses = set()
if args.resume and os.path.exists(csv_path):
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                completed_poses.add(int(row["pose_idx"]))
            except (KeyError, ValueError):
                pass
    print(f"Resume: {len(completed_poses)} poses already completed", flush=True)


# ============================================================================
# Build scene
# ============================================================================
print(f"\n{'='*60}", flush=True)
print(f"FORTIS Arm MC Sweep v4 -- {config_tag}", flush=True)
print(f"{'='*60}", flush=True)
print(f"Chassis mass override: {CHASSIS_MASS:.3f} kg "
      f"(arm 6.515 + wheels 4.0 + chassis = "
      f"{CHASSIS_MASS + 6.515 + 4.0:.3f} kg total = "
      f"{(CHASSIS_MASS + 6.515 + 4.0)/0.45359237:.2f} lb)", flush=True)
print(f"MC samples: n={n_samples}, seed={args.seed}", flush=True)

src_parts, src_center = read_source_wheel()

stage = omni.usd.get_context().get_stage()
build_physics_scene(stage)

if args.reactor:
    print(f"Loading reactor: {REACTOR_USD}", flush=True)
    rp = stage.DefinePrim("/World/Reactor", "Xform")
    rp.GetReferences().AddReference(REACTOR_USD)
    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)
elif args.step:
    build_step_env(stage)
else:
    build_ground(stage)

robot_path, chassis_path, drive_joint_paths, spawn_z, arm_joint_names, wheel_hub_paths = build_robot(
    stage, src_parts, src_center)

print("Arm-self / arm-chassis collisions DISABLED (raw sweep). "
      "Wheel-floor collisions ON for tipping detection.", flush=True)

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
print(f"Articulation: {ndof} DOFs", flush=True)

drive_dof_indices = []
for djp in drive_joint_paths:
    jn = djp.split("/")[-1]
    for i, dn in enumerate(dof_names):
        if jn in dn or dn == jn:
            drive_dof_indices.append(i); break

arm_dof_indices = []
for ajn in arm_joint_names:
    for i, dn in enumerate(dof_names):
        if ajn in dn or dn == ajn:
            arm_dof_indices.append(i); break

if len(arm_dof_indices) != 4:
    arm_dof_indices = [i for i, dn in enumerate(dof_names) if "Arm" in dn]
print(f"Arm DOF indices: {arm_dof_indices}", flush=True)

non_arm = np.array([i for i in range(ndof) if i not in arm_dof_indices])
art.switch_control_mode("velocity", joint_indices=non_arm)
va_lock = np.zeros(ndof)
art.set_joint_velocity_targets(va_lock.reshape(1, -1))

print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    world.step(render=False)

init_jp = art.get_joint_positions().copy()
print("Stowed position captured", flush=True)

# Launch to safe pose: J1=0, J2=90 (arm straight up), J3=J4=0
launch_jp = init_jp.flatten().copy()
launch_arm = [0.0, 90.0, 0.0, 0.0]
for i in range(4):
    launch_jp[arm_dof_indices[i]] = math.radians(launch_arm[i])
art.set_joint_positions(launch_jp.reshape(1, -1))
art.set_joint_velocities(np.zeros_like(launch_jp).reshape(1, -1))
for i in range(4):
    jp_path = f"/World/Robot/{ARM_JOINT_NAMES[i]}"
    jprim = stage.GetPrimAtPath(jp_path)
    if jprim.IsValid():
        UsdPhysics.DriveAPI(jprim, "angular").GetTargetPositionAttr().Set(float(launch_arm[i]))
art.set_joint_position_targets(launch_jp.reshape(1, -1))
art.set_joint_velocity_targets(va_lock.reshape(1, -1))

print(f"Launch: arm to (J1,J2,J3,J4) = (0, 90, 0, 0) deg", flush=True)
for i in range(int(1.5 * PHYSICS_HZ)):
    world.step(render=False)
print("Launch settled", flush=True)
for _ in range(60):
    world.step(render=False)

# Baseline chassis state for physics tipping detection
base_roll, base_pitch, base_yaw, base_x, base_y, base_z = get_chassis_state(stage, chassis_path)
base_hub_z = {wn: get_hub_z(stage, wheel_hub_paths[wn]) for wn in WHEEL_ORDER}
print(f"BASELINE chassis: roll={base_roll:.2f}, pitch={base_pitch:.2f}, "
      f"yaw={base_yaw:.2f} deg, z={base_z*1000:.1f} mm", flush=True)
print(f"BASELINE hub Z (mm): " +
      ", ".join([f"{wn}={base_hub_z[wn]*1000:.1f}" for wn in WHEEL_ORDER]), flush=True)

# Tipping thresholds (configurable in CSV via constants for downstream)
TIP_ROLL_THRESH_DEG = 5.0
TIP_PITCH_THRESH_DEG = 5.0
TIP_Z_DROP_THRESH_M = 0.010   # 1 cm drop -> tipped
TIP_HUB_LIFT_THRESH_M = 0.005 # 5 mm hub lift -> wheel left ground

import arm_ik_v3 as _arm_ik_mod
_arm_params = _arm_ik_mod.get_arm_params()


# ============================================================================
# MC sweep loop
# ============================================================================
poses = generate_mc_poses(n_samples, args.seed)
total_poses = len(poses)

csv_mode = "a" if (args.resume and len(completed_poses) > 0) else "w"
csv_file = open(csv_path, csv_mode, newline="", encoding="utf-8")
writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
if csv_mode == "w":
    writer.writeheader()
    csv_file.flush()

start_time = time.time()
start_utc = datetime.datetime.utcnow().isoformat() + "Z"

n_done = 0
n_skipped = 0
n_converged = 0
n_nan = 0
n_phys_tipped = 0
pose_times = []

print(f"\nStarting MC sweep: {total_poses} poses", flush=True)
print(f"Settle={SETTLE_FRAMES} frames ({SETTLE_FRAMES/PHYSICS_HZ:.2f}s), "
      f"Measure={MEASURE_FRAMES} frames ({MEASURE_FRAMES/PHYSICS_HZ:.2f}s)", flush=True)
print(f"Tipping thresh: |dRoll|>{TIP_ROLL_THRESH_DEG} or |dPitch|>{TIP_PITCH_THRESH_DEG} deg, "
      f"dZ_drop>{TIP_Z_DROP_THRESH_M*1000:.0f} mm, "
      f"hub_lift>{TIP_HUB_LIFT_THRESH_M*1000:.0f} mm", flush=True)
print(f"Output: {csv_path}", flush=True)

PROGRESS_INTERVAL = 60

for pose_idx, (j1_deg, j2_deg, j3_deg, j4_deg) in enumerate(poses):
    if pose_idx in completed_poses:
        n_skipped += 1
        continue

    pose_t0 = time.time()

    target_angles_deg = [j1_deg, j2_deg, j3_deg, j4_deg]
    target_jp = init_jp.flatten().copy()
    for i in range(4):
        target_jp[arm_dof_indices[i]] = math.radians(target_angles_deg[i])

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

    for _ in range(SETTLE_FRAMES):
        world.step(render=False)

    raw_jp = art.get_joint_positions()
    if raw_jp is None or np.any(np.isnan(raw_jp)):
        n_nan += 1
        n_done += 1
        row = {f: "" for f in CSV_FIELDS}
        row["pose_idx"] = pose_idx
        row["j1_deg"] = j1_deg; row["j2_deg"] = j2_deg
        row["j3_deg"] = j3_deg; row["j4_deg"] = j4_deg
        j2f, j3f, j4f = usd_to_fk_angles(j2_deg, j3_deg, j4_deg)
        row["j2_fk_deg"] = f"{j2f:.1f}"; row["j3_fk_deg"] = f"{j3f:.1f}"; row["j4_fk_deg"] = f"{j4f:.1f}"
        row["converged"] = "NaN"
        writer.writerow(row); csv_file.flush()
        pose_times.append(time.time() - pose_t0)
        if n_nan <= 5:
            print(f"  WARNING pose {pose_idx}: NaN", flush=True)
        # Reset to launch
        art.set_joint_positions(launch_jp.reshape(1, -1))
        art.set_joint_velocities(np.zeros_like(launch_jp).reshape(1, -1))
        art.set_joint_position_targets(launch_jp.reshape(1, -1))
        for _ in range(60):
            world.step(render=False)
        continue

    actual_jp = raw_jp.flatten()
    actual_arm_deg = [math.degrees(actual_jp[arm_dof_indices[i]]) for i in range(4)]
    pos_errors = [abs(target_angles_deg[i] - actual_arm_deg[i]) for i in range(4)]
    max_pos_error = max(pos_errors)
    converged = max_pos_error < POSITION_WARN_THRESH

    if not converged and n_done < 20:
        print(f"  WARN pose {pose_idx}: err={max_pos_error:.2f} deg "
              f"target=({j1_deg:.0f},{j2_deg:.0f},{j3_deg:.0f},{j4_deg:.0f}) "
              f"actual=({actual_arm_deg[0]:.1f},{actual_arm_deg[1]:.1f},"
              f"{actual_arm_deg[2]:.1f},{actual_arm_deg[3]:.1f})", flush=True)

    torque_samples = [[] for _ in range(4)]
    for _ in range(MEASURE_FRAMES):
        world.step(render=False)
        try:
            je = art.get_measured_joint_efforts()
            if je is not None:
                je_flat = je.flatten()
                for ai in range(4):
                    val = float(je_flat[arm_dof_indices[ai]])
                    if math.isfinite(val):
                        torque_samples[ai].append(val)
        except Exception:
            pass

    mean_torques = []
    for ai in range(4):
        if torque_samples[ai]:
            mean_torques.append(float(np.mean(torque_samples[ai])))
        else:
            mean_torques.append(float('nan'))

    # End-effector world position
    ee_pos = arm_tip_world(stage)
    ee_x = float(ee_pos[0]) if ee_pos else float('nan')
    ee_y = float(ee_pos[1]) if ee_pos else float('nan')
    ee_z = float(ee_pos[2]) if ee_pos else float('nan')

    # FK locals
    j2f, j3f, j4f = usd_to_fk_angles(j2_deg, j3_deg, j4_deg)
    try:
        fk = _arm_ik_mod.fk_planar(
            math.radians(j2f), math.radians(j3f), math.radians(j4f),
            _arm_params["L2"], _arm_params["L3"], _arm_params["L4"])
        j3_lx, j3_lz = fk["j3"]
        j4_lx, j4_lz = fk["j4"]
        ee_lx, ee_lz = fk["ee"]
    except Exception:
        j3_lx = j3_lz = j4_lx = j4_lz = ee_lx = ee_lz = float('nan')

    # Final chassis + hub state for tipping check
    roll, pitch, yaw, cx, cy, cz = get_chassis_state(stage, chassis_path)
    # Wrap angle deltas to [-180, 180] so Euler boundary (e.g. baseline near
    # +/-180 deg roll on the tilted step) doesn't fake a 360 deg jump.
    def _wrap180(d):
        return ((d + 180.0) % 360.0) - 180.0
    droll = _wrap180(roll - base_roll)
    dpitch = _wrap180(pitch - base_pitch)
    dz = cz - base_z   # negative = chassis dropped

    # Per-wheel hub Z (positive lift = wheel rose above its baseline)
    hub_lifts = []
    for wn in WHEEL_ORDER:
        z = get_hub_z(stage, wheel_hub_paths[wn])
        hub_lifts.append(z - base_hub_z[wn])
    max_hub_lift = max(hub_lifts) if hub_lifts else 0.0

    physics_tipped = (
        abs(droll) > TIP_ROLL_THRESH_DEG
        or abs(dpitch) > TIP_PITCH_THRESH_DEG
        or (-dz) > TIP_Z_DROP_THRESH_M
        or max_hub_lift > TIP_HUB_LIFT_THRESH_M
    )
    if physics_tipped:
        n_phys_tipped += 1

    try:
        ana_torques = compute_analytical_torques(j2_deg, j3_deg, j4_deg)
    except Exception:
        ana_torques = {"tau_j2": float('nan'), "tau_j3": float('nan'), "tau_j4": float('nan')}

    if converged:
        n_converged += 1
    n_done += 1

    row = {
        "pose_idx": pose_idx,
        "j1_deg": f"{j1_deg:.3f}", "j2_deg": f"{j2_deg:.3f}",
        "j3_deg": f"{j3_deg:.3f}", "j4_deg": f"{j4_deg:.3f}",
        "j2_fk_deg": f"{j2f:.3f}", "j3_fk_deg": f"{j3f:.3f}", "j4_fk_deg": f"{j4f:.3f}",
        "j1_actual_deg": f"{actual_arm_deg[0]:.3f}",
        "j2_actual_deg": f"{actual_arm_deg[1]:.3f}",
        "j3_actual_deg": f"{actual_arm_deg[2]:.3f}",
        "j4_actual_deg": f"{actual_arm_deg[3]:.3f}",
        "position_error_deg": f"{max_pos_error:.3f}",
        "tau_j1_Nm": f"{mean_torques[0]:.4f}",
        "tau_j2_Nm": f"{mean_torques[1]:.4f}",
        "tau_j3_Nm": f"{mean_torques[2]:.4f}",
        "tau_j4_Nm": f"{mean_torques[3]:.4f}",
        "tau_j2_analytical": f"{ana_torques['tau_j2']:.4f}",
        "tau_j3_analytical": f"{ana_torques['tau_j3']:.4f}",
        "tau_j4_analytical": f"{ana_torques['tau_j4']:.4f}",
        "ee_x_m": f"{ee_x:.6f}", "ee_y_m": f"{ee_y:.6f}", "ee_z_m": f"{ee_z:.6f}",
        "j3_local_x_m": f"{j3_lx:.6f}", "j3_local_z_m": f"{j3_lz:.6f}",
        "j4_local_x_m": f"{j4_lx:.6f}", "j4_local_z_m": f"{j4_lz:.6f}",
        "ee_local_x_m": f"{ee_lx:.6f}", "ee_local_z_m": f"{ee_lz:.6f}",
        "chassis_roll_deg": f"{roll:.3f}",
        "chassis_pitch_deg": f"{pitch:.3f}",
        "chassis_z_m": f"{cz:.6f}",
        "droll_deg": f"{droll:.3f}",
        "dpitch_deg": f"{dpitch:.3f}",
        "dz_chassis_m": f"{dz:.6f}",
        "max_hub_lift_m": f"{max_hub_lift:.6f}",
        "physics_tipped": "True" if physics_tipped else "False",
        "converged": "True" if converged else "False",
    }
    writer.writerow(row)
    csv_file.flush()

    # If chassis tipped, reset to launch before next pose so subsequent
    # baselines aren't polluted.
    if physics_tipped:
        art.set_joint_positions(launch_jp.reshape(1, -1))
        art.set_joint_velocities(np.zeros_like(launch_jp).reshape(1, -1))
        art.set_joint_position_targets(launch_jp.reshape(1, -1))
        for _ in range(int(0.8 * PHYSICS_HZ)):
            world.step(render=False)

    pose_dt = time.time() - pose_t0
    pose_times.append(pose_dt)

    if n_done <= 10:
        print(f"  [{n_done}/{total_poses}] J1={j1_deg:+.0f} J2={j2_deg:+.0f} "
              f"J3={j3_deg:+.0f} J4={j4_deg:+.0f}  "
              f"tau2={mean_torques[1]:.2f} tau3={mean_torques[2]:.2f} tau4={mean_torques[3]:.2f}  "
              f"err={max_pos_error:.2f} dRP=({droll:+.1f},{dpitch:+.1f}) dZ={dz*1000:+.1f}mm "
              f"tip={physics_tipped}  {pose_dt:.2f}s", flush=True)

    if n_done > 10 and (time.time() - pose_t0 > 0):
        elapsed = time.time() - start_time
        if (n_done % max(1, total_poses // 10) == 0) or (elapsed % PROGRESS_INTERVAL < pose_dt):
            rate = n_done / elapsed if elapsed > 0 else 0
            remaining = total_poses - n_done - n_skipped
            eta = remaining / rate if rate > 0 else 0
            mean_pt = np.mean(pose_times[-50:]) if pose_times else 0
            print(f"  [{n_done}/{total_poses}] conv={n_converged} nan={n_nan} "
                  f"phys_tip={n_phys_tipped} "
                  f"elapsed={elapsed/60:.1f}min ETA={eta/60:.1f}min "
                  f"({mean_pt:.3f}s/pose)", flush=True)

csv_file.close()

elapsed_total = time.time() - start_time
print(f"\n{'='*60}", flush=True)
print(f"MC SWEEP COMPLETE -- {config_tag}", flush=True)
print(f"{'='*60}", flush=True)
print(f"  Total samples: {total_poses}", flush=True)
print(f"  Skipped (resume): {n_skipped}", flush=True)
print(f"  Recorded: {n_done}", flush=True)
print(f"    Converged: {n_converged} ({(n_converged/max(n_done,1))*100:.1f}%)", flush=True)
print(f"    NaN: {n_nan}", flush=True)
print(f"    Physics-tipped: {n_phys_tipped} ({(n_phys_tipped/max(n_done,1))*100:.1f}%)", flush=True)
print(f"  Elapsed: {elapsed_total/60:.1f} min", flush=True)

summary = {
    "config": config_tag,
    "method": "monte_carlo_uniform_4dof",
    "n_samples": n_samples,
    "seed": int(args.seed),
    "j1_range_deg": [J1_MIN, J1_MAX],
    "j2_range_deg": [J2_MIN, J2_MAX],
    "j3_range_deg": [J3_MIN, J3_MAX],
    "j4_range_deg": [J4_MIN, J4_MAX],
    "chassis_mass_kg": CHASSIS_MASS,
    "wheels_total_kg": 4.0 * WHEEL_MASS,
    "arm_total_kg_approx": 6.515,
    "robot_total_kg": CHASSIS_MASS + 4.0 * WHEEL_MASS + 6.515,
    "tip_roll_thresh_deg": TIP_ROLL_THRESH_DEG,
    "tip_pitch_thresh_deg": TIP_PITCH_THRESH_DEG,
    "tip_z_drop_thresh_m": TIP_Z_DROP_THRESH_M,
    "tip_hub_lift_thresh_m": TIP_HUB_LIFT_THRESH_M,
    "baseline_roll_deg": base_roll,
    "baseline_pitch_deg": base_pitch,
    "baseline_z_m": base_z,
    "n_done": n_done,
    "n_converged": n_converged,
    "n_nan": n_nan,
    "n_physics_tipped": n_phys_tipped,
    "elapsed_s": elapsed_total,
    "started_utc": start_utc,
    "csv_path": csv_path,
    "test_mode": args.test,
    "environment": env_label,
    "arm_reach_in": ARM_REACH_IN,
    "loaded": ARM_LOADED,
}
with open(json_path, "w") as jf:
    json.dump(summary, jf, indent=2)
print(f"\nResults: {csv_path}", flush=True)
print(f"Summary: {json_path}", flush=True)

app.close()
