"""
FORTIS X-drive with REAL omni wheel meshes from Kaya omniwheels.usd.

Each roller is a separate rigid body with a free-spinning revolute joint
to the hub -- no sphere approximation. Convex hull collision on each
roller barrel gives physically accurate anisotropic friction.

GPU physics at 360Hz.

Usage: IsaacSim\\python.bat xdrive_realwheel.py --gui
       IsaacSim\\python.bat xdrive_realwheel.py --gui --belly 2.5
"""
import os, sys, math, argparse
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--belly", type=float, default=2.0,
                    help="Belly height above ground in inches (default 2.0)")
parser.add_argument("--hz", type=int, default=360,
                    help="Physics Hz (360-480, default 360)")
parser.add_argument("--reactor", action="store_true",
                    help="Load diiid_reactor.usd and spawn on outer floor")
parser.add_argument("--tunnel", action="store_true",
                    help="Spawn in the access tunnel (implies --reactor)")
parser.add_argument("--contact", action="store_true",
                    help="Enable chassis-reactor contact reporting (log scrape points)")
parser.add_argument("--drive-speed", type=float, default=0.2,
                    help="Forward drive speed for contact test in m/s (default 0.2)")
parser.add_argument("--drive-time", type=float, default=15.0,
                    help="How long to drive forward in seconds (default 15)")
parser.add_argument("--orbit-speed", type=float, default=None,
                    help="Headless orbit at this speed (m/s). Implies --reactor.")
parser.add_argument("--orbit-orbits", type=float, default=2.0,
                    help="Number of orbits to run (default 2)")
parser.add_argument("--orbit-radius", type=float, default=1.59,
                    help="Orbit radius in meters (default 1.59 = outer floor)")
parser.add_argument("--orbit-csv", type=str, default=None,
                    help="Output CSV path for orbit test (default: auto)")
args, _ = parser.parse_known_args()
if args.orbit_speed is not None:
    args.reactor = True
    if not args.gui:
        args.headless = True
if args.tunnel:
    args.reactor = True  # tunnel implies reactor
if args.contact:
    args.tunnel = True
    args.reactor = True
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
# Scripts live under xdrive/{canonical,deprecated,tools}/; assets, lib, and
# results all sit at the xdrive root. Resolve them relative to this file.
XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ASSETS_DIR  = os.path.join(XDRIVE_ROOT, "assets")
RESULTS_ROOT = os.path.join(XDRIVE_ROOT, "results")
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
# --- end bootstrap ---
OMNIWHEEL_USD = os.path.join(ASSETS_DIR, "omniwheels.usd")

# Chassis skeleton (octagonal with 3" chamfer faces)
CHASSIS_L = 13.082 * IN     # bounding box front-to-back (X)
CHASSIS_W = 8.54 * IN       # bounding box side-to-side (Y)
CHASSIS_H = 6.0 * IN        # height (Z)
CHAMFER_FACE = 3.0 * IN     # 45-deg chamfer face for wheel mounting
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
# Straight edges: 8.757" (length sides) x 4.257" (width faces)
# Total footprint with wheels: 19.022" L x 14.5" W

# Motor mount flats at front/rear (where gearbox sits, along X axis)
MOTOR_MOUNT_LEN = 1.272 * IN # flat area at each end for gearbox + motor

# Belly arch (recessed center section for step clearance)
BELLY_HEIGHT = args.belly * IN  # how high the belly center is raised above chassis bottom
# The arch is between the motor mount flats. Ramps connect the flat to the raised center.
ARCH_FLAT_WIDTH = 2.059 * IN # flat section at the raised belly center

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

# Mass — 40 lb total robot (chassis+arm+wheels): 14.144 + 4*1.0 = 18.144 kg
CHASSIS_MASS = 14.144       # kg (chassis body, arm lumped on top)
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

# Chassis geometry
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

# Octagon vertices (top view, wheels mount at chamfer face midpoints)
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

        # Roller collision: 5-sphere chain along the tangent direction.
        # GPU PhysX limits convex hulls to 64 verts -- mesh collision fails.
        # A sphere CHAIN gives a near-continuous contact arc (each sphere
        # overlaps neighbours along the chain) and lives on the same
        # revolute-jointed roller body so omni anisotropic rolling is
        # preserved.
        roller_center_dist = float(np.sqrt(scaled_center[0]**2 + scaled_center[2]**2))
        # Tangent direction in wheel/roller-body frame.
        theta = math.atan2(scaled_center[2], scaled_center[0])
        tang_x = -math.sin(theta)
        tang_z =  math.cos(theta)
        chord = 2.0 * roller_center_dist * math.sin(math.pi / NUM_ROLLERS)

        N_SUB = 5
        sub_r = max(0.005, chord * 0.25)
        spacing = max(0.0, (chord - 2.0 * sub_r) / float(N_SUB - 1))
        sub_r = min(sub_r, float(WHEEL_RADIUS - roller_center_dist) + 0.002)

        for si in range(N_SUB):
            off = (si - (N_SUB - 1) / 2.0) * spacing
            smp = rp + f"/collider_{si}"
            sphere = UsdGeom.Sphere.Define(stage, smp)
            sphere.GetRadiusAttr().Set(float(sub_r))
            sxf = UsdGeom.Xformable(sphere.GetPrim())
            sxf.ClearXformOpOrder()
            if abs(off) > 1e-9:
                sxf.AddTranslateOp().Set(Gf.Vec3d(
                    float(off * tang_x), 0.0, float(off * tang_z)))
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
                wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            UsdGeom.Imageable(sphere.GetPrim()).CreatePurposeAttr("guide")
            pcoll = PhysxSchema.PhysxCollisionAPI.Apply(sphere.GetPrim())
            pcoll.CreateContactOffsetAttr(0.001)
            pcoll.CreateRestOffsetAttr(0.0005)
        if ri == 0:
            outer_off = (N_SUB - 1) / 2.0 * spacing
            print(f"  Roller chain: {N_SUB} spheres r={sub_r*1000:.1f}mm "
                  f"spacing={spacing*1000:.1f}mm "
                  f"(outer @ {outer_off*1000:.1f}mm, chord={chord*1000:.1f}mm)",
                  flush=True)

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
    PhysxSchema.PhysxArticulationAPI.Apply(cp)

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

    # Wheel offset: must clear roller orbit radius from chassis collision.
    # Roller centers orbit at ~76mm from wheel hub in the rotation plane;
    # the inner-side rollers will be inside the chassis bounding box if the
    # offset is too small, causing PhysX to violently push them out. 40mm
    # clearance was the empirically-stable value with convexDecomposition
    # collision on the chassis mesh.
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

        # Visual bracket: thin strut from chamfer face to wheel hub
        bracket_path = chassis_path + f"/bracket_{wname}"
        bracket = UsdGeom.Cube.Define(stage, bracket_path)
        bracket.GetSizeAttr().Set(1.0)
        bxf = UsdGeom.Xformable(bracket.GetPrim())
        bxf.ClearXformOpOrder()
        bxf.AddTranslateOp().Set(Gf.Vec3d(
            (cx + wx) / 2.0, (cy + wy) / 2.0, float(wheel_z)))
        bxf.AddRotateZOp().Set(float(math.degrees(math.atan2(ny, nx))))
        bxf.AddScaleOp().Set(Gf.Vec3d(
            float(wheel_offset), 0.03, 0.03))
        bracket.GetDisplayColorAttr().Set(
            Vt.Vec3fArray([Gf.Vec3f(0.35, 0.35, 0.40)]))

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

    # Spawn position
    if args.reactor:
        global SPAWN_X, SPAWN_Y, spawn_yaw
        import sim_config as cfg

        if args.tunnel:
            # Tunnel spawn — from xdrive_r0_entry_v2.py
            # Robot in the access tunnel, facing toward reactor center (+Y direction)
            SPAWN_X = 0.0
            SPAWN_Y = -140.0 * IN            # -3.556m, in the tunnel
            R0_PORT_Z = -1.7 * IN            # -0.043m
            spawn_z = R0_PORT_Z + WHEEL_RADIUS + CHASSIS_H / 2.0 + 0.05
            spawn_yaw = 90.0                  # face +Y (toward reactor)
        else:
            # Outer floor near step — from xdrive_reactor.py
            SPAWN_X = 0.0
            SPAWN_Y = -1.59                   # 62.7" from center
            SPAWN_FLOOR_Z = cfg.Z_OUTER_IN * IN  # -49.3"
            spawn_z = SPAWN_FLOOR_Z + BELLY_HEIGHT + CHASSIS_H / 2.0 + 0.02
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
    else:
        spawn_z = BELLY_HEIGHT + CHASSIS_H / 2.0 + 0.01
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    print(f"\nChassis: {CHASSIS_L/IN:.1f}x{CHASSIS_W/IN:.1f}x{CHASSIS_H/IN:.1f}\" rect, "
          f"{CHASSIS_MASS}kg", flush=True)
    print(f"Belly height: {BELLY_HEIGHT/IN:.2f}\" ({BELLY_HEIGHT*100:.1f}cm)", flush=True)
    print(f"Wheel: {TARGET_DIA_MM:.0f}mm dia, {TARGET_WIDTH_MM:.0f}mm wide, "
          f"{NUM_ROLLERS} real rollers/wheel", flush=True)
    print(f"Wheel Z in chassis frame: {wheel_z/IN:.2f}\"", flush=True)
    print(f"Spawn Z: {spawn_z:.4f}m ({spawn_z/IN:.2f}\")", flush=True)
    print(f"Total bodies: 1 chassis + {4} hubs + {4*NUM_ROLLERS} rollers = "
          f"{1 + 4 + 4*NUM_ROLLERS}", flush=True)
    print(f"Total joints: {4} drive + {4*NUM_ROLLERS} roller = "
          f"{4 + 4*NUM_ROLLERS}", flush=True)

    return robot_path, chassis_path, drive_joint_paths, spawn_z


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
        self.orbit_radius = 1.59 if args.reactor else 1.0  # 62.7" = 1.59m from xdrive_reactor.py
        self.reset = self.pstate = self.stop = False
        self.orbit = 0
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
            elif k == K.EQUAL:
                self.speed = min(self.speed + 0.05, 1.0)
                print(f"Speed: {self.speed:.2f} m/s")
            elif k == K.MINUS:
                self.speed = max(self.speed - 0.05, 0.05)
                print(f"Speed: {self.speed:.2f} m/s")
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


# ============================================================================
# MAIN
# ============================================================================
print("=" * 60, flush=True)
print("FORTIS X-Drive (Real Omni Wheel Meshes, GPU Physics)", flush=True)
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

if args.reactor:
    # Load reactor environment
    print(f"Loading reactor: {REACTOR_USD}", flush=True)
    rp = stage.DefinePrim("/World/Reactor", "Xform")
    rp.GetReferences().AddReference(REACTOR_USD)
    # Still add a distant light
    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)
else:
    build_ground(stage)

robot_path, chassis_path, drive_joint_paths, spawn_z = build_robot(
    stage, src_parts, src_center)

for _ in range(20):
    app.update()

world = World(stage_units_in_meters=1.0)

# --- Contact reporting setup (before timeline play) ---
contact_sensor = None
contact_log = []
if args.contact:
    from isaacsim.sensors.physics import _sensor as _contact_sensor
    chassis_prim = stage.GetPrimAtPath(chassis_path)
    if not chassis_prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
        cr_api = PhysxSchema.PhysxContactReportAPI.Apply(chassis_prim)
    else:
        cr_api = PhysxSchema.PhysxContactReportAPI(chassis_prim)
    cr_api.CreateThresholdAttr().Set(0)
    # Prevent chassis from sleeping (sleep suppresses contact events)
    prb = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis_prim)
    prb.CreateSleepThresholdAttr(0.0)
    contact_sensor = _contact_sensor.acquire_contact_sensor_interface()
    print("Contact reporting ENABLED on chassis", flush=True)

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

art.switch_control_mode("velocity", joint_indices=np.arange(ndof))

# Set roller joints to near-zero damping (they should spin freely)
# The drive joints already have damping from USD

# Settle — print Z every 0.5s to track drop
print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    world.step(render=not headless)
    if i % (PHYSICS_HZ // 2) == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"  [{i/PHYSICS_HZ:.1f}s] Z={p[2]:.4f}m ({p[2]/IN:.2f}\") "
                  f"pos=({p[0]/IN:.1f},{p[1]/IN:.1f},{p[2]/IN:.1f})\"", flush=True)

print_state(art, stage, chassis_path, 0)

# ============================================================================
# Contact test mode: drive forward through R0, log all chassis-reactor contacts
# ============================================================================
if args.contact:
    import json
    render = not headless
    drive_speed = args.drive_speed
    drive_time = args.drive_time
    total_frames = int(drive_time * PHYSICS_HZ)

    # In tunnel mode, robot faces +Y (yaw=90). "Forward" in chassis frame = +X local.
    # The IK takes vx=forward, vy=strafe in chassis frame.
    print(f"\n{'='*60}", flush=True)
    print(f"CONTACT TEST: drive forward at {drive_speed} m/s for {drive_time}s", flush=True)
    print(f"  Total frames: {total_frames} at {PHYSICS_HZ}Hz", flush=True)
    print(f"{'='*60}\n", flush=True)

    tv = xdrive_ik(drive_speed, 0, 0)
    va = np.zeros(ndof)
    for ii, di in enumerate(drive_dof_indices):
        va[di] = tv[ii]

    contact_count = 0
    for frame in range(total_frames):
        if art.is_physics_handle_valid():
            art.set_joint_velocity_targets(va.reshape(1, -1))
        world.step(render=render)

        # Poll contacts
        contacts = read_chassis_contacts(contact_sensor, chassis_path)
        if contacts:
            s = get_state(art, stage, chassis_path)
            if s:
                p = s["pos"]
                yaw, pitch, roll = get_chassis_orientation(stage, chassis_path)
                yaw_rad = math.radians(yaw)
                t = frame / PHYSICS_HZ
                for ct in contacts:
                    imp = ct["impulse"]
                    imp_mag = math.sqrt(imp[0]**2 + imp[1]**2 + imp[2]**2)
                    force_n = imp_mag * PHYSICS_HZ
                    lx, ly, lz = world_to_chassis_local(ct["position"], p, yaw_rad)
                    entry = {
                        "time": round(t, 4),
                        "frame": frame,
                        "world_pos": [round(x, 5) for x in ct["position"]],
                        "local_pos": [round(lx, 5), round(ly, 5), round(lz, 5)],
                        "normal": [round(x, 4) for x in ct["normal"]],
                        "force_N": round(force_n, 2),
                        "chassis_pos": [round(float(p[0]), 5), round(float(p[1]), 5), round(float(p[2]), 5)],
                        "chassis_ypr": [round(yaw, 2), round(pitch, 2), round(roll, 2)],
                        "reactor_prim": ct["other"],
                    }
                    contact_log.append(entry)
                    contact_count += 1

        # Status every second
        if frame % PHYSICS_HZ == 0:
            s = get_state(art, stage, chassis_path)
            if s:
                p = s["pos"]
                yaw, pitch, roll = get_chassis_orientation(stage, chassis_path)
                ct_this_sec = sum(1 for e in contact_log if e["time"] >= frame/PHYSICS_HZ - 1)
                print(f"[{frame/PHYSICS_HZ:.0f}s] pos=({p[0]/IN:.1f},{p[1]/IN:.1f},{p[2]/IN:.1f})\" "
                      f"ypr=({yaw:.1f},{pitch:.1f},{roll:.1f}) "
                      f"contacts_total={contact_count} last_sec={ct_this_sec}", flush=True)

    # Save results
    results_dir = os.path.join(RESULTS_ROOT, "xdrive_realwheel")
    os.makedirs(results_dir, exist_ok=True)
    results = {
        "test": "r0_entry_contact",
        "params": {
            "drive_speed_mps": drive_speed,
            "drive_time_s": drive_time,
            "belly_height_in": args.belly,
            "physics_hz": PHYSICS_HZ,
            "chassis_dims_in": [CHASSIS_L/IN, CHASSIS_W/IN, CHASSIS_H/IN],
            "wheel_radius_mm": TARGET_DIA_MM / 2.0,
        },
        "summary": {
            "total_contacts": len(contact_log),
            "total_frames": total_frames,
            "frames_with_contact": len(set(e["frame"] for e in contact_log)),
        },
        "contacts": contact_log,
    }

    # Add summary stats if there were contacts
    if contact_log:
        forces = [e["force_N"] for e in contact_log]
        local_xs = [e["local_pos"][0] for e in contact_log]
        local_ys = [e["local_pos"][1] for e in contact_log]
        pitches = [e["chassis_ypr"][1] for e in contact_log]
        results["summary"]["peak_force_N"] = round(max(forces), 2)
        results["summary"]["mean_force_N"] = round(sum(forces)/len(forces), 2)
        results["summary"]["first_contact_time_s"] = contact_log[0]["time"]
        results["summary"]["last_contact_time_s"] = contact_log[-1]["time"]
        results["summary"]["local_x_range"] = [round(min(local_xs)/IN, 2), round(max(local_xs)/IN, 2)]
        results["summary"]["local_y_range"] = [round(min(local_ys)/IN, 2), round(max(local_ys)/IN, 2)]
        results["summary"]["pitch_range_deg"] = [round(min(pitches), 1), round(max(pitches), 1)]

    out_path = os.path.join(results_dir, "r0_contact_report.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\n{'='*60}", flush=True)
    print(f"CONTACT REPORT SAVED: {out_path}", flush=True)
    print(f"  Total contact events: {len(contact_log)}", flush=True)
    print(f"  Frames with contact: {results['summary']['frames_with_contact']} / {total_frames}", flush=True)
    if contact_log:
        print(f"  Peak force: {results['summary']['peak_force_N']:.1f} N", flush=True)
        print(f"  First contact at t={results['summary']['first_contact_time_s']:.2f}s", flush=True)
        print(f"  Chassis local X range: {results['summary']['local_x_range']} in", flush=True)
        print(f"  Chassis local Y range: {results['summary']['local_y_range']} in", flush=True)
        print(f"  Pitch range: {results['summary']['pitch_range_deg']} deg", flush=True)
    else:
        print("  NO chassis-reactor contacts detected!", flush=True)
    print(f"{'='*60}", flush=True)

    # Generate visualization
    if contact_log:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib.patches import FancyBboxPatch, Rectangle
            from matplotlib.collections import PathCollection

            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle(f"R0 Entry Contact Analysis — belly={args.belly}\" speed={drive_speed}m/s",
                         fontsize=14, fontweight="bold")

            # 1. Top-down chassis outline with contact heatmap (local frame)
            ax = axes[0, 0]
            lx_in = [e["local_pos"][0]/IN for e in contact_log]
            ly_in = [e["local_pos"][1]/IN for e in contact_log]
            forces_plot = [e["force_N"] for e in contact_log]
            sc = ax.scatter(lx_in, ly_in, c=forces_plot, cmap="hot", s=4, alpha=0.5)
            plt.colorbar(sc, ax=ax, label="Force (N)")
            # Draw chassis outline
            chl = CHASSIS_L / 2.0 / IN
            chw = CHASSIS_W / 2.0 / IN
            cc = CHAMFER_CUT / IN
            oct_x = [chl, chl, chl-cc, -(chl-cc), -chl, -chl, -(chl-cc), chl-cc, chl]
            oct_y = [-(chw-cc), chw-cc, chw, chw, chw-cc, -(chw-cc), -chw, -chw, -(chw-cc)]
            ax.plot(oct_x, oct_y, 'b-', linewidth=2, label="Chassis outline")
            ax.set_xlabel("Local X (in) — +X = forward")
            ax.set_ylabel("Local Y (in)")
            ax.set_title("Contact Points on Chassis (top-down)")
            ax.set_aspect("equal")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            # 2. Side view: local X vs local Z
            ax = axes[0, 1]
            lz_in = [e["local_pos"][2]/IN for e in contact_log]
            sc2 = ax.scatter(lx_in, lz_in, c=forces_plot, cmap="hot", s=4, alpha=0.5)
            plt.colorbar(sc2, ax=ax, label="Force (N)")
            # Draw chassis side profile
            half_h = CHASSIS_H / 2.0 / IN
            belly_h = BELLY_HEIGHT / IN
            ml = MOTOR_MOUNT_LEN / IN
            af = ARCH_FLAT_WIDTH / 2.0 / IN
            re = chl - ml
            ax.plot([-chl, -re, -af, af, re, chl], [-half_h]*2 + [-half_h+belly_h]*2 + [-half_h]*2,
                    'b-', linewidth=2, label="Belly profile")
            ax.axhline(-half_h, color='gray', linestyle='--', alpha=0.5)
            ax.axhline(half_h, color='gray', linestyle='--', alpha=0.5)
            ax.set_xlabel("Local X (in) — +X = forward")
            ax.set_ylabel("Local Z (in)")
            ax.set_title("Contact Points (side view)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            # 3. Timeline: pitch + contact force over time
            ax = axes[1, 0]
            times = [e["time"] for e in contact_log]
            ax.scatter(times, forces_plot, c="red", s=2, alpha=0.3, label="Contact force")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Force (N)", color="red")
            ax.tick_params(axis="y", labelcolor="red")
            ax2 = ax.twinx()
            ax2.scatter(times, [e["chassis_ypr"][1] for e in contact_log],
                       c="blue", s=2, alpha=0.3, label="Pitch")
            ax2.set_ylabel("Pitch (deg)", color="blue")
            ax2.tick_params(axis="y", labelcolor="blue")
            ax.set_title("Contact Force & Pitch vs Time")
            ax.grid(True, alpha=0.3)

            # 4. World-space contact positions (bird's eye)
            ax = axes[1, 1]
            wx = [e["world_pos"][0]/IN for e in contact_log]
            wy = [e["world_pos"][1]/IN for e in contact_log]
            sc4 = ax.scatter(wx, wy, c=times, cmap="viridis", s=4, alpha=0.5)
            plt.colorbar(sc4, ax=ax, label="Time (s)")
            ax.set_xlabel("World X (in)")
            ax.set_ylabel("World Y (in)")
            ax.set_title("Contact Points in World (bird's eye)")
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            plot_path = os.path.join(results_dir, "r0_contact_report.png")
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"Plot saved: {plot_path}", flush=True)
        except Exception as ex:
            print(f"WARNING: Could not generate plot: {ex}", flush=True)

    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

if args.orbit_speed is not None:
    import csv as _csv
    speed = args.orbit_speed
    r0 = args.orbit_radius
    omega = speed / r0
    duration = args.orbit_orbits * 2.0 * math.pi * r0 / speed + 1.0
    settle_s = 2.0
    out_csv = args.orbit_csv or os.path.join(
        RESULTS_ROOT, "orbit_realwheel_5sphere",
        f"orbit_realwheel_{speed:.2f}mps.csv")
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)

    print(f"\nOrbit test: {speed:.2f} m/s @ R={r0:.3f}m  -> "
          f"omega={omega:.4f} rad/s, duration={duration:.1f}s", flush=True)
    print(f"CSV: {out_csv}", flush=True)

    art.set_joint_velocity_targets(np.zeros((1, ndof)))
    _orbit_render = not headless
    for _ in range(int(settle_s * PHYSICS_HZ)):
        world.step(render=_orbit_render)

    def _chassis_mat():
        prim = stage.GetPrimAtPath(chassis_path)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    def _yaw_deg(mat):
        return math.degrees(math.atan2(mat[1][0], mat[0][0]))

    s0 = get_state(art, stage, chassis_path)
    p0 = s0["pos"]
    yaw0 = _yaw_deg(_chassis_mat())
    print(f"Initial: ({p0[0]:.3f}, {p0[1]:.3f}, {p0[2]:.3f})m  yaw={yaw0:.1f}", flush=True)

    K_pos = 0.5
    V_MAX_FACTOR = 1.4
    print(f"Tracker gains: K_pos={K_pos}, vmax={V_MAX_FACTOR}xspeed", flush=True)
    va = np.zeros(ndof)
    init_phase = math.atan2(p0[1], p0[0])

    fcsv = open(out_csv, "w", newline="")
    writer = _csv.writer(fcsv)
    writer.writerow(["sim_time", "torque_FR", "torque_FL", "torque_BL",
                     "torque_BR", "x", "y", "z", "yaw_deg",
                     "actual_speed_mps", "R_m"])

    log_hz = 100
    log_interval = max(1, PHYSICS_HZ // log_hz)
    total_frames = int(duration * PHYSICS_HZ)
    prev_pos = np.array(p0)
    prev_t = 0.0

    for frame in range(total_frames):
        t = frame / PHYSICS_HZ

        phase_t = init_phase + omega * t
        tgt_x = r0 * math.cos(phase_t)
        tgt_y = r0 * math.sin(phase_t)
        tgt_vx = -r0 * omega * math.sin(phase_t)
        tgt_vy =  r0 * omega * math.cos(phase_t)

        mat_now = _chassis_mat()
        pos_now = mat_now.ExtractTranslation()
        yaw_now = math.atan2(mat_now[1][0], mat_now[0][0])

        ex = tgt_x - pos_now[0]
        ey = tgt_y - pos_now[1]
        v_world_x = tgt_vx + K_pos * ex
        v_world_y = tgt_vy + K_pos * ey

        vmag = math.sqrt(v_world_x * v_world_x + v_world_y * v_world_y)
        vcap = V_MAX_FACTOR * speed
        if vmag > vcap:
            v_world_x *= vcap / vmag
            v_world_y *= vcap / vmag

        cy = math.cos(-yaw_now)
        sy = math.sin(-yaw_now)
        vx_chs = cy * v_world_x - sy * v_world_y
        vy_chs = sy * v_world_x + cy * v_world_y

        tv = xdrive_ik(vx_chs, vy_chs, omega)
        for ii, di in enumerate(drive_dof_indices):
            va[di] = tv[ii]

        art.set_joint_velocity_targets(va.reshape(1, -1))
        world.step(render=_orbit_render)

        if frame % log_interval == 0:
            s = get_state(art, stage, chassis_path)
            if not s:
                continue
            pos = s["pos"]
            yaw = _yaw_deg(_chassis_mat())
            R = math.sqrt(pos[0] ** 2 + pos[1] ** 2)
            dt_log = t - prev_t if prev_t > 0 else 1.0 / log_hz
            if dt_log <= 0: dt_log = 1.0 / log_hz
            ddx = pos[0] - prev_pos[0]
            ddy = pos[1] - prev_pos[1]
            inst_v = math.sqrt(ddx * ddx + ddy * ddy) / dt_log

            torques = [0.0] * 4
            try:
                je = art.get_measured_joint_efforts()
            except Exception:
                je = None
            if je is not None:
                je_flat = np.asarray(je).flatten()
                for ii, di in enumerate(drive_dof_indices):
                    if di < len(je_flat):
                        torques[ii] = float(je_flat[di])

            writer.writerow([
                f"{t:.4f}",
                f"{torques[0]:.4f}", f"{torques[1]:.4f}",
                f"{torques[2]:.4f}", f"{torques[3]:.4f}",
                f"{pos[0]:.5f}", f"{pos[1]:.5f}", f"{pos[2]:.5f}",
                f"{yaw:.2f}", f"{inst_v:.4f}", f"{R:.4f}",
            ])
            prev_pos = np.array(pos)
            prev_t = t

        if frame % (PHYSICS_HZ * 2) == 0 and frame > 0:
            s = get_state(art, stage, chassis_path)
            if s:
                pos = s["pos"]
                R = math.sqrt(pos[0] ** 2 + pos[1] ** 2)
                ang = math.degrees(math.atan2(pos[1], pos[0]))
                print(f"  t={t:5.1f}s  R={R:.3f}m  ang={ang:+.1f}deg  "
                      f"yaw={_yaw_deg(_chassis_mat()):+.1f}", flush=True)

    fcsv.close()
    sf = get_state(art, stage, chassis_path)
    if sf:
        pf = sf["pos"]
        net = math.sqrt((pf[0] - p0[0]) ** 2 + (pf[1] - p0[1]) ** 2)
        print(f"\nDone. Net displacement {net:.3f}m. CSV: {out_csv}", flush=True)
    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

if headless:
    for test_name, tvx, tvy, tw in [
        ("forward", 0.2, 0, 0),
        ("strafe left", 0, 0.2, 0),
        ("rotate CW", 0, 0, -0.5),
    ]:
        print(f"\nTest: {test_name} for 2s...", flush=True)
        tv = xdrive_ik(tvx, tvy, tw)
        va = np.zeros(ndof)
        for ii, di in enumerate(drive_dof_indices):
            va[di] = tv[ii]
        for _ in range(2 * PHYSICS_HZ):
            art.set_joint_velocity_targets(va.reshape(1, -1))
            world.step(render=False)
        print_state(art, stage, chassis_path, 0)
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
        for _ in range(PHYSICS_HZ):
            world.step(render=False)

    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# GUI
kb = KB()
if args.reactor:
    spawn_pos = [SPAWN_X, SPAWN_Y, spawn_z]
    spawn_yaw_reset = spawn_yaw
else:
    spawn_pos = [0, 0, spawn_z]
    spawn_yaw_reset = 0.0
print("\nControls:", flush=True)
print("  Arrows = translate  |  Q/E = rotate  |  +/- = speed", flush=True)
print("  O = orbit toggle  |  9/0 = orbit radius", flush=True)
print("  R = reset  |  P = print state  |  Space = stop", flush=True)
if contact_sensor:
    print("  Contact reporting active — scrape events logged to console", flush=True)

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
        art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
        contact_log.clear()
        print("RESET (contact log cleared)", flush=True)
        continue

    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    for ii, di in enumerate(drive_dof_indices):
        va[di] = tv[ii]
    art.set_joint_velocity_targets(va.reshape(1, -1))
    world.step(render=True)
    frame += 1

    # Log contacts in GUI mode too
    if contact_sensor:
        contacts = read_chassis_contacts(contact_sensor, chassis_path)
        if contacts:
            s = get_state(art, stage, chassis_path)
            if s:
                p = s["pos"]
                yaw, pitch, roll = get_chassis_orientation(stage, chassis_path)
                for ct in contacts:
                    imp = ct["impulse"]
                    force_n = math.sqrt(imp[0]**2 + imp[1]**2 + imp[2]**2) * PHYSICS_HZ
                    if frame % 30 == 0:  # don't spam console every frame
                        print(f"  SCRAPE frame={frame} force={force_n:.1f}N "
                              f"at world=({ct['position'][0]/IN:.1f},{ct['position'][1]/IN:.1f},"
                              f"{ct['position'][2]/IN:.1f})\" pitch={pitch:.1f}deg", flush=True)

    if kb.pstate:
        kb.pstate = False
        print_state(art, stage, chassis_path, frame)
        print(f"  Cmd: vx={vx:.2f} vy={vy:.2f} w={w:.2f}", flush=True)

    if frame % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            prim = stage.GetPrimAtPath(chassis_path)
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
            jv_str = ""
            if s["jv"] is not None:
                drive_vels = [s["jv"].flatten()[di] for di in drive_dof_indices]
                jv_str = f" wv=[{','.join(f'{x:.1f}' for x in drive_vels)}]"
            print(f"[{frame/PHYSICS_HZ:.0f}s] pos=({p[0]/IN:.1f},{p[1]/IN:.1f},{p[2]/IN:.1f})\" "
                  f"yaw={yaw:.1f}deg{jv_str}", flush=True)

app.close()
