"""
FORTIS X-drive with arched underbelly and belly rollers.

15x9x5.5" chassis. Motors at 4 chamfer corners (NEO 2.0 + 5:1 MAXPlanetary).
Flat motor zones at front/rear cover the diagonal motor positions, then the
underbelly arches up in the center. Passive rollers span the arch zone.
8" wheels (sphere contact surface = 4" radius).

Usage: IsaacSim\\python.bat xdrive_belly_rollers.py --gui
       IsaacSim\\python.bat xdrive_belly_rollers.py --gui --drop 2
"""
import os, sys, math, argparse
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drop", type=float, default=0.0, help="Wheel drop in inches")
cli, _ = parser.parse_known_args()
headless = cli.headless or not cli.gui

from isaacsim import SimulationApp
app = SimulationApp({"headless": headless, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
import carb.input
import omni.appwindow
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

IN = 0.0254
MM = 0.001

# === CHASSIS ===
CHASSIS_L = 15.0 * IN
CHASSIS_W = 9.0 * IN
CHASSIS_H = 5.5 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
CHASSIS_MASS = 20.4  # 45 lbs

# === WHEELS (AndyMark 8" Dualie am-0463) ===
WHEEL_RADIUS = 4.0 * IN   # 8" diameter, sphere surfaces define this
WHEEL_WIDTH = 2.04 * IN
WHEEL_MASS = 1.0
WHEEL_DROP = cli.drop * IN

# === MOTORS (NEO 2.0 + 5:1 MAXPlanetary) ===
# NEO 2.0: 48mm body + MAXPlanetary ~50mm = ~98mm total = 3.86"
# Motor extends inward from chamfer face along the diagonal.
# X-component of 3.86" at 45 deg = 2.73". Add margin -> ~4" from chassis edge.
MOTOR_TOTAL_LEN = (48 + 50) * MM  # 98mm = 3.86"
MOTOR_MOUNT_LEN = 1.5 * IN  # 90-degree gearbox: motor points up, tiny mounting pad

# === ARCH ===
ARCH_HEIGHT = 2.5 * IN
ARCH_FLAT_WIDTH = 5.0 * IN  # wide flat top -- rollers cover flat and ramps

# === BELLY ROLLERS ===
ROLLER_OD = 1.25 * IN
ROLLER_RADIUS = ROLLER_OD / 2.0
ROLLER_LENGTH = 7.5 * IN
ROLLER_SPACING = 1.375 * IN  # tighter packing, 0.125" gap between rollers
ROLLER_MASS = 0.1
ROLLER_PROTRUSION = 0.25 * IN  # below the arch ceiling
BRACKET_WIDTH = 0.5 * IN
BRACKET_THICKNESS = 0.125 * IN

# === OMNI WHEEL SPHERES ===
SPHERE_RADIUS = 12.7 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS
ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 3
ROW_OFFSET_DEG = 15.0

# === PHYSICS ===
FRICTION_MU = 0.5
OMNI_ANGLE = 90.0
DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5
PHYSICS_HZ = 240

# === GEOMETRY ===
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT
half_h = CHASSIS_H / 2.0

OCT_XY = [
    ( SL,       -(SW - C)),  ( SL,        (SW - C)),
    ( (SL - C),  SW      ),  (-(SL - C),  SW      ),
    (-SL,        (SW - C)),  (-SL,       -(SW - C)),
    (-(SL - C), -SW      ),  ( (SL - C), -SW      ),
]

def mid(a, b): return ((a[0]+b[0])/2, (a[1]+b[1])/2)
WHEEL_XY = {
    "FR": mid(OCT_XY[0], OCT_XY[7]), "FL": mid(OCT_XY[1], OCT_XY[2]),
    "BL": mid(OCT_XY[3], OCT_XY[4]), "BR": mid(OCT_XY[5], OCT_XY[6]),
}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]


def build_arched_chassis(stage, path):
    """Octagonal chassis with trapezoidal arch on the underbelly.
    Front/rear zones stay flat for motor mounting at the 4 chamfer corners.
    Center zone arches up. Cross-sections along X define the profile."""
    z_low = -half_h
    z_high = -half_h + ARCH_HEIGHT
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half
    ramp_end = SL - MOTOR_MOUNT_LEN  # where ramp meets motor zone

    x_stations = sorted(set([
        -SL, -(SL - C), -ramp_end, -ramp_start,
        0.0,
        ramp_start, ramp_end, (SL - C), SL,
    ]))

    def bottom_z(x):
        ax = abs(x)
        if ax <= ramp_start: return z_high    # flat arch center
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)  # ramp
        else: return z_low                     # motor mount flat

    def half_width_at(x):
        ax = abs(x)
        if ax <= (SL - C): return SW
        else: return SW - (ax - (SL - C)) / C * C  # chamfer taper

    verts, face_indices, face_counts = [], [], []
    sections = []
    for x in x_stations:
        hw = half_width_at(x)
        bz = bottom_z(x)
        idx = len(verts)
        verts.extend([
            Gf.Vec3f(x,  hw, half_h),   # top-left
            Gf.Vec3f(x, -hw, half_h),   # top-right
            Gf.Vec3f(x, -hw, bz),       # bottom-right
            Gf.Vec3f(x,  hw, bz),       # bottom-left
        ])
        sections.append((idx, idx+1, idx+2, idx+3))

    for i in range(len(sections) - 1):
        tl0, tr0, br0, bl0 = sections[i]
        tl1, tr1, br1, bl1 = sections[i + 1]
        for quad in [(tl0,tl1,tr1,tr0), (bl0,br0,br1,bl1),
                     (tl0,bl0,bl1,tl1), (tr0,tr1,br1,br0)]:
            face_indices.extend(quad)
            face_counts.append(4)

    # End caps
    tl, tr, br, bl = sections[-1]
    face_indices.extend([tl, tr, br, bl]); face_counts.append(4)
    tl, tr, br, bl = sections[0]
    face_indices.extend([tr, tl, bl, br]); face_counts.append(4)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.25, 0.25, 0.35)]))
    return mesh


def build_sphere_roller_wheel(stage, wp, wheel_mat, color):
    """Omni wheel spheres. Contact surface at WHEEL_RADIUS from center."""
    hw = WHEEL_WIDTH / 2.0
    for row in range(2):
        ao = ROW_OFFSET_DEG * row
        ys = -hw if row == 0 else 0.0
        ye = 0.0 if row == 0 else hw
        ysp = (ye - ys) / (SPHERES_PER_ROLLER + 1)
        for ri in range(ROLLERS_PER_ROW):
            ra = math.radians(360.0 * ri / ROLLERS_PER_ROW + ao)
            rx = SPHERE_CENTER_R * math.cos(ra)
            rz = SPHERE_CENTER_R * math.sin(ra)
            for si in range(SPHERES_PER_ROLLER):
                sy = ys + ysp * (si + 1)
                sp = f"{wp}/row{row}_r{ri}_s{si}"
                sphere = UsdGeom.Sphere.Define(stage, sp)
                sphere.GetRadiusAttr().Set(SPHERE_RADIUS)
                sxf = UsdGeom.Xformable(sphere.GetPrim())
                sxf.ClearXformOpOrder()
                sxf.AddTranslateOp().Set(Gf.Vec3d(rx, sy, rz))
                UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
                UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
                    wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    vis = UsdGeom.Cylinder.Define(stage, wp + "/visual")
    vis.GetRadiusAttr().Set(WHEEL_RADIUS * 0.92)
    vis.GetHeightAttr().Set(WHEEL_WIDTH * 0.85)
    vis.GetAxisAttr().Set("Y")
    vis.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def build_scene(stage):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateSolverTypeAttr("TGS")
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateBroadphaseTypeAttr("GPU")

    gp = UsdGeom.Mesh.Define(stage, "/World/Ground")
    gs = 10.0
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


def build_robot(stage):
    robot_path = "/World/Robot"
    chassis_path = robot_path + "/Chassis"
    UsdGeom.Xform.Define(stage, robot_path)
    UsdGeom.Xform.Define(stage, chassis_path)

    cp = stage.GetPrimAtPath(chassis_path)
    UsdPhysics.RigidBodyAPI.Apply(cp)
    mapi = UsdPhysics.MassAPI.Apply(cp)
    mapi.CreateMassAttr(CHASSIS_MASS)
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(cp)
    PhysxSchema.PhysxArticulationAPI.Apply(cp)

    # convexDecomposition breaks the concave arch into convex pieces
    body = build_arched_chassis(stage, chassis_path + "/body")
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(body.GetPrim()).GetApproximationAttr().Set("convexDecomposition")

    # Forward arrow
    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim())
    axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))

    # Materials
    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    rmat = UsdShade.Material.Define(stage, "/World/RollerMat")
    rpm = UsdPhysics.MaterialAPI.Apply(rmat.GetPrim())
    rpm.CreateStaticFrictionAttr(FRICTION_MU)
    rpm.CreateDynamicFrictionAttr(FRICTION_MU)
    rpm.CreateRestitutionAttr(0.05)
    roller_mat = UsdShade.Material(stage.GetPrimAtPath("/World/RollerMat"))

    wcolors = {"FR": (0.85, 0.2, 0.2), "FL": (0.2, 0.75, 0.2),
               "BL": (0.2, 0.2, 0.85), "BR": (0.85, 0.75, 0.2)}

    # === DRIVE WHEELS ===
    wheel_z = -WHEEL_DROP
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.003
    joint_paths = []

    for wn in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wn]
        ad = AXLE_ANGLES[wn]
        ar = math.radians(ad)
        nl = math.sqrt(cx*cx + cy*cy)
        nx, ny = cx/nl, cy/nl
        wx = cx + nx * wheel_offset
        wy = cy + ny * wheel_offset

        wp = robot_path + f"/Wheel_{wn}"
        UsdGeom.Xform.Define(stage, wp)
        wprim = stage.GetPrimAtPath(wp)
        UsdPhysics.RigidBodyAPI.Apply(wprim)
        UsdPhysics.MassAPI.Apply(wprim).CreateMassAttr(WHEEL_MASS)
        wxf = UsdGeom.Xformable(wprim)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(ad)

        build_sphere_roller_wheel(stage, wp, wheel_mat, wcolors[wn])

        jp = robot_path + f"/Joint_{wn}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.GetBody0Rel().SetTargets([Sdf.Path(chassis_path)])
        joint.GetBody1Rel().SetTargets([Sdf.Path(wp)])
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(wx, wy, wheel_z))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.GetAxisAttr().Set("Y")
        ha = ar / 2.0
        joint.GetLocalRot0Attr().Set(Gf.Quatf(math.cos(ha), 0, 0, math.sin(ha)))
        joint.GetLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetDampingAttr().Set(100.0)
        drive.GetMaxForceAttr().Set(200.0)
        drive.GetTargetVelocityAttr().Set(0.0)

        jprim = stage.GetPrimAtPath(jp)
        jprim.CreateAttribute("isaacmecanumwheel:radius",
                              Sdf.ValueTypeNames.Float).Set(float(WHEEL_RADIUS))
        jprim.CreateAttribute("isaacmecanumwheel:angle",
                              Sdf.ValueTypeNames.Float).Set(float(OMNI_ANGLE))
        joint_paths.append(jp)

    # === BELLY ROLLERS ===
    # Live in the arch zone (center of chassis). Mounted on brackets.
    arch_ceiling_z = -half_h + ARCH_HEIGHT  # arch ceiling in chassis frame
    roller_center_z = arch_ceiling_z - ROLLER_PROTRUSION  # below arch ceiling

    # Roller zone = arch zone = between the motor mount flat areas
    arch_zone_half = SL - MOTOR_MOUNT_LEN  # 7.5 - 4.0 = 3.5"
    n_rollers = max(1, int(2 * arch_zone_half / ROLLER_SPACING))
    if n_rollers % 2 == 0:
        n_rollers -= 1

    roller_xs = [(i - n_rollers // 2) * ROLLER_SPACING for i in range(n_rollers)]

    # Bracket dimensions: hang from arch ceiling down to roller center
    bracket_drop = ROLLER_PROTRUSION + ROLLER_RADIUS
    bracket_half_y = ROLLER_LENGTH / 2.0 + BRACKET_WIDTH / 2.0

    # Clearance report
    ctr_z = WHEEL_RADIUS + WHEEL_DROP
    flat_z = ctr_z - half_h  # motor mount flat above ground
    arch_z = ctr_z + arch_ceiling_z  # arch ceiling above ground
    roller_bot_z = ctr_z + roller_center_z - ROLLER_RADIUS  # roller bottom above ground

    print(f"\n--- CHASSIS PROFILE ---", flush=True)
    print(f"  Chassis: {CHASSIS_L/IN:.0f}x{CHASSIS_W/IN:.0f}x{CHASSIS_H/IN:.1f}\" "
          f"({CHASSIS_MASS/0.4536:.0f} lbs)", flush=True)
    print(f"  Motor mount flat: {MOTOR_MOUNT_LEN/IN:.1f}\" each end "
          f"(NEO 2.0 + MAXPlanetary = {MOTOR_TOTAL_LEN/IN:.2f}\" at 45deg = "
          f"{MOTOR_TOTAL_LEN*math.cos(math.radians(45))/IN:.2f}\" X-component)", flush=True)
    print(f"  Arch: {ARCH_HEIGHT/IN:.1f}\" rise, {ARCH_FLAT_WIDTH/IN:.1f}\" flat center, "
          f"zone: +/-{arch_zone_half/IN:.1f}\"", flush=True)
    print(f"\n--- BELLY ROLLERS ---", flush=True)
    print(f"  {n_rollers}x {ROLLER_OD/IN:.2f}\" OD x {ROLLER_LENGTH/IN:.1f}\" long, "
          f"spacing {ROLLER_SPACING/IN:.2f}\" (gap {(ROLLER_SPACING-ROLLER_OD)/IN:.3f}\")", flush=True)
    print(f"  Protrusion below arch ceiling: {ROLLER_PROTRUSION/IN:.2f}\"", flush=True)
    print(f"\n--- CLEARANCES (drop={WHEEL_DROP/IN:.1f}\") ---", flush=True)
    print(f"  Motor mount flat above ground: {flat_z/IN:.2f}\"", flush=True)
    print(f"  Arch ceiling above ground: {arch_z/IN:.2f}\"", flush=True)
    print(f"  Roller bottom above ground: {roller_bot_z/IN:.2f}\"", flush=True)
    if roller_bot_z < 0.01:
        print(f"  ** WARNING: rollers hit ground! Increase drop or reduce protrusion **", flush=True)
    else:
        print(f"  All clear", flush=True)
    print(flush=True)

    for ri, rx in enumerate(roller_xs):
        rp = robot_path + f"/Roller_{ri}"
        UsdGeom.Xform.Define(stage, rp)
        rprim = stage.GetPrimAtPath(rp)
        UsdPhysics.RigidBodyAPI.Apply(rprim)
        UsdPhysics.MassAPI.Apply(rprim).CreateMassAttr(ROLLER_MASS)

        rxf = UsdGeom.Xformable(rprim)
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(rx, 0, roller_center_z))

        # Roller cylinder
        cyl = UsdGeom.Cylinder.Define(stage, rp + "/cyl")
        cyl.GetRadiusAttr().Set(ROLLER_RADIUS)
        cyl.GetHeightAttr().Set(ROLLER_LENGTH)
        cyl.GetAxisAttr().Set("Y")
        cyl.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.75, 0.75, 0.3)]))
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(
            roller_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

        # Mounting brackets (visual, part of chassis)
        for side, sy in [("L", bracket_half_y), ("R", -bracket_half_y)]:
            br = UsdGeom.Cube.Define(stage, chassis_path + f"/brk_{side}_{ri}")
            br.GetSizeAttr().Set(1.0)
            bxf = UsdGeom.Xformable(br.GetPrim())
            bxf.ClearXformOpOrder()
            bxf.AddTranslateOp().Set(Gf.Vec3d(rx, sy,
                                               arch_ceiling_z - bracket_drop / 2.0))
            bxf.AddScaleOp().Set(Gf.Vec3d(BRACKET_WIDTH, BRACKET_THICKNESS, bracket_drop))
            br.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.4, 0.4, 0.4)]))

        # Revolute joint -- free spinning
        jrp = robot_path + f"/RollerJoint_{ri}"
        rjoint = UsdPhysics.RevoluteJoint.Define(stage, jrp)
        rjoint.GetBody0Rel().SetTargets([Sdf.Path(chassis_path)])
        rjoint.GetBody1Rel().SetTargets([Sdf.Path(rp)])
        rjoint.GetLocalPos0Attr().Set(Gf.Vec3f(rx, 0, roller_center_z))
        rjoint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        rjoint.GetAxisAttr().Set("Y")
        rjoint.GetLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        rjoint.GetLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        rdrive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jrp), "angular")
        rdrive.GetStiffnessAttr().Set(0.0)
        rdrive.GetDampingAttr().Set(0.1)
        rdrive.GetMaxForceAttr().Set(0.0)

    # Spawn
    spawn_z = WHEEL_RADIUS + WHEEL_DROP + 0.01
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    ts = 2 * ROLLERS_PER_ROW * SPHERES_PER_ROLLER
    print(f"Wheels: 8\" dia, {WHEEL_WIDTH/IN:.2f}\" wide, "
          f"drop={WHEEL_DROP/IN:.1f}\", {ts} spheres/wheel", flush=True)
    print(f"Motor: NEO 2.0 + 5:1 MAXPlanetary = 18.75 Nm at wheel", flush=True)
    print(flush=True)

    return robot_path, chassis_path, joint_paths, spawn_z, n_rollers


def xdrive_ik(vx, vy, omega):
    r = WHEEL_RADIUS
    vels = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        vwx = vx - omega * wy
        vwy = vy + omega * wx
        a = math.radians(AXLE_ANGLES[wn])
        vels.append((vwx * math.cos(a) + vwy * math.sin(a)) / r)
    return np.array(vels)


def get_state(art, stage, chassis_path):
    prim = stage.GetPrimAtPath(chassis_path)
    if not prim.IsValid(): return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
    pitch = math.degrees(math.atan2(-mat[2][0], math.sqrt(mat[2][1]**2 + mat[2][2]**2)))
    roll = math.degrees(math.atan2(mat[2][1], mat[2][2]))
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos": np.array([t[0], t[1], t[2]]), "yaw": yaw, "pitch": pitch,
            "roll": roll, "jv": jv, "je": je}


class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.orbit_radius = 1.0
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
                print(f"Orbit: {labels[self.orbit]} r={self.orbit_radius/IN:.1f}\"")
            elif k == K.KEY_9:
                self.orbit_radius = max(self.orbit_radius - 0.1, 0.2)
                print(f"Orbit radius: {self.orbit_radius/IN:.1f}\"")
            elif k == K.KEY_0:
                self.orbit_radius = min(self.orbit_radius + 0.1, 5.0)
                print(f"Orbit radius: {self.orbit_radius/IN:.1f}\"")
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


# ========== MAIN ==========
print("=" * 60, flush=True)
print("FORTIS X-Drive — Arched Chassis + Belly Rollers", flush=True)
print("=" * 60, flush=True)

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_scene(stage)
robot_path, chassis_path, joint_paths, spawn_z, n_rollers = build_robot(stage)

for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10): world.step(render=not headless)

if art.is_physics_handle_valid():
    ndof = art.num_dof
    print(f"Articulation: {ndof} DOFs (4 wheels + {n_rollers} rollers)", flush=True)
    art.switch_control_mode("velocity", joint_indices=np.arange(4))
else:
    print("FATAL: Articulation invalid", flush=True)
    app.close()
    sys.exit(1)

print("Settling 2s...", flush=True)
for _ in range(2 * PHYSICS_HZ):
    world.step(render=not headless)

s = get_state(art, stage, chassis_path)
if s:
    p = s["pos"]
    print(f"  Settled: ({p[0]/IN:.2f}\",{p[1]/IN:.2f}\",{p[2]/IN:.2f}\") "
          f"yaw={s['yaw']:.2f} pitch={s['pitch']:.2f}", flush=True)

if headless:
    for name, tvx, tvy, tw in [
        ("forward", 0.2, 0, 0), ("strafe", 0, 0.2, 0),
        ("rotate", 0, 0, -0.5), ("orbit", 0, 0.15, 0.15/1.0),
    ]:
        print(f"\nHeadless: {name} for 3s...", flush=True)
        tv = xdrive_ik(tvx, tvy, tw)
        va = np.zeros(ndof)
        va[:4] = tv
        for _ in range(3 * PHYSICS_HZ):
            art.set_joint_velocity_targets(va.reshape(1, -1))
            world.step(render=False)
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"  Result: ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                  f"yaw={s['yaw']:.1f}", flush=True)
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
        for _ in range(PHYSICS_HZ): world.step(render=False)
    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# GUI
kb = KB()
spawn_pos = [0, 0, spawn_z]
print("Controls:", flush=True)
print("  Arrows=drive Q/E=rotate +/-=speed O=orbit 9/0=radius", flush=True)
print("  R=reset P=state Space=stop", flush=True)

frame = 0
while app.is_running():
    vx, vy, w = kb.cmd()

    if kb.stop:
        kb.stop = False
        if art.is_physics_handle_valid():
            art.set_joint_velocity_targets(np.zeros((1, ndof)))
        vx = vy = w = 0.0
    elif kb.reset:
        kb.reset = False
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5): app.update()
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos))
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(10): world.step(render=True)
        art.switch_control_mode("velocity", joint_indices=np.arange(4))
        print("RESET", flush=True)
        continue

    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    va[:4] = tv
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))
    world.step(render=True)
    frame += 1

    if kb.pstate:
        kb.pstate = False
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"\n--- Frame {frame} ---", flush=True)
            print(f"  Pos: ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\")", flush=True)
            print(f"  YPR: ({s['yaw']:.1f},{s['pitch']:.1f},{s['roll']:.1f})", flush=True)
            if s["je"] is not None:
                e = s["je"].flatten()[:4]
                print(f"  Wheel torque: [{','.join(f'{x:.2f}' for x in e)}] Nm", flush=True)

    if frame % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            je_str = ""
            if s["je"] is not None:
                e = s["je"].flatten()[:4]
                je_str = f" t=[{','.join(f'{x:.2f}' for x in e)}]"
            orb_str = ""
            if kb.orbit != 0:
                orb_str = f" orb={'CCW' if kb.orbit>0 else 'CW'} r={kb.orbit_radius/IN:.1f}\""
            print(f"[{frame/PHYSICS_HZ:.0f}s] ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                  f"ypr=({s['yaw']:.1f},{s['pitch']:.1f},{s['roll']:.1f}) "
                  f"cmd=({vx:.2f},{vy:.2f},{w:.2f}){orb_str}{je_str}", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
