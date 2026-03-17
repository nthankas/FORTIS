"""
FORTIS X-drive drift and directional accuracy test.

Runs forward/backward/strafe/rotate/diagonal at multiple speeds, measures
lateral drift, heading error, and coasting distance. Self-contained --
all constants and build functions are inline (no imports from xdrive_o3dyn).

Usage: IsaacSim\\python.bat test_drift.py --gui
       IsaacSim\\python.bat test_drift.py --headless
"""
import os, sys, math, argparse
import numpy as np
os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
cli, _ = parser.parse_known_args()
headless = cli.headless or not cli.gui

from isaacsim import SimulationApp
app = SimulationApp({"headless": headless, "width": 1920, "height": 1080})

import omni.usd, omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

IN = 0.0254
MM = 0.001

CHASSIS_L = 15.0 * IN
CHASSIS_W = 9.0 * IN
CHASSIS_H = 6.0 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)

WHEEL_RADIUS = 90 * MM
WHEEL_WIDTH = 2.205 * IN
WHEEL_MASS = 1.0
CHASSIS_MASS = 18.1

FRICTION_MU = 0.5
OMNI_ANGLE = 90.0
DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5

ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 3
SPHERE_RADIUS = 12.7 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS
ROW_OFFSET_DEG = 15.0
PHYSICS_HZ = 240

SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

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


def build_octagonal_prism(stage, path, half_h, color):
    n = len(OCT_XY)
    verts = []
    for x, y in OCT_XY:
        verts.append(Gf.Vec3f(x, y, -half_h))
    for x, y in OCT_XY:
        verts.append(Gf.Vec3f(x, y, half_h))
    fi, fc = [], []
    for i in range(n):
        j = (i + 1) % n
        fi.extend([i, j, j + n, i + n])
        fc.append(4)
    fi.extend(list(range(n - 1, -1, -1)))
    fc.append(n)
    fi.extend(list(range(n, 2 * n)))
    fc.append(n)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def build_sphere_roller_wheel(stage, wp, wheel_mat, color):
    """Roller-based sphere layout. Gaps between rollers create directional friction."""
    half_w = WHEEL_WIDTH / 2.0

    for row in range(2):
        angle_offset = ROW_OFFSET_DEG * row
        y_start = -half_w if row == 0 else 0.0
        y_end = 0.0 if row == 0 else half_w
        y_spacing = (y_end - y_start) / (SPHERES_PER_ROLLER + 1)

        for ri in range(ROLLERS_PER_ROW):
            roller_angle = math.radians(360.0 * ri / ROLLERS_PER_ROW + angle_offset)
            rx = SPHERE_CENTER_R * math.cos(roller_angle)
            rz = SPHERE_CENTER_R * math.sin(roller_angle)

            for si in range(SPHERES_PER_ROLLER):
                sy = y_start + y_spacing * (si + 1)

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
    vis.GetRadiusAttr().Set(WHEEL_RADIUS * 0.95)
    vis.GetHeightAttr().Set(WHEEL_WIDTH * 0.9)
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
    UsdPhysics.MassAPI.Apply(cp).CreateMassAttr(CHASSIS_MASS)
    UsdPhysics.MassAPI(cp).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(cp)
    PhysxSchema.PhysxArticulationAPI.Apply(cp)

    half_h = CHASSIS_H / 2.0
    build_octagonal_prism(stage, chassis_path + "/body", half_h, (0.25, 0.25, 0.35))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(chassis_path + "/body"))
    UsdPhysics.MeshCollisionAPI.Apply(
        stage.GetPrimAtPath(chassis_path + "/body")).GetApproximationAttr().Set("convexHull")

    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim())
    axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))

    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    wcolors = {"FR": (0.85,0.2,0.2), "FL": (0.2,0.75,0.2),
               "BL": (0.2,0.2,0.85), "BR": (0.85,0.75,0.2)}

    wheel_z = 0.0
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.003

    for wn in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wn]
        ad = AXLE_ANGLES[wn]
        ar = math.radians(ad)
        nl = math.sqrt(cx*cx + cy*cy)
        nx, ny = cx/nl, cy/nl
        wx, wy = cx + nx*wheel_offset, cy + ny*wheel_offset

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

    spawn_z = WHEEL_RADIUS + 0.01
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    return robot_path, chassis_path, spawn_z


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


def get_pose(stage, chassis_path):
    prim = stage.GetPrimAtPath(chassis_path)
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
    pitch = math.degrees(math.atan2(-mat[2][0], math.sqrt(mat[2][1]**2 + mat[2][2]**2)))
    roll = math.degrees(math.atan2(mat[2][1], mat[2][2]))
    return np.array([t[0], t[1], t[2]]), yaw, pitch, roll


def get_state(art, stage, chassis_path):
    p, yaw, pitch, roll = get_pose(stage, chassis_path)
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos": p, "yaw": yaw, "pitch": pitch, "roll": roll, "jv": jv, "je": je}


def make_step_surface(stage, step_height):
    """Two flat surfaces separated by a step at X=0."""
    # Lower surface: Z=0, extends in -X direction
    lower = UsdGeom.Mesh.Define(stage, "/World/LowerFloor")
    s = 2.0
    lower.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-s, -s, 0), Gf.Vec3f(0, -s, 0),
        Gf.Vec3f(0, s, 0), Gf.Vec3f(-s, s, 0)]))
    lower.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    lower.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    lower.GetSubdivisionSchemeAttr().Set("none")
    lower.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.6, 0.6, 0.7)]))
    UsdPhysics.CollisionAPI.Apply(lower.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(lower.GetPrim()).GetApproximationAttr().Set("none")

    # Upper surface: Z=step_height, extends in +X direction
    h = step_height
    upper = UsdGeom.Mesh.Define(stage, "/World/UpperFloor")
    upper.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(0, -s, h), Gf.Vec3f(s, -s, h),
        Gf.Vec3f(s, s, h), Gf.Vec3f(0, s, h)]))
    upper.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    upper.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    upper.GetSubdivisionSchemeAttr().Set("none")
    upper.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.7, 0.6, 0.6)]))
    UsdPhysics.CollisionAPI.Apply(upper.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(upper.GetPrim()).GetApproximationAttr().Set("none")

    # Ground friction
    gmat = UsdShade.Material.Define(stage, "/World/FloorMat")
    gpm = UsdPhysics.MaterialAPI.Apply(gmat.GetPrim())
    gpm.CreateStaticFrictionAttr(FRICTION_MU)
    gpm.CreateDynamicFrictionAttr(FRICTION_MU)
    gpm.CreateRestitutionAttr(0.05)
    for p in [lower, upper]:
        UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(
            gmat, UsdShade.Tokens.weakerThanDescendants, "physics")


# ========== MAIN ==========
print("=" * 90, flush=True)
print("FORTIS X-Drive Drift Test", flush=True)
total_spheres = 2 * ROLLERS_PER_ROW * SPHERES_PER_ROLLER
print(f"Sphere: r={SPHERE_RADIUS*1000:.1f}mm at R={SPHERE_CENTER_R*1000:.1f}mm, "
      f"2x{ROLLERS_PER_ROW}x{SPHERES_PER_ROLLER} = "
      f"{total_spheres}/wheel ({total_spheres*4} total)", flush=True)
print(f"Wheel: {WHEEL_RADIUS*1000:.0f}mm dia, {WHEEL_WIDTH/IN:.3f}\" wide", flush=True)
print(f"Chassis: {CHASSIS_L/IN:.0f}x{CHASSIS_W/IN:.0f}x{CHASSIS_H/IN:.0f}\"", flush=True)
print("=" * 90, flush=True)

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
build_scene(stage)

robot_path, chassis_path, spawn_z = build_robot(stage)

for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10): world.step(render=not headless)

ndof = 0
if art.is_physics_handle_valid():
    ndof = art.num_dof
    print(f"Articulation: {ndof} DOFs", flush=True)
    art.switch_control_mode("velocity", joint_indices=np.arange(ndof))

# Settle
print("Settling 2s...", flush=True)
for _ in range(2 * PHYSICS_HZ):
    world.step(render=not headless)

s = get_state(art, stage, chassis_path)
if s:
    p = s["pos"]
    print(f"  Settled: ({p[0]/IN:.2f}\",{p[1]/IN:.2f}\",{p[2]/IN:.2f}\") "
          f"yaw={s['yaw']:.2f}", flush=True)

# Headless: run automated sweep at multiple speeds and directions
if headless:
    speeds = [0.1, 0.2, 0.3, 0.5]
    tests = [
        ("forward",      1, 0, 0),
        ("backward",    -1, 0, 0),
        ("strafe_left",  0, 1, 0),
        ("strafe_right", 0,-1, 0),
        ("rotate_cw",   0, 0,-1),
        ("rotate_ccw",  0, 0, 1),
        ("diagonal",    0.707, 0.707, 0),
    ]

    print(f"\n{'Test':<20} {'Speed':>6} {'dx':>7} {'dy':>7} {'dyaw':>7} "
          f"{'Drift%':>7} {'HeadErr':>8} {'Coast':>6}", flush=True)
    print("-" * 85, flush=True)

    for name, dvx, dvy, dw in tests:
        for spd in speeds:
            vx = dvx * spd
            vy = dvy * spd
            omega = dw * spd

            p0, yaw0, _, _ = get_pose(stage, chassis_path)

            tv = xdrive_ik(vx, vy, omega)
            va = np.zeros(ndof)
            va[:4] = tv
            for _ in range(5 * PHYSICS_HZ):  # 5 seconds
                if art.is_physics_handle_valid():
                    art.set_joint_velocity_targets(va.reshape(1, -1))
                world.step(render=False)

            p1, yaw1, _, _ = get_pose(stage, chassis_path)

            # Stop and coast 1s
            if art.is_physics_handle_valid():
                art.set_joint_velocity_targets(np.zeros((1, ndof)))
            for _ in range(PHYSICS_HZ):
                world.step(render=False)
            p2, _, _, _ = get_pose(stage, chassis_path)

            dx = (p1[0] - p0[0]) / IN
            dy = (p1[1] - p0[1]) / IN
            dyaw = yaw1 - yaw0
            dist = math.sqrt(dx*dx + dy*dy)
            heading = math.degrees(math.atan2(dy, dx)) if dist > 0.1 else 0
            coast = np.linalg.norm(p2[:2] - p1[:2]) / IN

            if dvx != 0 and dvy == 0 and dw == 0:
                exp_head = 0 if dvx > 0 else 180
                drift = abs(dy / dx * 100) if abs(dx) > 0.1 else 0
            elif dvy != 0 and dvx == 0 and dw == 0:
                exp_head = 90 if dvy > 0 else -90
                drift = abs(dx / dy * 100) if abs(dy) > 0.1 else 0
            elif dw != 0:
                exp_head = 0
                drift = dist
            else:
                exp_head = math.degrees(math.atan2(dvy, dvx))
                drift = 0

            head_err = heading - exp_head if dist > 0.5 else 0

            print(f"{name:<20} {spd:6.2f} {dx:+7.1f} {dy:+7.1f} {dyaw:+7.1f} "
                  f"{drift:7.1f} {head_err:+8.1f} {coast:6.2f}", flush=True)

    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# GUI: interactive manual testing with per-second logging
import carb.input
import omni.appwindow

class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.orbit = 0
        self.orbit_radius = 1.0
        self.reset = self.pstate = self.stop = False
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

kb = KB()
spawn_pos = [0, 0, spawn_z]
print("\nControls:", flush=True)
print("  Arrows=drive Q/E=rotate +/-=speed O=orbit 9/0=orbit radius", flush=True)
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
        if art.is_physics_handle_valid():
            art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
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
            if s["jv"] is not None:
                v = s["jv"].flatten()[:4]
                print(f"  Wheel vel: [{','.join(f'{x:.2f}' for x in v)}]", flush=True)
            if s["je"] is not None:
                e = s["je"].flatten()[:4]
                print(f"  Wheel torque: [{','.join(f'{x:.2f}' for x in e)}] Nm", flush=True)
            print(f"  Cmd: ({vx:.2f},{vy:.2f},{w:.2f})", flush=True)

    if frame % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            jv_str = ""
            if s["jv"] is not None:
                v = s["jv"].flatten()[:4]
                jv_str = f" wv=[{','.join(f'{x:.1f}' for x in v)}]"
            je_str = ""
            if s["je"] is not None:
                e = s["je"].flatten()[:4]
                je_str = f" τ=[{','.join(f'{x:.2f}' for x in e)}]"
            orb_str = ""
            if kb.orbit != 0:
                orb_str = f" orb={'CCW' if kb.orbit>0 else 'CW'} r={kb.orbit_radius/IN:.1f}\""
            print(f"[{frame/PHYSICS_HZ:.0f}s] ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                  f"ypr=({s['yaw']:.1f},{s['pitch']:.1f},{s['roll']:.1f}) "
                  f"cmd=({vx:.2f},{vy:.2f},{w:.2f}){orb_str}{jv_str}{je_str}", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
