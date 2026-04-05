"""
FORTIS X-drive inside the DIII-D reactor.

Same robot build as xdrive_o3dyn.py, spawned on the outer floor near the step.
Reactor geometry loaded from diiid_reactor.usd (triangle mesh collision
applied via Physics API Editor in GUI -- don't regenerate that file).

Usage: IsaacSim\\python.bat xdrive_reactor.py --gui
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

import omni.usd
import omni.timeline
import carb.input
import omni.appwindow
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "lib"))
import sim_config as cfg

IN = 0.0254
MM = 0.001

# Robot geometry -- keep in sync with xdrive_o3dyn.py
CHASSIS_L = 15.0 * IN   # front-to-back (X axis)
CHASSIS_W = 9.0 * IN    # side-to-side (Y axis)
CHASSIS_H = 6.0 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)

WHEEL_RADIUS = 90 * MM
WHEEL_WIDTH = 2.5 * IN
WHEEL_MASS = 1.0
CHASSIS_MASS = 18.1

FRICTION_MU = 0.5
OMNI_ANGLE = 90.0
DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5

ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 3
SPHERE_RADIUS = 10 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS
ROW_OFFSET_DEG = 15.0
PHYSICS_HZ = 240

SL = CHASSIS_L / 2.0   # half-length (X)
SW = CHASSIS_W / 2.0   # half-width (Y)
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

# Reactor USD is 1:1 real-world scale (meters). No scaling needed.

def build_octagonal_prism(stage, path, half_h, color):
    n = len(OCT_XY)
    verts = []
    for x, y in OCT_XY:
        verts.append(Gf.Vec3f(x, y, -half_h))
    for x, y in OCT_XY:
        verts.append(Gf.Vec3f(x, y, half_h))
    face_indices, face_counts = [], []
    for i in range(n):
        j = (i + 1) % n
        face_indices.extend([i, j, j + n, i + n])
        face_counts.append(4)
    face_indices.extend(list(range(n - 1, -1, -1)))
    face_counts.append(n)
    face_indices.extend(list(range(n, 2 * n)))
    face_counts.append(n)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def build_sphere_roller_wheel(stage, wheel_path, wheel_mat, axle_angle_deg, color):
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
                sp = f"{wheel_path}/row{row}_r{ri}_s{si}"
                sphere = UsdGeom.Sphere.Define(stage, sp)
                sphere.GetRadiusAttr().Set(SPHERE_RADIUS)
                sxf = UsdGeom.Xformable(sphere.GetPrim())
                sxf.ClearXformOpOrder()
                sxf.AddTranslateOp().Set(Gf.Vec3d(rx, sy, rz))
                UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
                UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
                    wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    vis = UsdGeom.Cylinder.Define(stage, wheel_path + "/visual")
    vis.GetRadiusAttr().Set(WHEEL_RADIUS * 0.95)
    vis.GetHeightAttr().Set(WHEEL_WIDTH * 0.9)
    vis.GetAxisAttr().Set("Y")
    vis.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def build_physics_scene(stage):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateSolverTypeAttr("TGS")
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)


REACTOR_SIM_USD = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "assets", "diiid_reactor.usd")


def load_reactor(stage):
    """Load reactor USD. Has triangle mesh collision + graphite friction baked in."""
    rp = stage.DefinePrim("/World/Reactor", "Xform")
    rp.GetReferences().AddReference(REACTOR_SIM_USD)
    print(f"  Reactor loaded: {REACTOR_SIM_USD}", flush=True)


def build_robot(stage, spawn_pos, spawn_yaw=0.0):
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

    half_h = CHASSIS_H / 2.0
    chassis_mesh = build_octagonal_prism(stage, chassis_path + "/body", half_h, (0.25, 0.25, 0.35))
    UsdPhysics.CollisionAPI.Apply(chassis_mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(chassis_mesh.GetPrim()).GetApproximationAttr().Set("convexHull")

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

    wheel_colors = {
        "FR": (0.85, 0.2, 0.2), "FL": (0.2, 0.75, 0.2),
        "BL": (0.2, 0.2, 0.85), "BR": (0.85, 0.75, 0.2),
    }

    wheel_z = 0.0
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.003
    joint_paths = []

    for wname in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wname]
        angle_deg = AXLE_ANGLES[wname]
        angle_rad = math.radians(angle_deg)
        norm_len = math.sqrt(cx * cx + cy * cy)
        nx, ny = cx / norm_len, cy / norm_len
        wx = cx + nx * wheel_offset
        wy = cy + ny * wheel_offset

        wp = robot_path + f"/Wheel_{wname}"
        UsdGeom.Xform.Define(stage, wp)
        wprim = stage.GetPrimAtPath(wp)
        UsdPhysics.RigidBodyAPI.Apply(wprim)
        UsdPhysics.MassAPI.Apply(wprim).CreateMassAttr(WHEEL_MASS)
        wxf = UsdGeom.Xformable(wprim)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(angle_deg)

        build_sphere_roller_wheel(stage, wp, wheel_mat, angle_deg, wheel_colors[wname])

        jp = robot_path + f"/Joint_{wname}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.GetBody0Rel().SetTargets([Sdf.Path(chassis_path)])
        joint.GetBody1Rel().SetTargets([Sdf.Path(wp)])
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(wx, wy, wheel_z))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.GetAxisAttr().Set("Y")
        half_a = angle_rad / 2.0
        joint.GetLocalRot0Attr().Set(Gf.Quatf(math.cos(half_a), 0, 0, math.sin(half_a)))
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

    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(spawn_pos[0], spawn_pos[1], spawn_pos[2]))
    if spawn_yaw != 0.0:
        rxf.AddRotateZOp().Set(spawn_yaw)

    print(f"  Robot at ({spawn_pos[0]/IN:.1f}\", {spawn_pos[1]/IN:.1f}\", {spawn_pos[2]/IN:.1f}\") "
          f"yaw={spawn_yaw:.1f}deg", flush=True)
    return robot_path, chassis_path, joint_paths


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
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos": np.array([t[0], t[1], t[2]]), "jv": jv, "je": je, "mat": mat}


def print_state(art, stage, chassis_path, frame):
    s = get_state(art, stage, chassis_path)
    if not s: return
    p = s["pos"]
    mat = s["mat"]
    yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
    pitch = math.degrees(math.atan2(-mat[2][0], math.sqrt(mat[2][1]**2 + mat[2][2]**2)))
    roll = math.degrees(math.atan2(mat[2][1], mat[2][2]))
    r_from_origin = math.sqrt(p[0]**2 + p[1]**2)
    print(f"\n--- Frame {frame} ---", flush=True)
    print(f"  Pos: ({p[0]/IN:.1f}\", {p[1]/IN:.1f}\", {p[2]/IN:.1f}\")", flush=True)
    print(f"  R from origin: {r_from_origin/IN:.1f}\"", flush=True)
    print(f"  YPR: ({yaw:.1f}, {pitch:.1f}, {roll:.1f}) deg", flush=True)
    if s["jv"] is not None:
        v = s["jv"].flatten()[:4]
        print(f"  Wheel vel: [{', '.join(f'{x:.2f}' for x in v)}] rad/s", flush=True)
    if s["je"] is not None:
        e = s["je"].flatten()[:4]
        print(f"  Wheel torque: [{', '.join(f'{x:.3f}' for x in e)}] Nm", flush=True)


class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.orbit_radius = cfg.STEP_R_IN * IN  # default: reactor step radius
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
                print(f"Orbit: {labels[self.orbit]}, radius={self.orbit_radius/IN:.1f}\"")
            elif k == K.KEY_9:
                self.orbit_radius = max(self.orbit_radius - 0.05, 0.1)
                print(f"Orbit radius: {self.orbit_radius/IN:.1f}\"")
            elif k == K.KEY_0:
                self.orbit_radius = min(self.orbit_radius + 0.05, 3.0)
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
print("FORTIS X-Drive -- Reactor Environment", flush=True)
print("=" * 60, flush=True)

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_physics_scene(stage)

# Bright even lighting -- need to see inside the reactor vessel
dome = stage.DefinePrim("/World/DomeLight", "DomeLight")
dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)
dome.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))

for i, (rx, ry, rz) in enumerate([(45, 0, 0), (-45, 0, 0), (0, 45, 0), (0, -45, 0)]):
    dl = stage.DefinePrim(f"/World/DirLight{i}", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(300.0)
    dl.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    dlxf = UsdGeom.Xformable(dl)
    dlxf.ClearXformOpOrder()
    dlxf.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))

print("\nLoading reactor (SDF collision)...", flush=True)
load_reactor(stage)

for _ in range(30): app.update()

# Spawn on outer floor near the step. Coordinates are approximate -- tweak in GUI.
SPAWN_X = 0.0      # aligned with R0 center
SPAWN_Y = -1.5916  # radial position toward R0
SPAWN_FLOOR_Z = -1.3175
spawn_z = SPAWN_FLOOR_Z + WHEEL_RADIUS + 0.15
spawn_pos = np.array([SPAWN_X, SPAWN_Y, spawn_z])

# Yaw: face the center column (origin direction)
spawn_yaw = math.degrees(math.atan2(0 - SPAWN_Y, 0 - SPAWN_X))

r_from_origin = math.sqrt(SPAWN_X**2 + SPAWN_Y**2)
print(f"\nSpawn: ({SPAWN_X:.3f}m, {SPAWN_Y:.3f}m, {spawn_z:.3f}m)", flush=True)
print(f"  = ({SPAWN_X/IN:.1f}\", {SPAWN_Y/IN:.1f}\", {spawn_z/IN:.1f}\")", flush=True)
print(f"  R from origin: {r_from_origin:.2f}m ({r_from_origin/IN:.1f}\")", flush=True)
print(f"  Facing center: yaw={spawn_yaw:.1f}deg", flush=True)

robot_path, chassis_path, joint_paths = build_robot(stage, spawn_pos, spawn_yaw)

for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10): world.step(render=not headless)

if art.is_physics_handle_valid():
    ndof = art.num_dof
    print(f"Articulation: {ndof} DOFs, names={art.dof_names}", flush=True)
    art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
else:
    print("FATAL: Articulation invalid", flush=True)
    app.close()
    sys.exit(1)

print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    world.step(render=not headless)
    if (i+1) % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"  t={i//PHYSICS_HZ+1}s Z={p[2]/IN:.2f}\"", flush=True)

print_state(art, stage, chassis_path, 0)

if headless:
    print("\nHeadless: orbit CCW at 0.1 m/s for 5s...", flush=True)
    orbit_r = cfg.STEP_R_IN * IN
    for _ in range(5 * PHYSICS_HZ):
        vy = 0.1
        omega = vy / orbit_r
        tv = xdrive_ik(0, vy, omega)
        va = np.zeros(ndof); va[:4] = tv
        art.set_joint_velocity_targets(va.reshape(1, -1))
        world.step(render=False)
    print_state(art, stage, chassis_path, 5 * PHYSICS_HZ)
    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# GUI
kb = KB()
print(f"\nOrbit radius default: {kb.orbit_radius/IN:.1f}\" (reactor step radius)", flush=True)
print("\nControls:", flush=True)
print("  Arrows = translate  |  Q/E = rotate  |  +/- = speed", flush=True)
print("  O = toggle orbit (OFF/CCW/CW)  |  9/0 = decrease/increase orbit radius", flush=True)
print("  R = reset  |  P = print state  |  Space = stop", flush=True)

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
        if spawn_yaw != 0.0:
            rxf.AddRotateZOp().Set(spawn_yaw)
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(10): world.step(render=True)
        art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
        print("RESET", flush=True)
        continue

    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof); va[:4] = tv
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))
    world.step(render=True)
    frame += 1

    if kb.pstate:
        kb.pstate = False
        print_state(art, stage, chassis_path, frame)
        orbit_str = f"  Orbit: {'CCW' if kb.orbit>0 else 'CW' if kb.orbit<0 else 'OFF'}"
        if kb.orbit != 0:
            orbit_str += f" r={kb.orbit_radius/IN:.1f}\""
        print(f"  Cmd: vx={vx:.2f} vy={vy:.2f} w={w:.2f}", flush=True)
        print(orbit_str, flush=True)

    if frame % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            mat = s["mat"]
            yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
            r = math.sqrt(p[0]**2 + p[1]**2)
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
            print(f"[{frame/PHYSICS_HZ:.1f}s] pos=({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                  f"r={r/IN:.1f}\" yaw={yaw:.1f} "
                  f"cmd=({vx:.2f},{vy:.2f},{w:.2f}){orb_str}{jv_str}{je_str}", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
