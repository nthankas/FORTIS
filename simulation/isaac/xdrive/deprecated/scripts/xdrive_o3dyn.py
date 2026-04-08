"""
FORTIS X-drive flat ground simulation.

Omni wheel physics: each wheel is a rigid body with sphere colliders arranged
in two staggered rows matching the Rotacaster roller layout. Gaps between
rollers create directional friction -- full circumferential rings would just
act like a cylinder. 240Hz physics keeps the small spheres stable.

Usage: IsaacSim\\python.bat xdrive_o3dyn.py --gui
       IsaacSim\\python.bat xdrive_o3dyn.py --headless
"""
import os, sys, math, argparse
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--reactor", action="store_true")
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

CHASSIS_L = 15.0 * IN   # front-to-back (X axis)
CHASSIS_W = 9.0 * IN    # side-to-side (Y axis)
CHASSIS_H = 6.0 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)

WHEEL_RADIUS = 90 * MM
WHEEL_WIDTH = 2.205 * IN    # actual Rotacaster width (both rows)
WHEEL_MASS = 1.0
CHASSIS_MASS = 18.1

FRICTION_MU = 0.5
OMNI_ANGLE = 90.0

DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5

# Sphere rollers sized to Rotacaster R2-1801: 12 rollers/row, 1" barrel dia.
# Two rows staggered 15 deg so there's always ground contact.
ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 3
SPHERE_RADIUS = 12.7 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS  # 77.3mm
ROW_OFFSET_DEG = 15.0
PHYSICS_HZ = 240

# Rectangular octagon half-extents
SL = CHASSIS_L / 2.0   # half-length (X)
SW = CHASSIS_W / 2.0   # half-width (Y)
C = CHAMFER_CUT

# CCW vertex list starting at front-right edge
OCT_XY = [
    ( SL,       -(SW - C)),   # v0: front side, right end
    ( SL,        (SW - C)),   # v1: front side, left end
    ( (SL - C),  SW      ),   # v2: left side, front end
    (-(SL - C),  SW      ),   # v3: left side, back end
    (-SL,        (SW - C)),   # v4: back side, left end
    (-SL,       -(SW - C)),   # v5: back side, right end
    (-(SL - C), -SW      ),   # v6: right side, back end
    ( (SL - C), -SW      ),   # v7: right side, front end
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


def build_sphere_roller_wheel(stage, wheel_path, wheel_mat, axle_angle_deg, color):
    """Two staggered rows of sphere colliders mimicking the Rotacaster roller layout."""
    half_w = WHEEL_WIDTH / 2.0

    for row in range(2):
        angle_offset = ROW_OFFSET_DEG * row
        if row == 0:
            y_start = -half_w
            y_end = 0.0
        else:
            y_start = 0.0
            y_end = half_w
        row_width = y_end - y_start
        y_spacing = row_width / (SPHERES_PER_ROLLER + 1)

        for ri in range(ROLLERS_PER_ROW):
            roller_angle = math.radians(360.0 * ri / ROLLERS_PER_ROW + angle_offset)
            rx = SPHERE_CENTER_R * math.cos(roller_angle)
            rz = SPHERE_CENTER_R * math.sin(roller_angle)

            for si in range(SPHERES_PER_ROLLER):
                sy = y_start + y_spacing * (si + 1)

                sp_path = f"{wheel_path}/row{row}_r{ri}_s{si}"
                sphere = UsdGeom.Sphere.Define(stage, sp_path)
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

    STEP_HEIGHT = 4.5 * IN
    wheel_z = 0.0  # wheels in line with chassis center

    # Step straddling clearance check
    underbelly_flat = WHEEL_RADIUS - half_h  # 0.54"
    front_to_rear = 2.0 * abs(WHEEL_XY["FR"][0])  # 6.88"
    tilt_rad = math.atan2(STEP_HEIGHT, front_to_rear)
    mid_z = WHEEL_RADIUS + STEP_HEIGHT / 2.0
    chassis_bottom_at_step = mid_z - half_h * math.cos(tilt_rad)
    clearance_at_step = chassis_bottom_at_step - STEP_HEIGHT
    # If negative, chassis hits step by |clearance_at_step|
    drop_needed = -clearance_at_step if clearance_at_step < 0 else 0.0

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
        rot_q = Gf.Quatf(math.cos(half_a), 0, 0, math.sin(half_a))
        joint.GetLocalRot0Attr().Set(rot_q)
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

    spawn_z = WHEEL_RADIUS + 0.01
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))

    print(f"Chassis: {CHASSIS_L/IN:.0f}x{CHASSIS_W/IN:.0f}x{CHASSIS_H/IN:.0f}\" octagon, "
          f"{CHASSIS_MASS}kg", flush=True)
    total_spheres = 2 * ROLLERS_PER_ROW * SPHERES_PER_ROLLER
    print(f"Wheels: {WHEEL_RADIUS*1000:.0f}mm dia, {WHEEL_WIDTH/IN:.1f}\" wide, "
          f"2 rows x {ROLLERS_PER_ROW} rollers x {SPHERES_PER_ROLLER} spheres = "
          f"{total_spheres} spheres/wheel ({total_spheres*4} total)", flush=True)
    print(f"Sphere radius: {SPHERE_RADIUS*1000:.1f}mm, center at r={SPHERE_CENTER_R*1000:.1f}mm", flush=True)
    print(f"Physics: {PHYSICS_HZ}Hz, friction mu={FRICTION_MU}", flush=True)
    print(f"", flush=True)
    print(f"--- STEP STRADDLING CLEARANCE ---", flush=True)
    print(f"  Wheel radius:          {WHEEL_RADIUS/IN:.2f}\"  ({WHEEL_RADIUS*100:.1f}cm)", flush=True)
    print(f"  Chassis half-height:   {half_h/IN:.2f}\"  ({half_h*100:.1f}cm)", flush=True)
    print(f"  Wheelbase (F-R):       {front_to_rear/IN:.2f}\"  ({front_to_rear*100:.1f}cm)", flush=True)
    print(f"  Step height:           {STEP_HEIGHT/IN:.2f}\"  ({STEP_HEIGHT*100:.1f}cm)", flush=True)
    print(f"  Straddle tilt angle:   {math.degrees(tilt_rad):.1f} deg", flush=True)
    print(f"  Flat ground clearance: {underbelly_flat/IN:.2f}\"  ({underbelly_flat*100:.1f}cm)", flush=True)
    print(f"  Clearance at step:     {clearance_at_step/IN:.2f}\"  ({clearance_at_step*100:.1f}cm)", flush=True)
    if clearance_at_step < 0:
        print(f"  ** COLLISION: chassis hits step by {abs(clearance_at_step)/IN:.2f}\" ({abs(clearance_at_step)*100:.1f}cm)", flush=True)
        print(f"  ** To clear with 0.5\" margin, wheels must drop {(drop_needed + 0.5*IN)/IN:.2f}\" below chassis center", flush=True)
        print(f"     OR increase wheel diameter to {2*(half_h + STEP_HEIGHT/2 + half_h*(1-math.cos(tilt_rad)))/IN:.1f}\"", flush=True)
        print(f"     OR reduce chassis height to {2*(WHEEL_RADIUS + STEP_HEIGHT/2 - STEP_HEIGHT/math.cos(tilt_rad)*0 - STEP_HEIGHT)/IN:.1f}\"... (see tilt geometry)", flush=True)
    else:
        print(f"  OK: {clearance_at_step/IN:.2f}\" clearance above step edge", flush=True)
    print(f"", flush=True)

    return robot_path, chassis_path, joint_paths, spawn_z


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
    if s["jv"] is not None:
        v = s["jv"].flatten()[:4]
        print(f"  Wheel vel (rad/s): [{', '.join(f'{x:.2f}' for x in v)}]", flush=True)
    if s["je"] is not None:
        e = s["je"].flatten()[:4]
        print(f"  Wheel torque (Nm): [{', '.join(f'{x:.3f}' for x in e)}]", flush=True)


class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.orbit_radius = 1.0  # meters, radius for arc/orbit mode
        self.reset = self.pstate = self.stop = False
        self.orbit = 0  # 0=off, +1=orbit left (CCW), -1=orbit right (CW)
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
                # Cycle orbit: off -> CCW -> CW -> off
                self.orbit = (self.orbit + 2) % 3 - 1  # cycles -1, 0, 1
                labels = {-1: "CW", 0: "OFF", 1: "CCW"}
                print(f"Orbit: {labels[self.orbit]}, radius={self.orbit_radius:.2f}m ({self.orbit_radius/IN:.1f}\")")
            elif k == K.KEY_9:
                self.orbit_radius = max(self.orbit_radius - 0.1, 0.2)
                print(f"Orbit radius: {self.orbit_radius:.2f}m ({self.orbit_radius/IN:.1f}\")")
            elif k == K.KEY_0:
                self.orbit_radius = min(self.orbit_radius + 0.1, 5.0)
                print(f"Orbit radius: {self.orbit_radius:.2f}m ({self.orbit_radius/IN:.1f}\")")
        elif ev.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys.discard(k)
        return True

    def cmd(self):
        K = carb.input.KeyboardInput

        if self.orbit != 0:
            # Orbit: strafe + yaw to follow a circular arc
            vy = self.speed * self.orbit
            omega = vy / self.orbit_radius
            return 0.0, vy, omega

        vx = self.speed * (int(K.UP in self._keys) - int(K.DOWN in self._keys))
        vy = self.speed * (int(K.LEFT in self._keys) - int(K.RIGHT in self._keys))
        w = self.rspeed * (int(K.Q in self._keys) - int(K.E in self._keys))
        return vx, vy, w


# ========== MAIN ==========
print("=" * 60, flush=True)
print("FORTIS X-Drive (Sphere Roller Approximation)", flush=True)
print("=" * 60, flush=True)

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10):
    app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_scene(stage)
robot_path, chassis_path, joint_paths, spawn_z = build_robot(stage)

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

if art.is_physics_handle_valid():
    ndof = art.num_dof
    print(f"Articulation: {ndof} DOFs, names={art.dof_names}", flush=True)
    art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
else:
    print("FATAL: Articulation invalid", flush=True)
    app.close()
    sys.exit(1)

# Settle
print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    world.step(render=not headless)

print_state(art, stage, chassis_path, 0)

# Headless test
if headless:
    for test_name, tvx, tvy, tw in [
        ("forward", 0.2, 0, 0),
        ("strafe left", 0, 0.2, 0),
        ("strafe right", 0, -0.2, 0),
        ("rotate CW", 0, 0, -0.5),
        ("diagonal", 0.15, 0.15, 0),
    ]:
        print(f"\nHeadless: {test_name} for 2s...", flush=True)
        tv = xdrive_ik(tvx, tvy, tw)
        print(f"  IK vels: [{', '.join(f'{x:.2f}' for x in tv)}]", flush=True)
        va = np.zeros(ndof)
        va[:4] = tv
        for _ in range(2 * PHYSICS_HZ):
            art.set_joint_velocity_targets(va.reshape(1, -1))
            world.step(render=False)
        print_state(art, stage, chassis_path, 0)

        # Stop
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
        for _ in range(PHYSICS_HZ):
            world.step(render=False)

    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# GUI
kb = KB()
spawn_pos = [0, 0, spawn_z]
print("\nControls:", flush=True)
print("  Arrows = translate  |  Q/E = rotate  |  +/- = speed", flush=True)
print("  O = toggle orbit (OFF/CCW/CW)  |  9/0 = decrease/increase orbit radius", flush=True)
print("  R = reset  |  P = print state  |  Space = stop", flush=True)

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
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(10): world.step(render=True)
        art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
        print("RESET", flush=True)
        continue

    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    va[:4] = tv
    art.set_joint_velocity_targets(va.reshape(1, -1))
    world.step(render=True)
    frame += 1

    if kb.pstate:
        kb.pstate = False
        print_state(art, stage, chassis_path, frame)
        print(f"  Cmd: vx={vx:.2f} vy={vy:.2f} w={w:.2f}", flush=True)
        print(f"  IK: [{', '.join(f'{x:.2f}' for x in tv)}]", flush=True)

    if frame % PHYSICS_HZ == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            prim = stage.GetPrimAtPath(chassis_path)
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            r00, r01, r02 = mat[0][0], mat[0][1], mat[0][2]
            r10, r11, r12 = mat[1][0], mat[1][1], mat[1][2]
            r20, r21, r22 = mat[2][0], mat[2][1], mat[2][2]
            yaw = math.degrees(math.atan2(r10, r00))
            pitch = math.degrees(math.atan2(-r20, math.sqrt(r21**2 + r22**2)))
            roll = math.degrees(math.atan2(r21, r22))
            jv_str = ""
            if s["jv"] is not None:
                v = s["jv"].flatten()[:4]
                jv_str = f" wv=[{','.join(f'{x:.1f}' for x in v)}]"
            je_str = ""
            if s["je"] is not None:
                e = s["je"].flatten()[:4]
                je_str = f" τ=[{','.join(f'{x:.2f}' for x in e)}]"
            orbit_str = ""
            if kb.orbit != 0:
                orbit_str = f" orb={'CCW' if kb.orbit>0 else 'CW'} r={kb.orbit_radius/IN:.1f}\""
            print(f"[{frame/PHYSICS_HZ:.1f}s] pos=({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                  f"ypr=({yaw:.1f},{pitch:.1f},{roll:.1f}) "
                  f"cmd=({vx:.2f},{vy:.2f},{w:.2f}){orbit_str}{jv_str}{je_str}", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
