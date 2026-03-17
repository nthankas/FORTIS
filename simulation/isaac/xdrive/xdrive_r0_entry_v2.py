"""
FORTIS R0 port entry maneuver.

Robot drives through the 15.75" tunnel, over the vessel lip, and descends
the interior slope on tether to the reactor floor. Chassis has a trapezoidal
arch on the underbelly (straight cuts only, weldable) for lip clearance.

NOTE: The PID tether model here is force-at-a-point, which doesn't properly
model a real cable. Needs rework to distance-constraint + winch model.

Usage: IsaacSim\\python.bat xdrive_r0_entry_v2.py --gui
       IsaacSim\\python.bat xdrive_r0_entry_v2.py --gui --mass 30
"""
import os, sys, math, argparse, time
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--mass", type=float, default=40.0, help="Robot mass in lbs (30/40/50)")
parser.add_argument("--arch-height", type=float, default=2.0, help="Arch height in inches")
parser.add_argument("--mode", choices=["manual", "pid", "auto"], default="manual")
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
LBS_TO_KG = 0.453592

CHASSIS_L = 15.0 * IN
CHASSIS_W = 9.0 * IN
CHASSIS_H = 6.0 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)

WHEEL_RADIUS = 90 * MM
WHEEL_WIDTH = 2.5 * IN
WHEEL_MASS = 1.0
CHASSIS_MASS = cli.mass * LBS_TO_KG
WHEEL_DROP = 2.0 * IN

# Trapezoidal arch on underbelly -- straight cuts only for weldability
ARCH_HEIGHT = cli.arch_height * IN
MOTOR_MOUNT_LEN = 2.5 * IN   # flat area at front/rear for motor mounting
ARCH_FLAT_WIDTH = 3.0 * IN   # flat section at the raised center

FRICTION_MU = 0.5
OMNI_ANGLE = 90.0
DRIVE_SPEED = 0.1
ROTATE_SPEED = 0.3

ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 3
SPHERE_RADIUS = 10 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS
ROW_OFFSET_DEG = 15.0
PHYSICS_HZ = 240

SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

# Top octagon vertices (same as before)
OCT_TOP = [
    ( SL,       -(SW - C)),  ( SL,        (SW - C)),
    ( (SL - C),  SW      ),  (-(SL - C),  SW      ),
    (-SL,        (SW - C)),  (-SL,       -(SW - C)),
    (-(SL - C), -SW      ),  ( (SL - C), -SW      ),
]

def mid(a, b): return ((a[0]+b[0])/2, (a[1]+b[1])/2)
WHEEL_XY = {
    "FR": mid(OCT_TOP[0], OCT_TOP[7]), "FL": mid(OCT_TOP[1], OCT_TOP[2]),
    "BL": mid(OCT_TOP[3], OCT_TOP[4]), "BR": mid(OCT_TOP[5], OCT_TOP[6]),
}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]

REACTOR_SIM_USD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "diiid_reactor.usd")
R0_PORT_Z = -1.7 * IN
TETHER_ANCHOR_Y = -145.0 * IN


def build_arched_chassis_mesh(stage, path, half_h):
    """Chassis mesh with trapezoidal arch on the bottom for lip clearance.
    Profile: flat motor mounts at front/rear, angled ramps, raised flat center."""
    # Define X stations along the chassis length for the bottom profile
    # Symmetric about X=0
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half  # where ramp meets center flat
    ramp_end = SL - MOTOR_MOUNT_LEN  # where ramp meets motor mount flat

    # Bottom Z profile (relative to chassis center, negative = below center)
    # Motor mount areas: at -half_h (original bottom)
    # Center arch: at -half_h + ARCH_HEIGHT (raised)
    z_low = -half_h
    z_high = -half_h + ARCH_HEIGHT

    # X stations for the bottom profile (front to back)
    # Each station gets a full width cross-section
    x_stations = sorted(set([
        -SL, -(SL - C),        # rear octagon corners
        -ramp_end,              # rear ramp start (motor mount edge)
        -ramp_start,            # rear ramp end (arch center edge)
        0.0,                    # center
        ramp_start,             # front ramp end (arch center edge)
        ramp_end,               # front ramp start (motor mount edge)
        (SL - C), SL,          # front octagon corners
    ]))

    def bottom_z(x):
        ax = abs(x)
        if ax <= ramp_start:
            return z_high  # flat raised center
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)  # linear ramp
        else:
            return z_low  # motor mount area

    def half_width_at(x):
        ax = abs(x)
        if ax <= (SL - C):
            return SW  # full width
        else:
            t = (ax - (SL - C)) / C
            return SW - t * C  # chamfer taper

    verts = []
    face_indices = []
    face_counts = []

    # For each X station, create top and bottom vertex pairs across the width
    # Top vertices at +half_h, bottom at bottom_z(x)
    # Width tapers at the chamfer corners
    sections = []
    for x in x_stations:
        hw = half_width_at(x)
        bz = bottom_z(x)
        # Each section: 4 vertices (top-left, top-right, bottom-right, bottom-left)
        tl = Gf.Vec3f(x, hw, half_h)
        tr = Gf.Vec3f(x, -hw, half_h)
        br = Gf.Vec3f(x, -hw, bz)
        bl = Gf.Vec3f(x, hw, bz)
        idx_start = len(verts)
        verts.extend([tl, tr, br, bl])
        sections.append((idx_start, idx_start + 1, idx_start + 2, idx_start + 3))

    # Side quads between adjacent sections
    for i in range(len(sections) - 1):
        tl0, tr0, br0, bl0 = sections[i]
        tl1, tr1, br1, bl1 = sections[i + 1]
        # Top face
        face_indices.extend([tl0, tl1, tr1, tr0])
        face_counts.append(4)
        # Bottom face
        face_indices.extend([bl0, br0, br1, bl1])
        face_counts.append(4)
        # Left side (positive Y)
        face_indices.extend([tl0, bl0, bl1, tl1])
        face_counts.append(4)
        # Right side (negative Y)
        face_indices.extend([tr0, tr1, br1, br0])
        face_counts.append(4)

    # End caps (front and rear)
    # Front cap (last section)
    tl, tr, br, bl = sections[-1]
    face_indices.extend([tl, tr, br, bl])
    face_counts.append(4)
    # Rear cap (first section)
    tl, tr, br, bl = sections[0]
    face_indices.extend([tr, tl, bl, br])
    face_counts.append(4)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.25, 0.25, 0.35)]))
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


def load_reactor(stage):
    rp = stage.DefinePrim("/World/Reactor", "Xform")
    rp.GetReferences().AddReference(REACTOR_SIM_USD)


def build_sphere_roller_wheel(stage, wp, wheel_mat, color):
    half_w = WHEEL_WIDTH / 2.0
    for row in range(2):
        ao = ROW_OFFSET_DEG * row
        ys = -half_w if row == 0 else 0.0
        ye = 0.0 if row == 0 else half_w
        ysp = (ye - ys) / (SPHERES_PER_ROLLER + 1)
        for ri in range(ROLLERS_PER_ROW):
            ra = math.radians(360.0 * ri / ROLLERS_PER_ROW + ao)
            rx, rz = SPHERE_CENTER_R * math.cos(ra), SPHERE_CENTER_R * math.sin(ra)
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
    vis.GetRadiusAttr().Set(WHEEL_RADIUS * 0.95)
    vis.GetHeightAttr().Set(WHEEL_WIDTH * 0.9)
    vis.GetAxisAttr().Set("Y")
    vis.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def build_robot(stage, spawn_pos, spawn_yaw=0.0):
    robot_path = "/World/Robot"
    chassis_path = robot_path + "/Chassis"
    UsdGeom.Xform.Define(stage, robot_path)
    UsdGeom.Xform.Define(stage, chassis_path)

    cp = stage.GetPrimAtPath(chassis_path)
    UsdPhysics.RigidBodyAPI.Apply(cp)
    ma = UsdPhysics.MassAPI.Apply(cp)
    ma.CreateMassAttr(CHASSIS_MASS)
    ma.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(cp)
    PhysxSchema.PhysxArticulationAPI.Apply(cp)

    half_h = CHASSIS_H / 2.0
    body_mesh = build_arched_chassis_mesh(stage, chassis_path + "/body", half_h)
    # SDF collision -- convexHull would fill in the arch concavity
    UsdPhysics.CollisionAPI.Apply(body_mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(body_mesh.GetPrim()).GetApproximationAttr().Set("sdf")

    # Forward arrow
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

    wheel_z = -WHEEL_DROP
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

    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos))
    if spawn_yaw != 0.0:
        rxf.AddRotateZOp().Set(spawn_yaw)

    underbelly_edge = WHEEL_DROP + WHEEL_RADIUS - half_h
    underbelly_center = underbelly_edge + ARCH_HEIGHT
    print(f"  Chassis: {CHASSIS_L/IN:.0f}x{CHASSIS_W/IN:.0f}x{CHASSIS_H/IN:.0f}\" "
          f"{CHASSIS_MASS/LBS_TO_KG:.0f}lbs", flush=True)
    print(f"  Wheel drop: {WHEEL_DROP/IN:.1f}\"", flush=True)
    print(f"  Underbelly edge: {underbelly_edge/IN:.2f}\"", flush=True)
    print(f"  Underbelly center (arch): {underbelly_center/IN:.2f}\"", flush=True)
    print(f"  Arch height: {ARCH_HEIGHT/IN:.1f}\", flat center: {ARCH_FLAT_WIDTH/IN:.1f}\", "
          f"motor mount: {MOTOR_MOUNT_LEN/IN:.1f}\"", flush=True)

    return robot_path, chassis_path


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
    r00, r10, r20, r21, r22 = mat[0][0], mat[1][0], mat[2][0], mat[2][1], mat[2][2]
    yaw = math.degrees(math.atan2(r10, r00))
    pitch = math.degrees(math.atan2(-r20, math.sqrt(r21**2 + r22**2)))
    roll = math.degrees(math.atan2(r21, r22))
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos": np.array([t[0], t[1], t[2]]), "yaw": yaw, "pitch": pitch,
            "roll": roll, "jv": jv, "je": je}


class PIDTether:
    """PID-controlled tether tension. NOTE: this is force-at-a-point, not a real
    cable constraint. A real cable can't push or go slack while bearing load."""
    def __init__(self, anchor, mass_kg):
        self.anchor = np.array(anchor)
        self.mass = mass_kg
        self.weight = mass_kg * 9.81
        # PID for descent rate control
        self.kp = 500.0
        self.ki = 50.0
        self.kd = 100.0
        self.target_vz = 0.0  # target Z velocity (negative = descending)
        self.integral = 0.0
        self.prev_error = 0.0
        self.tension = 0.0
        self.mode = "manual"  # "manual", "pid", "auto"
        self.manual_tension = 50.0
        self.prev_z = None
        self.physx_iface = None
        self.auto_phase = 0
        self.auto_timer = 0

    def initialize(self):
        try:
            from omni.physx import get_physx_interface
            self.physx_iface = get_physx_interface()
        except: pass

    def set_target_vz(self, vz):
        self.target_vz = vz

    def update_pid(self, current_z, dt):
        if self.prev_z is None:
            self.prev_z = current_z
            return self.weight * 0.5  # start at half weight support

        current_vz = (current_z - self.prev_z) / dt
        self.prev_z = current_z

        error = self.target_vz - current_vz
        self.integral += error * dt
        self.integral = max(-50, min(50, self.integral))  # anti-windup
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        # PID output is tension force
        tension = self.kp * error + self.ki * self.integral + self.kd * derivative
        # Clamp: can't push (negative tension), max = 2x weight
        tension = max(0, min(self.weight * 2.0, tension))
        return tension

    def auto_step(self, state, dt):
        """Automated entry phases. Needs proper cable model to work right."""
        self.auto_timer += dt
        z = state["pos"][2]
        pitch = state["pitch"]

        if self.auto_phase == 0:
            # Phase 0: Drive forward at port level
            self.target_vz = 0.0  # maintain height
            if self.auto_timer > 2.0:
                self.auto_phase = 1
                self.auto_timer = 0
            return 0.1, 0, 0  # forward
        elif self.auto_phase == 1:
            # Phase 1: Approaching lip, slow controlled descent
            self.target_vz = -0.02  # 2 cm/s descent
            return 0.05, 0, 0  # slow forward
        elif self.auto_phase == 2:
            # Phase 2: On slope, controlled descent
            self.target_vz = -0.05  # 5 cm/s descent
            return 0.02, 0, 0  # very slow forward
        return 0, 0, 0

    def apply(self, stage, chassis_path, dt=1.0/240.0):
        if self.physx_iface is None:
            return

        prim = stage.GetPrimAtPath(chassis_path)
        if not prim.IsValid(): return

        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = mat.ExtractTranslation()
        pos = np.array([t[0], t[1], t[2]])

        # Tether attaches to rear
        rear_world = np.array([
            mat[0][0]*(-SL) + t[0],
            mat[1][0]*(-SL) + t[1],
            mat[2][0]*(-SL) + t[2],
        ])

        if self.mode == "manual":
            self.tension = self.manual_tension
        elif self.mode in ("pid", "auto"):
            self.tension = self.update_pid(pos[2], dt)

        if self.tension < 0.1:
            return

        # Force direction: from rear toward anchor
        diff = self.anchor - rear_world
        dist = np.linalg.norm(diff)
        if dist < 0.01: return
        direction = diff / dist
        force = direction * self.tension

        try:
            self.physx_iface.apply_force_at_pos(
                chassis_path,
                carb.Float3(float(force[0]), float(force[1]), float(force[2])),
                carb.Float3(float(rear_world[0]), float(rear_world[1]), float(rear_world[2]))
            )
        except: pass


class KB:
    def __init__(self, tether):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.tether = tether
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
                self.speed = min(self.speed + 0.02, 0.5)
                print(f"Speed: {self.speed:.2f} m/s")
            elif k == K.MINUS:
                self.speed = max(self.speed - 0.02, 0.02)
                print(f"Speed: {self.speed:.2f} m/s")
            elif k == K.T:
                self.tether.manual_tension = min(self.tether.manual_tension + 10, 400)
                print(f"Tether: {self.tether.manual_tension:.0f}N ({self.tether.manual_tension/9.81:.1f}kg)")
            elif k == K.G:
                self.tether.manual_tension = max(self.tether.manual_tension - 10, 0)
                print(f"Tether: {self.tether.manual_tension:.0f}N ({self.tether.manual_tension/9.81:.1f}kg)")
            elif k == K.KEY_1:
                self.tether.mode = "manual"
                print("Mode: MANUAL tether")
            elif k == K.KEY_2:
                self.tether.mode = "pid"
                self.tether.target_vz = -0.02
                self.tether.integral = 0
                print(f"Mode: PID (target Vz={self.tether.target_vz:.3f} m/s)")
            elif k == K.KEY_3:
                self.tether.mode = "auto"
                self.tether.auto_phase = 0
                self.tether.auto_timer = 0
                self.tether.integral = 0
                print("Mode: AUTO sequence")
            elif k == K.KEY_8:
                self.tether.target_vz = max(self.tether.target_vz - 0.01, -0.2)
                print(f"Target Vz: {self.tether.target_vz:.3f} m/s")
            elif k == K.KEY_9:
                self.tether.target_vz = min(self.tether.target_vz + 0.01, 0.0)
                print(f"Target Vz: {self.tether.target_vz:.3f} m/s")
            elif k == K.KEY_0:
                self.tether.manual_tension = 0
                self.tether.mode = "manual"
                print("Tether OFF")
            elif k == K.C:
                global active_cam_idx
                active_cam_idx = (active_cam_idx + 1) % len(CAM_NAMES)
                set_active_camera(active_cam_idx)
        elif ev.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys.discard(k)
        return True

    def cmd(self):
        K = carb.input.KeyboardInput
        if self.tether.mode == "auto":
            s = get_state(art, stage, chassis_path)
            if s:
                return self.tether.auto_step(s, 1.0/PHYSICS_HZ)
            return 0, 0, 0
        vx = self.speed * (int(K.UP in self._keys) - int(K.DOWN in self._keys))
        vy = self.speed * (int(K.LEFT in self._keys) - int(K.RIGHT in self._keys))
        w = self.rspeed * (int(K.Q in self._keys) - int(K.E in self._keys))
        return vx, vy, w


# ========== MAIN ==========
print("=" * 60, flush=True)
print("FORTIS X-Drive -- R0 Entry (Arched Chassis + PID Tether)", flush=True)
print("=" * 60, flush=True)
print(f"Mass: {cli.mass:.0f} lbs ({CHASSIS_MASS:.1f} kg)", flush=True)
print(f"Arch height: {cli.arch_height:.1f}\"", flush=True)
print(f"Mode: {cli.mode}", flush=True)

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
build_physics_scene(stage)

dome = stage.DefinePrim("/World/DomeLight", "DomeLight")
dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)
for i, (rx, ry, rz) in enumerate([(45,0,0),(-45,0,0),(0,45,0),(0,-45,0)]):
    dl = stage.DefinePrim(f"/World/DirLight{i}", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(300.0)
    dlxf = UsdGeom.Xformable(dl)
    dlxf.ClearXformOpOrder()
    dlxf.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))

# Cameras: front follow, side follow, overhead
for cam_name, cam_offset, cam_target_offset in [
    ("FrontCam",  (0.5, 0, 0.15),  (0, 0, 0)),      # in front, looking back
    ("SideCam",   (0, 0.5, 0.1),   (0, 0, 0)),       # from the side
    ("TopCam",    (0, 0, 1.0),     (0, 0, 0)),        # overhead
    ("RearCam",   (-0.5, 0, 0.15), (0, 0, 0)),        # behind, looking forward
]:
    cam = UsdGeom.Camera.Define(stage, f"/World/{cam_name}")
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    cam.GetFocalLengthAttr().Set(18.0)

active_cam_idx = 0
CAM_NAMES = ["FrontCam", "SideCam", "TopCam", "RearCam"]
CAM_OFFSETS = [
    np.array([0.5, 0, 0.15]),    # front
    np.array([0, 0.5, 0.1]),     # side
    np.array([0, 0, 1.0]),       # top
    np.array([-0.5, 0, 0.15]),   # rear
]

def update_follow_camera(stage, chassis_path, cam_idx):
    s = get_state(None, stage, chassis_path) if stage.GetPrimAtPath(chassis_path).IsValid() else None
    if s is None:
        return
    pos = s["pos"]
    yaw_rad = math.radians(s["yaw"])

    offset = CAM_OFFSETS[cam_idx]
    # Rotate offset by chassis yaw
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    world_offset = np.array([
        cos_y * offset[0] - sin_y * offset[1],
        sin_y * offset[0] + cos_y * offset[1],
        offset[2]
    ])
    cam_pos = pos + world_offset

    cam_path = f"/World/{CAM_NAMES[cam_idx]}"
    cam_prim = stage.GetPrimAtPath(cam_path)
    if not cam_prim.IsValid():
        return

    # Point camera at robot center
    dx = pos[0] - cam_pos[0]
    dy = pos[1] - cam_pos[1]
    dz = pos[2] - cam_pos[2]
    dist_xy = math.sqrt(dx*dx + dy*dy)
    cam_yaw = math.degrees(math.atan2(dy, dx))
    cam_pitch = math.degrees(math.atan2(-dz, dist_xy))

    cxf = UsdGeom.Xformable(cam_prim)
    cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(cam_pos[0], cam_pos[1], cam_pos[2]))
    cxf.AddRotateZOp().Set(cam_yaw + 90)  # USD camera faces -Z by default
    cxf.AddRotateXOp().Set(cam_pitch + 90)

def set_active_camera(cam_idx):
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        vp.set_active_camera(f"/World/{CAM_NAMES[cam_idx]}")
        print(f"Camera: {CAM_NAMES[cam_idx]}", flush=True)
    except Exception as e:
        print(f"Camera switch failed: {e}", flush=True)

print("\nLoading reactor...", flush=True)
load_reactor(stage)
for _ in range(30): app.update()

spawn_x = 0.0
spawn_y = -140.0 * IN
spawn_z = R0_PORT_Z + WHEEL_DROP + WHEEL_RADIUS + 0.02
spawn_yaw = 90.0
spawn_pos = np.array([spawn_x, spawn_y, spawn_z])

print(f"\nSpawn: ({spawn_x/IN:.1f}\", {spawn_y/IN:.1f}\", {spawn_z/IN:.1f}\")", flush=True)
robot_path, chassis_path = build_robot(stage, spawn_pos, spawn_yaw)

for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(20): world.step(render=not headless)

ndof = 0
if art.is_physics_handle_valid():
    ndof = art.num_dof
    print(f"Articulation: {ndof} DOFs", flush=True)
    art.switch_control_mode("velocity", joint_indices=np.arange(ndof))

tether = PIDTether(
    anchor=np.array([spawn_x, TETHER_ANCHOR_Y, R0_PORT_Z]),
    mass_kg=CHASSIS_MASS + 4 * WHEEL_MASS
)
tether.initialize()
tether.mode = cli.mode
print(f"Tether: anchor at Y={TETHER_ANCHOR_Y/IN:.0f}\", mode={tether.mode}", flush=True)

print("Settling 2s...", flush=True)
for _ in range(2 * PHYSICS_HZ):
    tether.apply(stage, chassis_path)
    world.step(render=not headless)

s = get_state(art, stage, chassis_path)
if s:
    p = s["pos"]
    print(f"  Settled: ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
          f"pitch={s['pitch']:.1f} roll={s['roll']:.1f}", flush=True)

if headless:
    print("\nHeadless auto entry test...", flush=True)
    tether.mode = "pid"
    tether.target_vz = 0.0
    tv = xdrive_ik(0.1, 0, 0)
    va = np.zeros(ndof)
    va[:4] = tv
    for i in range(10 * PHYSICS_HZ):
        if art.is_physics_handle_valid():
            art.set_joint_velocity_targets(va.reshape(1, -1))
        tether.apply(stage, chassis_path)
        world.step(render=False)
        if (i+1) % PHYSICS_HZ == 0:
            s = get_state(art, stage, chassis_path)
            if s:
                p = s["pos"]
                je_peak = 0
                if s["je"] is not None:
                    je_peak = max(abs(x) for x in s["je"].flatten()[:4])
                print(f"  t={i//PHYSICS_HZ+1}s ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                      f"pitch={s['pitch']:.1f} tether={tether.tension:.0f}N "
                      f"peak_τ={je_peak:.1f}Nm", flush=True)
    omni.timeline.get_timeline_interface().stop()
    app.close()
    sys.exit(0)

# Activate front follow camera
update_follow_camera(stage, chassis_path, 0)
set_active_camera(0)

# GUI
kb = KB(tether)
print("\nControls:", flush=True)
print("  Arrows = drive  |  Q/E = rotate  |  +/- = speed", flush=True)
print("  T/G = tether tension +/-  |  0 = tether off", flush=True)
print("  1 = manual mode  |  2 = PID mode  |  3 = auto mode", flush=True)
print("  8/9 = PID target descent rate -/+", flush=True)
print("  C = cycle camera (front/side/top/rear)", flush=True)
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
        rxf.AddRotateZOp().Set(spawn_yaw)
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(20): world.step(render=True)
        if art.is_physics_handle_valid():
            art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
        tether.prev_z = None
        tether.integral = 0
        print("RESET", flush=True)
        continue

    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    va[:4] = tv
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))

    tether.apply(stage, chassis_path)
    world.step(render=True)
    frame += 1

    # Update follow camera every frame
    update_follow_camera(stage, chassis_path, active_cam_idx)

    if kb.pstate:
        kb.pstate = False
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            print(f"\n--- Frame {frame} ---", flush=True)
            print(f"  Pos: ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\")", flush=True)
            print(f"  YPR: ({s['yaw']:.1f},{s['pitch']:.1f},{s['roll']:.1f})", flush=True)
            print(f"  Tether: {tether.tension:.0f}N mode={tether.mode}", flush=True)
            if s["jv"] is not None:
                v = s["jv"].flatten()[:4]
                print(f"  Wheel vel: [{','.join(f'{x:.2f}' for x in v)}]", flush=True)
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
                je_str = f" peak_τ={max(abs(x) for x in e):.1f}Nm"
            mode_str = f" [{tether.mode}]"
            if tether.mode == "pid":
                mode_str += f" Vz_tgt={tether.target_vz:.3f}"
            print(f"[{frame/PHYSICS_HZ:.0f}s] ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
                  f"pitch={s['pitch']:.1f} roll={s['roll']:.1f} "
                  f"tether={tether.tension:.0f}N{mode_str}{je_str} "
                  f"cmd=({vx:.2f},{vy:.2f},{w:.2f})", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
