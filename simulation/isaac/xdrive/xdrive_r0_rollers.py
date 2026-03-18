"""
FORTIS R0 entry — fully automatic belly roller test.

Robot drives forward at constant speed. Tether acts like a winch with a
speed governor: maintains light tension to stay taut, brakes hard if
descent rate exceeds the limit. Heading auto-corrects via yaw PID.
No manual intervention needed — just watch.

Usage: IsaacSim\\python.bat xdrive_r0_rollers.py --gui
"""
import os, sys, math, argparse, json
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--mass", type=float, default=45.0)
parser.add_argument("--drop", type=float, default=2.0)
parser.add_argument("--speed", type=float, default=0.05, help="Forward speed m/s")
parser.add_argument("--max-descent", type=float, default=0.03, help="Max descent rate m/s")
cli, _ = parser.parse_known_args()
headless = cli.headless or not cli.gui

from isaacsim import SimulationApp
app = SimulationApp({"headless": headless, "width": 1920, "height": 1080})

import omni.usd, omni.timeline
import carb.input, omni.appwindow
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

IN = 0.0254
MM = 0.001

CHASSIS_L = 15.0 * IN
CHASSIS_W = 9.0 * IN
CHASSIS_H = 5.5 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
CHASSIS_MASS = cli.mass * 0.453592

WHEEL_RADIUS = 4.0 * IN
WHEEL_WIDTH = 2.04 * IN
WHEEL_MASS = 1.0
WHEEL_DROP = cli.drop * IN

MOTOR_MOUNT_LEN = 1.5 * IN
ARCH_HEIGHT = 2.5 * IN
ARCH_FLAT_WIDTH = 5.0 * IN

ROLLER_OD = 1.25 * IN
ROLLER_RADIUS = ROLLER_OD / 2.0
ROLLER_LENGTH = 7.5 * IN
ROLLER_SPACING = 1.375 * IN
ROLLER_MASS = 0.1
ROLLER_PROTRUSION = 0.25 * IN
BRACKET_WIDTH = 0.5 * IN
BRACKET_THICKNESS = 0.125 * IN

SPHERE_RADIUS = 12.7 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS
ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 3
ROW_OFFSET_DEG = 15.0

FRICTION_MU = 0.5
OMNI_ANGLE = 90.0
PHYSICS_HZ = 240

REACTOR_SIM_USD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "diiid_reactor.usd")
R0_PORT_Z = -1.7 * IN
TETHER_ANCHOR_Y = -145.0 * IN

SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT
half_h = CHASSIS_H / 2.0

OCT_XY = [
    (SL, -(SW-C)), (SL, (SW-C)), ((SL-C), SW), (-(SL-C), SW),
    (-SL, (SW-C)), (-SL, -(SW-C)), (-(SL-C), -SW), ((SL-C), -SW),
]
def mid(a, b): return ((a[0]+b[0])/2, (a[1]+b[1])/2)
WHEEL_XY = {
    "FR": mid(OCT_XY[0], OCT_XY[7]), "FL": mid(OCT_XY[1], OCT_XY[2]),
    "BL": mid(OCT_XY[3], OCT_XY[4]), "BR": mid(OCT_XY[5], OCT_XY[6]),
}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]


# ==================== BUILD ====================

def build_arched_chassis(stage, path):
    z_low, z_high = -half_h, -half_h + ARCH_HEIGHT
    rs, re = ARCH_FLAT_WIDTH/2.0, SL - MOTOR_MOUNT_LEN
    xs = sorted(set([-SL, -(SL-C), -re, -rs, 0.0, rs, re, (SL-C), SL]))
    def bz(x):
        ax = abs(x)
        if ax <= rs: return z_high
        elif ax <= re: return z_high + (ax-rs)/(re-rs)*(z_low-z_high)
        else: return z_low
    def hw(x):
        ax = abs(x)
        return SW if ax <= (SL-C) else SW - (ax-(SL-C))/C*C
    verts, fi, fc, secs = [], [], [], []
    for x in xs:
        w, b = hw(x), bz(x)
        i = len(verts)
        verts.extend([Gf.Vec3f(x,w,half_h), Gf.Vec3f(x,-w,half_h),
                      Gf.Vec3f(x,-w,b), Gf.Vec3f(x,w,b)])
        secs.append((i,i+1,i+2,i+3))
    for i in range(len(secs)-1):
        a,b = secs[i], secs[i+1]
        for q in [(a[0],b[0],b[1],a[1]),(a[3],a[2],b[2],b[3]),
                  (a[0],a[3],b[3],b[0]),(a[1],b[1],b[2],a[2])]:
            fi.extend(q); fc.append(4)
    a=secs[-1]; fi.extend([a[0],a[1],a[2],a[3]]); fc.append(4)
    a=secs[0]; fi.extend([a[1],a[0],a[3],a[2]]); fc.append(4)
    m = UsdGeom.Mesh.Define(stage, path)
    m.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    m.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    m.GetSubdivisionSchemeAttr().Set("none")
    m.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.25,0.25,0.35)]))
    return m

def build_sphere_wheel(stage, wp, wmat, color):
    hw = WHEEL_WIDTH / 2.0
    for row in range(2):
        ao = ROW_OFFSET_DEG * row
        ys = -hw if row == 0 else 0.0
        ye = 0.0 if row == 0 else hw
        ysp = (ye-ys)/(SPHERES_PER_ROLLER+1)
        for ri in range(ROLLERS_PER_ROW):
            ra = math.radians(360.0*ri/ROLLERS_PER_ROW + ao)
            rx, rz = SPHERE_CENTER_R*math.cos(ra), SPHERE_CENTER_R*math.sin(ra)
            for si in range(SPHERES_PER_ROLLER):
                sy = ys + ysp*(si+1)
                s = UsdGeom.Sphere.Define(stage, f"{wp}/r{row}_{ri}_{si}")
                s.GetRadiusAttr().Set(SPHERE_RADIUS)
                sx = UsdGeom.Xformable(s.GetPrim()); sx.ClearXformOpOrder()
                sx.AddTranslateOp().Set(Gf.Vec3d(rx, sy, rz))
                UsdPhysics.CollisionAPI.Apply(s.GetPrim())
                UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(
                    wmat, UsdShade.Tokens.weakerThanDescendants, "physics")
    v = UsdGeom.Cylinder.Define(stage, wp+"/vis")
    v.GetRadiusAttr().Set(WHEEL_RADIUS*0.92); v.GetHeightAttr().Set(WHEEL_WIDTH*0.85)
    v.GetAxisAttr().Set("Y")
    v.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))

def build_robot(stage, spawn_pos, spawn_yaw):
    rp, cp = "/World/Robot", "/World/Robot/Chassis"
    UsdGeom.Xform.Define(stage, rp); UsdGeom.Xform.Define(stage, cp)
    p = stage.GetPrimAtPath(cp)
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(CHASSIS_MASS)
    UsdPhysics.MassAPI(p).CreateCenterOfMassAttr(Gf.Vec3f(0,0,0))
    UsdPhysics.ArticulationRootAPI.Apply(p)
    PhysxSchema.PhysxArticulationAPI.Apply(p)

    body = build_arched_chassis(stage, cp+"/body")
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(body.GetPrim()).GetApproximationAttr().Set("convexDecomposition")

    ar = UsdGeom.Cube.Define(stage, cp+"/fwd"); ar.GetSizeAttr().Set(1.0)
    ax = UsdGeom.Xformable(ar.GetPrim()); ax.ClearXformOpOrder()
    ax.AddTranslateOp().Set(Gf.Vec3d(SL*0.6,0,half_h+0.003))
    ax.AddScaleOp().Set(Gf.Vec3d(SL*0.4,SW*0.15,0.005))
    ar.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9,0.15,0.15)]))

    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wp = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wp.CreateStaticFrictionAttr(FRICTION_MU); wp.CreateDynamicFrictionAttr(FRICTION_MU)
    wp.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    rmat = UsdShade.Material.Define(stage, "/World/RollerMat")
    rp2 = UsdPhysics.MaterialAPI.Apply(rmat.GetPrim())
    rp2.CreateStaticFrictionAttr(FRICTION_MU); rp2.CreateDynamicFrictionAttr(FRICTION_MU)
    rp2.CreateRestitutionAttr(0.05)
    roller_mat = UsdShade.Material(stage.GetPrimAtPath("/World/RollerMat"))

    wcol = {"FR":(0.85,0.2,0.2),"FL":(0.2,0.75,0.2),"BL":(0.2,0.2,0.85),"BR":(0.85,0.75,0.2)}
    wz = -WHEEL_DROP; wo = WHEEL_WIDTH/2.0+0.003
    for wn in WHEEL_ORDER:
        cx,cy = WHEEL_XY[wn]; ad = AXLE_ANGLES[wn]; arad = math.radians(ad)
        nl = math.sqrt(cx*cx+cy*cy); wx,wy = cx+cx/nl*wo, cy+cy/nl*wo
        wpp = rp+f"/Wheel_{wn}"
        UsdGeom.Xform.Define(stage, wpp)
        wpr = stage.GetPrimAtPath(wpp)
        UsdPhysics.RigidBodyAPI.Apply(wpr)
        UsdPhysics.MassAPI.Apply(wpr).CreateMassAttr(WHEEL_MASS)
        wxf = UsdGeom.Xformable(wpr); wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx,wy,wz)); wxf.AddRotateZOp().Set(ad)
        build_sphere_wheel(stage, wpp, wheel_mat, wcol[wn])
        jp = rp+f"/Joint_{wn}"
        j = UsdPhysics.RevoluteJoint.Define(stage, jp)
        j.GetBody0Rel().SetTargets([Sdf.Path(cp)]); j.GetBody1Rel().SetTargets([Sdf.Path(wpp)])
        j.GetLocalPos0Attr().Set(Gf.Vec3f(wx,wy,wz)); j.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        j.GetAxisAttr().Set("Y")
        ha = arad/2.0
        j.GetLocalRot0Attr().Set(Gf.Quatf(math.cos(ha),0,0,math.sin(ha)))
        j.GetLocalRot1Attr().Set(Gf.Quatf(1,0,0,0))
        d = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        d.GetStiffnessAttr().Set(0.0); d.GetDampingAttr().Set(100.0)
        d.GetMaxForceAttr().Set(200.0); d.GetTargetVelocityAttr().Set(0.0)
        jprim = stage.GetPrimAtPath(jp)
        jprim.CreateAttribute("isaacmecanumwheel:radius",Sdf.ValueTypeNames.Float).Set(float(WHEEL_RADIUS))
        jprim.CreateAttribute("isaacmecanumwheel:angle",Sdf.ValueTypeNames.Float).Set(float(OMNI_ANGLE))

    # Belly rollers
    z_lo, z_hi = -half_h, -half_h+ARCH_HEIGHT
    rs2, re2 = ARCH_FLAT_WIDTH/2.0, SL-MOTOR_MOUNT_LEN
    def az(x):
        ax=abs(x)
        if ax<=rs2: return z_hi
        elif ax<=re2: return z_hi+(ax-rs2)/(re2-rs2)*(z_lo-z_hi)
        else: return z_lo
    nr = max(1,int(2*re2/ROLLER_SPACING))
    if nr%2==0: nr-=1
    rxs = [(i-nr//2)*ROLLER_SPACING for i in range(nr)]
    bhy = ROLLER_LENGTH/2.0+BRACKET_WIDTH/2.0
    for ri, rx in enumerate(rxs):
        laz=az(rx); rcz=laz-ROLLER_PROTRUSION; bd=ROLLER_PROTRUSION+ROLLER_RADIUS
        rlp = rp+f"/Roller_{ri}"
        UsdGeom.Xform.Define(stage, rlp)
        rlpr = stage.GetPrimAtPath(rlp)
        UsdPhysics.RigidBodyAPI.Apply(rlpr)
        UsdPhysics.MassAPI.Apply(rlpr).CreateMassAttr(ROLLER_MASS)
        rxf = UsdGeom.Xformable(rlpr); rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(rx,0,rcz))
        cy = UsdGeom.Cylinder.Define(stage, rlp+"/cyl")
        cy.GetRadiusAttr().Set(ROLLER_RADIUS); cy.GetHeightAttr().Set(ROLLER_LENGTH)
        cy.GetAxisAttr().Set("Y")
        cy.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.75,0.75,0.3)]))
        UsdPhysics.CollisionAPI.Apply(cy.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cy.GetPrim()).Bind(
            roller_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        for side, sy in [("L",bhy),("R",-bhy)]:
            br = UsdGeom.Cube.Define(stage, cp+f"/brk_{side}_{ri}")
            br.GetSizeAttr().Set(1.0)
            bx = UsdGeom.Xformable(br.GetPrim()); bx.ClearXformOpOrder()
            bx.AddTranslateOp().Set(Gf.Vec3d(rx,sy,laz-bd/2.0))
            bx.AddScaleOp().Set(Gf.Vec3d(BRACKET_WIDTH,BRACKET_THICKNESS,bd))
            br.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.4,0.4,0.4)]))
        jrp = rp+f"/RJ_{ri}"
        rj = UsdPhysics.RevoluteJoint.Define(stage, jrp)
        rj.GetBody0Rel().SetTargets([Sdf.Path(cp)]); rj.GetBody1Rel().SetTargets([Sdf.Path(rlp)])
        rj.GetLocalPos0Attr().Set(Gf.Vec3f(rx,0,rcz)); rj.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        rj.GetAxisAttr().Set("Y")
        rj.GetLocalRot0Attr().Set(Gf.Quatf(1,0,0,0)); rj.GetLocalRot1Attr().Set(Gf.Quatf(1,0,0,0))
        rd = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jrp), "angular")
        rd.GetStiffnessAttr().Set(0.0); rd.GetDampingAttr().Set(0.05); rd.GetMaxForceAttr().Set(0.0)

    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(rp)); rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos))
    if spawn_yaw != 0: rxf.AddRotateZOp().Set(spawn_yaw)
    print(f"  Robot: {CHASSIS_L/IN:.0f}x{CHASSIS_W/IN:.0f}x{CHASSIS_H/IN:.1f}\" "
          f"{cli.mass:.0f}lbs drop={WHEEL_DROP/IN:.1f}\" rollers={nr}", flush=True)
    return rp, cp, nr


# ==================== IK + STABILIZATION ====================

def xdrive_ik(vx, vy, omega):
    r = WHEEL_RADIUS
    vels = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        vwx = vx - omega*wy; vwy = vy + omega*wx
        a = math.radians(AXLE_ANGLES[wn])
        vels.append((vwx*math.cos(a) + vwy*math.sin(a)) / r)
    return np.array(vels)

class YawStabilizer:
    """Manual drive with automatic yaw correction.
    You control forward/strafe with arrow keys. Any yaw deviation from
    the locked heading is corrected by the motors automatically."""
    def __init__(self):
        self.target_yaw = None
        self.yaw_kp = 4.0
        self.speed = 0.1
        self.cmd_fwd = 0.0  # set by keyboard
        self.cmd_lat = 0.0

    def update(self, state):
        if state is None:
            return xdrive_ik(0, 0, 0)
        if self.target_yaw is None:
            self.target_yaw = state["yaw"]

        yaw_err = state["yaw"] - self.target_yaw
        while yaw_err > 180: yaw_err -= 360
        while yaw_err < -180: yaw_err += 360
        omega = -yaw_err * self.yaw_kp * (math.pi/180)
        omega = max(-1.5, min(1.5, omega))

        vx = self.cmd_fwd * self.speed
        vy = self.cmd_lat * self.speed
        return xdrive_ik(vx, vy, omega)


# ==================== TETHER (winch governor) ====================

class WinchTether:
    """Purely reactive tether. Zero tension while the robot is fine.
    Tension scales with how fast the robot is actually falling.
    No hardcoded weight fractions, no base tension, no assumptions.

    descent_speed = 0       -> tension = 0 (driving level, cable slack)
    descent_speed < max     -> tension = proportional (gentle braking)
    descent_speed > max     -> tension = heavy braking (catch it)

    This naturally handles every phase:
    - Horizontal drive: vz~0, tension~0, cable trails behind
    - Tipping over lip: vz starts negative, tension ramps up smoothly
    - Controlled descent: tension holds vz near the max rate
    - Landing: vz goes to 0, tension drops to 0
    """
    def __init__(self, anchor, max_descent_rate):
        self.anchor = np.array(anchor)
        self.max_rate = max_descent_rate
        self.tension = 0.0
        self.prev_z = None
        self.vz = 0.0
        self.physx_iface = None

    def initialize(self):
        try:
            from omni.physx import get_physx_interface
            self.physx_iface = get_physx_interface()
        except: pass

    def apply(self, stage, chassis_path, dt=1.0/240.0):
        if self.physx_iface is None: return
        prim = stage.GetPrimAtPath(chassis_path)
        if not prim.IsValid(): return
        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = mat.ExtractTranslation()

        if self.prev_z is None:
            self.prev_z = t[2]
            self.tension = 0
            return

        self.vz = (t[2] - self.prev_z) / dt
        self.prev_z = t[2]

        # Only respond to downward motion
        descent_speed = max(0, -self.vz)  # positive when falling

        if descent_speed < 0.001:
            # Not falling -- zero tension
            self.tension = 0
        elif descent_speed <= self.max_rate:
            # Falling within limits -- light proportional tension to slow it gently
            self.tension = 200.0 * (descent_speed / self.max_rate)
        else:
            # Falling too fast -- heavy braking, scales with overspeed
            overspeed = descent_speed - self.max_rate
            self.tension = 200.0 + 5000.0 * overspeed

        self.tension = max(0, min(3000, self.tension))

        if self.tension < 0.1: return

        rear = np.array([mat[0][0]*(-SL)+t[0], mat[1][0]*(-SL)+t[1], mat[2][0]*(-SL)+t[2]])
        diff = self.anchor - rear
        dist = np.linalg.norm(diff)
        if dist < 0.01: return
        force = (diff/dist) * self.tension

        try:
            self.physx_iface.apply_force_at_pos(
                chassis_path,
                carb.Float3(float(force[0]),float(force[1]),float(force[2])),
                carb.Float3(float(rear[0]),float(rear[1]),float(rear[2])))
        except: pass

    def reset(self):
        self.prev_z = None; self.vz = 0; self.tension = 0


# ==================== STATE ====================

def get_state(art, stage, cp):
    prim = stage.GetPrimAtPath(cp)
    if not prim.IsValid(): return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
    pitch = math.degrees(math.atan2(-mat[2][0], math.sqrt(mat[2][1]**2+mat[2][2]**2)))
    roll = math.degrees(math.atan2(mat[2][1], mat[2][2]))
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos":np.array([t[0],t[1],t[2]]), "yaw":yaw, "pitch":pitch, "roll":roll, "jv":jv, "je":je}


# ==================== CAMERAS ====================





# ==================== MAIN ====================

print("="*70, flush=True)
print("FORTIS R0 Entry — Auto Drive + Winch Governor Tether", flush=True)
print("="*70, flush=True)
print(f"Mass: {cli.mass:.0f}lbs | Speed: {cli.speed:.2f} m/s | "
      f"Max descent: {cli.max_descent:.3f} m/s ({cli.max_descent/IN:.1f}\"/s)", flush=True)

ctx = omni.usd.get_context(); ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
ps.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); ps.CreateGravityMagnitudeAttr(9.81)
px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
px.CreateEnableCCDAttr(True); px.CreateEnableStabilizationAttr(True)
px.CreateSolverTypeAttr("TGS"); px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)

dome = stage.DefinePrim("/World/DomeLight","DomeLight")
dome.CreateAttribute("inputs:intensity",Sdf.ValueTypeNames.Float).Set(1000.0)
for i,(rx,ry,rz) in enumerate([(45,0,0),(-45,0,0),(0,45,0),(0,-45,0)]):
    dl = stage.DefinePrim(f"/World/DL{i}","DistantLight")
    dl.CreateAttribute("inputs:intensity",Sdf.ValueTypeNames.Float).Set(300.0)
    dx = UsdGeom.Xformable(dl); dx.ClearXformOpOrder()
    dx.AddRotateXYZOp().Set(Gf.Vec3f(rx,ry,rz))


print("\nLoading reactor...", flush=True)
rp = stage.DefinePrim("/World/Reactor","Xform")
rp.GetReferences().AddReference(REACTOR_SIM_USD)
for _ in range(30): app.update()

spawn_x, spawn_y = 0.0, -140.0 * IN
spawn_z = R0_PORT_Z + WHEEL_DROP + WHEEL_RADIUS + 0.02
spawn_yaw = 90.0
spawn_pos = np.array([spawn_x, spawn_y, spawn_z])
print(f"Spawn: ({spawn_x/IN:.1f}\",{spawn_y/IN:.1f}\",{spawn_z/IN:.1f}\")", flush=True)

robot_path, chassis_path, n_rollers = build_robot(stage, spawn_pos, spawn_yaw)
for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path); art.initialize()
for _ in range(10): world.step(render=not headless)

ndof = 0
if art.is_physics_handle_valid():
    ndof = art.num_dof
    print(f"Articulation: {ndof} DOFs", flush=True)
    art.switch_control_mode("velocity", joint_indices=np.arange(4))
else:
    print("FATAL"); app.close(); sys.exit(1)

driver = YawStabilizer()
tether = WinchTether(
    anchor=np.array([spawn_x, TETHER_ANCHOR_Y, R0_PORT_Z]),
    max_descent_rate=cli.max_descent)
tether.initialize()

print(f"Tether: reactive, max_descent={tether.max_rate*1000:.0f}mm/s, "
      f"zero tension while level", flush=True)

print("Settling 3s...", flush=True)
for _ in range(3*PHYSICS_HZ):
    world.step(render=not headless)

s = get_state(art, stage, chassis_path)
if s:
    driver.target_yaw = s["yaw"]
    print(f"  Settled: yaw={s['yaw']:.1f} pitch={s['pitch']:.1f}", flush=True)

log_data = []

class KB:
    def __init__(self):
        self.reset_flag = self.pstate = self.stop = False
        self._keys = set()
        self._inp = carb.input.acquire_input_interface()
        self._kb = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = self._inp.subscribe_to_keyboard_events(self._kb, self._on)
    def _on(self, ev, *a, **kw):
        k = ev.input; K = carb.input.KeyboardInput
        if ev.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._keys.add(k)
            if k == K.R: self.reset_flag = True
            elif k == K.P: self.pstate = True
            elif k == K.SPACE: self.stop = True
            elif k == K.EQUAL:
                driver.speed = min(driver.speed+0.02, 0.5)
                print(f"Speed: {driver.speed:.2f} m/s")
            elif k == K.MINUS:
                driver.speed = max(driver.speed-0.02, 0.02)
                print(f"Speed: {driver.speed:.2f} m/s")
        elif ev.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys.discard(k)
        return True
    def update_cmd(self):
        K = carb.input.KeyboardInput
        driver.cmd_fwd = float(int(K.UP in self._keys) - int(K.DOWN in self._keys))
        driver.cmd_lat = float(int(K.LEFT in self._keys) - int(K.RIGHT in self._keys))

kb = KB()
print("\nControls: Arrows=drive (yaw auto-corrects), +/-=speed, C=cam, R=reset, P=state, Space=stop", flush=True)

frame = 0
while app.is_running():
    kb.update_cmd()

    if kb.stop:
        kb.stop = False
        driver.cmd_fwd = driver.cmd_lat = 0

    if kb.reset_flag:
        kb.reset_flag = False
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5): app.update()
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path)); rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos)); rxf.AddRotateZOp().Set(spawn_yaw)
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path); art.initialize()
        for _ in range(10): world.step(render=True)
        art.switch_control_mode("velocity", joint_indices=np.arange(4))
        tether.reset(); driver.target_yaw = None; log_data.clear()
        print("RESET", flush=True); continue

    s = get_state(art, stage, chassis_path)
    tv = driver.update(s)
    va = np.zeros(ndof); va[:4] = tv
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1,-1))

    # No tether -- pure geometry test of belly rollers vs lip
    world.step(render=True)
    frame += 1

    if kb.pstate:
        kb.pstate = False
        if s:
            p=s["pos"]; r=math.sqrt(p[0]**2+p[1]**2)
            rv_cnt = 0
            if s["jv"] is not None and len(s["jv"].flatten())>4:
                rv_cnt = sum(1 for v in s["jv"].flatten()[4:4+n_rollers] if abs(v)>0.5)
            print(f"\n  Pos:({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") r={r/IN:.1f}\"", flush=True)
            print(f"  YPR:({s['yaw']:.1f},{s['pitch']:.1f},{s['roll']:.1f})", flush=True)
            print(f"  Tether:{tether.tension:.0f}N vz={tether.vz:.3f} rollers={rv_cnt}/{n_rollers}", flush=True)

    if frame % PHYSICS_HZ == 0 and s:
        p=s["pos"]; r=math.sqrt(p[0]**2+p[1]**2)
        je_pk = max(abs(x) for x in s["je"].flatten()[:4]) if s["je"] is not None else 0
        rv_cnt = 0
        if s["jv"] is not None and len(s["jv"].flatten())>4:
            rv_cnt = sum(1 for v in s["jv"].flatten()[4:4+n_rollers] if abs(v)>0.5)
        log_data.append({"t":frame/PHYSICS_HZ,"x":p[0]/IN,"y":p[1]/IN,"z":p[2]/IN,
            "r":r/IN,"pitch":s["pitch"],"roll":s["roll"],"yaw":s["yaw"],
            "tether_N":tether.tension,"vz":tether.vz,"torque":je_pk,"rollers":rv_cnt})
        st = f"spd={driver.speed:.2f}"
        print(f"[{frame/PHYSICS_HZ:.0f}s] ({p[0]/IN:.1f}\",{p[1]/IN:.1f}\",{p[2]/IN:.1f}\") "
              f"r={r/IN:.1f}\" pitch={s['pitch']:.1f} roll={s['roll']:.1f} "
              f"tether={tether.tension:.0f}N vz={tether.vz:.3f} "
              f"τ={je_pk:.1f}Nm rollers={rv_cnt} [{st}]", flush=True)

if log_data:
    lp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "r0_entry_log.json")
    with open(lp,"w") as f: json.dump(log_data,f,indent=2)
    print(f"\nLog: {lp} ({len(log_data)} entries)", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
