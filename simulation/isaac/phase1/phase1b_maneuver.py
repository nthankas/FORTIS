"""
Phase 1b Maneuver Optimizer: Find optimal robot config for reactor maneuver.

Systematically sweeps chassis dimensions, tire size, motor torque, and friction
to find configurations that can reliably:
  1. Rotate 90 degrees from radial to tangential while straddling the step
  2. Drive in a circle around the reactor

The robot enters through the R0 port facing the reactor center. It must
pivot 90 degrees (skid-steer) then drive along the circular floor.

Run single test:
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_maneuver.py
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_maneuver.py --gui

Run sweep (subprocess isolation per config):
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_maneuver.py --sweep
"""
import os, sys, math, json, argparse, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--sweep", action="store_true")
parser.add_argument("--tire_r", type=float, default=3.0)
parser.add_argument("--drive_max", type=float, default=30.0, help="Max wheel velocity rad/s")
parser.add_argument("--mu", type=float, default=0.7)
parser.add_argument("--weight", type=float, default=50.0)
parser.add_argument("--length", type=float, default=14.0)
parser.add_argument("--width", type=float, default=13.0)
parser.add_argument("--height", type=float, default=9.0)
parser.add_argument("--damping", type=float, default=10000.0, help="Drive damping (torque constant)")
parser.add_argument("--max_force", type=float, default=100000.0, help="Drive max force")
parser.add_argument("--result_file", type=str, default=None, help="Output file for subprocess mode")
cli, _ = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not cli.gui, "width": 1920, "height": 1080})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt

from fortis_config import (
    SCENE_REACTOR, RESULTS_DIR,
    Z_INNER, Z_OUTER,
    R_INNER_FLOOR_MIN, R_INNER_FLOOR_MAX,
    R_OUTER_FLOOR_MIN, R_OUTER_FLOOR_MAX,
    R_CENTER_POST, R_OUTER_WALL,
)

ES = 0.024
LBS_TO_KG = 0.453592
DT = 1.0 / 60.0

# -- Floor geometry constants --
# The smooth floor surfaces must be HIGH ENOUGH above the reactor mesh that
# wheels never contact the underlying mesh (which has an abrupt step).
# At r=56.2" the mesh outer floor is at Z=-49.3"; our ramp must be above that.
# With FLOOR_OFFSET=3.0" and ramp from 48-60", ramp_z(56.2) = -50.8 + 3.69 = -47.11
# which is 2.19" above the mesh outer floor. Safe.
FLOOR_OFFSET = 3.0  # inches above mesh floor

# The ramp must be wide enough that its slope < arctan(mu).
# At mu=0.3 (worst case), max slope = 16.7 deg.
# Step height = 4.5". Need radial width > 4.5/tan(16.7) = 15".
# Use 48" to 62" = 14" wide. Slope = 4.5/14 = 17.8 deg. Just over for mu=0.3.
# At mu=0.5: max slope = 26.6 deg. 4.5/14 = 17.8 deg. Well under.
RAMP_R_INNER = 48.0  # ramp starts here (inner floor ends here)
RAMP_R_OUTER = 62.0  # ramp ends here (outer floor starts here)


def i2w(inches):
    return inches * ES


# -- Scene Setup ---------------------------------------------------------------

def setup_physics(stage):
    ps = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(ps).IsValid():
        p = UsdPhysics.Scene.Define(stage, ps)
        p.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        p.CreateGravityMagnitudeAttr(9.81)
        px = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(ps))
        px.CreateEnableCCDAttr(True)
        px.CreateEnableStabilizationAttr(True)
        px.CreateSolverTypeAttr("TGS")


def make_floor_ring(stage, path, z_in, r_inner_in, r_outer_in, mu, segments=72):
    """Smooth annular collision ring at given Z height."""
    if stage.GetPrimAtPath(path).IsValid():
        return
    z = i2w(z_in)
    r_in, r_out = i2w(r_inner_in), i2w(r_outer_in)
    verts = []
    for i in range(segments + 1):
        a = 2 * math.pi * i / segments
        c, s = math.cos(a), math.sin(a)
        verts.append(Gf.Vec3f(r_in * c, r_in * s, z))
        verts.append(Gf.Vec3f(r_out * c, r_out * s, z))
    fi, fc = [], []
    for i in range(segments):
        j = i * 2
        fi.extend([j, j+1, j+3, j+2])  # CCW from above -> normal UP
        fc.append(4)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetVisibilityAttr().Set("invisible")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("none")
    mat = UsdShade.Material.Define(stage, path + "_Mat")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(mu))
    pm.CreateDynamicFrictionAttr(float(mu))
    pm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def make_step_ramp(stage, path, mu, segments=72, radial_steps=8):
    """Smooth ramp between inner and outer floor across the widened step zone.

    The ramp spans from RAMP_R_INNER to RAMP_R_OUTER (48"-62" = 14" wide).
    This gives a gentle slope of 4.5/14 = 17.8 degrees, well under arctan(mu=0.5)=26.6 deg.
    The 3" floor offset ensures the ramp is always ABOVE the reactor mesh,
    even where the mesh's outer floor starts at r=56.2" (Z=-49.3").
    """
    if stage.GetPrimAtPath(path).IsValid():
        return
    r_in = i2w(RAMP_R_INNER)
    r_out = i2w(RAMP_R_OUTER)
    z_in = i2w(Z_INNER + FLOOR_OFFSET)   # -53.8 + 3.0 = -50.8"
    z_out = i2w(Z_OUTER + FLOOR_OFFSET)  # -49.3 + 3.0 = -46.3"

    verts = []
    for i in range(segments + 1):
        a = 2 * math.pi * i / segments
        c, s = math.cos(a), math.sin(a)
        for j in range(radial_steps + 1):
            t = j / radial_steps
            r = r_in + t * (r_out - r_in)
            z = z_in + t * (z_out - z_in)
            verts.append(Gf.Vec3f(r * c, r * s, z))

    fi, fc = [], []
    cols = radial_steps + 1
    for i in range(segments):
        for j in range(radial_steps):
            v0 = i * cols + j
            v1 = i * cols + j + 1
            v2 = (i + 1) * cols + j + 1
            v3 = (i + 1) * cols + j
            fi.extend([v0, v1, v2, v3])
            fc.append(4)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetVisibilityAttr().Set("invisible")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("none")
    mat = UsdShade.Material.Define(stage, path + "_Mat")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(mu))
    pm.CreateDynamicFrictionAttr(float(mu))
    pm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    slope_deg = math.degrees(math.atan2(Z_OUTER - Z_INNER, RAMP_R_OUTER - RAMP_R_INNER))
    print(f"  Step ramp: r={RAMP_R_INNER}-{RAMP_R_OUTER}\" ({RAMP_R_OUTER-RAMP_R_INNER}\" wide), "
          f"Z={Z_INNER+FLOOR_OFFSET:.1f} to {Z_OUTER+FLOOR_OFFSET:.1f}\", "
          f"slope={slope_deg:.1f}deg")


def prepare_scene(stage, mu=0.5):
    """Add smooth floors + wide gentle step ramp, keep reactor mesh for walls."""
    # Inner floor: from center post region to where ramp starts
    make_floor_ring(stage, "/World/InnerFloor",
                    Z_INNER + FLOOR_OFFSET, R_INNER_FLOOR_MIN, RAMP_R_INNER, mu)
    # Outer floor: from where ramp ends to outer wall
    make_floor_ring(stage, "/World/OuterFloor",
                    Z_OUTER + FLOOR_OFFSET, RAMP_R_OUTER, R_OUTER_FLOOR_MAX, mu)
    # Wide gentle ramp across the step zone
    make_step_ramp(stage, "/World/StepRamp", mu)
    print(f"  Floors + ramp created (mu={mu}, offset={FLOOR_OFFSET}\")")
    print(f"    Inner floor: r={R_INNER_FLOOR_MIN}-{RAMP_R_INNER}\", Z={Z_INNER+FLOOR_OFFSET:.1f}\"")
    print(f"    Outer floor: r={RAMP_R_OUTER}-{R_OUTER_FLOOR_MAX}\", Z={Z_OUTER+FLOOR_OFFSET:.1f}\"")


# -- Robot Builder -------------------------------------------------------------

def floor_z_at_radius(r_inches):
    """Get smooth floor Z (inches) at given radius, accounting for ramp."""
    if r_inches <= RAMP_R_INNER:
        return Z_INNER + FLOOR_OFFSET
    elif r_inches >= RAMP_R_OUTER:
        return Z_OUTER + FLOOR_OFFSET
    else:
        t = (r_inches - RAMP_R_INNER) / (RAMP_R_OUTER - RAMP_R_INNER)
        return Z_INNER + FLOOR_OFFSET + t * (Z_OUTER - Z_INNER)


def build_robot(stage, cfg, azimuth_deg=0):
    """
    Build robot at step center, facing radially (towards reactor center).

    azimuth_deg: position around reactor (0 = on +X axis)
    Robot long axis points radially (towards center).
    """
    rp = "/World/Robot"
    if stage.GetPrimAtPath(rp).IsValid():
        stage.RemovePrim(Sdf.Path(rp))

    wr = i2w(cfg["tire_r"])
    hl = i2w(cfg["length"] / 2)
    hw = i2w(cfg["width"] / 2)
    hh = i2w(cfg["height"] / 2)
    whw = i2w(1.0)  # wheel half-width
    mass = cfg["weight"] * LBS_TO_KG
    damping = cfg.get("damping", 10000.0)
    max_force = cfg.get("max_force", 100000.0)

    # Robot center at step center (r=55"), facing radially
    step_r = (RAMP_R_INNER + RAMP_R_OUTER) / 2  # 55" (center of ramp zone)
    az = math.radians(azimuth_deg)
    cx = i2w(step_r) * math.cos(az)
    cy = i2w(step_r) * math.sin(az)

    # Floor Z at robot center
    center_floor_z = i2w(floor_z_at_radius(step_r))
    cz = center_floor_z + wr + hh + 0.005  # small gap for settling

    # Robot long axis (X) points radially toward center
    # At azimuth=0: radial direction is -X (toward center from +X side)
    # So robot heading = azimuth + pi (pointing inward)
    heading = az + math.pi
    half_h = heading / 2

    UsdGeom.Xform.Define(stage, rp)
    cp = f"{rp}/Chassis"
    verts = [Gf.Vec3f(-hl,-hw,-hh), Gf.Vec3f(hl,-hw,-hh),
             Gf.Vec3f(hl,hw,-hh),   Gf.Vec3f(-hl,hw,-hh),
             Gf.Vec3f(-hl,-hw,hh),  Gf.Vec3f(hl,-hw,hh),
             Gf.Vec3f(hl,hw,hh),    Gf.Vec3f(-hl,hw,hh)]
    fi = [0,1,2,3, 4,5,6,7, 0,4,5,1, 2,6,7,3, 0,3,7,4, 1,5,6,2]
    fc = [4,4,4,4,4,4]
    cm = UsdGeom.Mesh.Define(stage, cp)
    cm.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    cm.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    cm.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    cm.GetSubdivisionSchemeAttr().Set("none")

    xf = UsdGeom.Xformable(cm)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xf.AddOrientOp().Set(Gf.Quatf(
        float(math.cos(half_h)), 0, 0, float(math.sin(half_h))))

    cprim = cm.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(cprim)
    UsdPhysics.CollisionAPI.Apply(cprim)
    UsdPhysics.MeshCollisionAPI.Apply(cprim).GetApproximationAttr().Set("convexHull")
    UsdPhysics.MassAPI.Apply(cprim).CreateMassAttr(float(mass * 0.80))
    UsdPhysics.ArticulationRootAPI.Apply(cprim)
    PhysxSchema.PhysxArticulationAPI.Apply(cprim).CreateEnabledSelfCollisionsAttr(False)

    # Wheel material
    wmp = "/World/WheelMat"
    if not stage.GetPrimAtPath(wmp).IsValid():
        wm = UsdShade.Material.Define(stage, wmp)
        wpm = UsdPhysics.MaterialAPI.Apply(wm.GetPrim())
        wpm.CreateStaticFrictionAttr(float(cfg["mu"]))
        wpm.CreateDynamicFrictionAttr(float(cfg["mu"]))
        wpm.CreateRestitutionAttr(0.2)
    else:
        wpm = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(wmp))
        wpm.GetStaticFrictionAttr().Set(float(cfg["mu"]))
        wpm.GetDynamicFrictionAttr().Set(float(cfg["mu"]))
    wmat = UsdShade.Material(stage.GetPrimAtPath(wmp))

    wheels = [("FL", +hl, +hw), ("FR", +hl, -hw),
              ("RL", -hl, +hw), ("RR", -hl, -hw)]
    joint_names = []

    for name, dx, dy in wheels:
        wp = f"{rp}/{name}"
        wc = UsdGeom.Cylinder.Define(stage, wp)
        wc.GetRadiusAttr().Set(float(wr))
        wc.GetHeightAttr().Set(float(whw * 2))
        wc.GetAxisAttr().Set("Y")

        # Wheel position in world: rotate local offset by heading
        wx = cx + dx * math.cos(heading) - dy * math.sin(heading)
        wy = cy + dx * math.sin(heading) + dy * math.cos(heading)
        # Wheel Z: based on smooth floor height at this radius
        r_wheel = math.sqrt(wx**2 + wy**2) / ES
        wz_floor = i2w(floor_z_at_radius(r_wheel))
        wz = wz_floor + wr

        wxf = UsdGeom.Xformable(wc)
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
        wxf.AddOrientOp().Set(Gf.Quatf(
            float(math.cos(half_h)), 0, 0, float(math.sin(half_h))))

        wprim = wc.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(wprim)
        UsdPhysics.CollisionAPI.Apply(wprim)
        UsdPhysics.MassAPI.Apply(wprim).CreateMassAttr(float(mass * 0.20 / 4))
        UsdShade.MaterialBindingAPI.Apply(wprim).Bind(
            wmat, UsdShade.Tokens.weakerThanDescendants, "physics")
        pc = PhysxSchema.PhysxCollisionAPI.Apply(wprim)
        pc.CreateContactOffsetAttr(0.003)
        pc.CreateRestOffsetAttr(0.001)

        jn = f"{name}_joint"
        jp = f"{rp}/{jn}"
        j = UsdPhysics.RevoluteJoint.Define(stage, jp)
        j.CreateBody0Rel().SetTargets([cp])
        j.CreateBody1Rel().SetTargets([wp])
        j.CreateAxisAttr("Y")
        j.CreateLocalPos0Attr(Gf.Vec3f(float(dx), float(dy), float(-hh)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

        d = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        d.CreateTypeAttr("force")
        d.CreateTargetVelocityAttr(0.0)
        d.CreateDampingAttr(float(damping))
        d.CreateStiffnessAttr(0.0)
        d.CreateMaxForceAttr(float(max_force))
        joint_names.append(jn)

    print(f"  Robot at r={step_r:.0f}\", az={azimuth_deg}deg, facing center")
    print(f"    {cfg['length']}x{cfg['width']}x{cfg['height']}\" {cfg['weight']}lbs "
          f"tire_r={cfg['tire_r']}\" mu={cfg['mu']} damping={damping} maxF={max_force}")
    return cp, joint_names


# -- Measurement ---------------------------------------------------------------

def get_pose(prim):
    """Get position, heading yaw, and tilt from prim transform."""
    tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = tf.ExtractTranslation()
    rot = tf.ExtractRotation()
    quat = rot.GetQuat()
    w = quat.GetReal()
    img = quat.GetImaginary()
    yaw = 2.0 * math.atan2(img[2], w)
    axis = rot.GetAxis()
    tilt = rot.GetAngle() * math.sqrt(axis[0]**2 + axis[1]**2)
    return pos, yaw, tilt


# -- Maneuver Test -------------------------------------------------------------

def test_maneuver(world, robot, drive_max, render=False):
    """
    Complete maneuver test:
      Phase 1: Rotate 90deg from radial to tangential (proportional control)
      Phase 2: Drive forward along circle for 30s

    Returns dict with metrics for each phase.
    """
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(robot.prim_path)
    from isaacsim.core.utils.types import ArticulationAction

    pos0, yaw0, _ = get_pose(chassis)
    init_z = pos0[2]

    # -- Phase 1: Proportional yaw control to rotate 90deg CCW --
    print("  Phase 1: Rotating 90deg (radial -> tangential)...")
    Kp = 6.0   # Proportional gain
    Kd = 1.0   # Derivative gain (damps oscillation)
    max_vel = drive_max
    min_vel = 1.5   # Minimum velocity to overcome static friction
    phase1_steps = int(30.0 / DT)  # 30s timeout
    phase1_done = False
    phase1_time = None
    phase1_max_tilt = 0
    phase1_yaw_history = []
    prev_yaw = yaw0
    yaw_acc = 0.0
    prev_error = math.pi / 2  # Initial error = 90deg

    for s in range(phase1_steps):
        pos, yaw, tilt = get_pose(chassis)

        # Accumulate yaw (handles wraparound)
        dy = yaw - prev_yaw
        if dy > math.pi: dy -= 2 * math.pi
        elif dy < -math.pi: dy += 2 * math.pi
        yaw_acc += dy
        prev_yaw = yaw

        # Error to target (90 degrees CCW)
        error = (math.pi / 2) - yaw_acc
        derror = (error - prev_error) / DT
        prev_error = error
        phase1_max_tilt = max(phase1_max_tilt, tilt)

        if s % 60 == 0:
            phase1_yaw_history.append({
                "t": round(s * DT, 1),
                "yaw_deg": round(math.degrees(yaw_acc), 1),
                "error_deg": round(math.degrees(error), 1),
                "tilt_deg": round(tilt, 1),
            })
            print(f"    t={s*DT:5.1f}s  yaw={math.degrees(yaw_acc):7.1f}deg  "
                  f"err={math.degrees(error):7.1f}deg  tilt={tilt:5.1f}deg")

        # Check if tipped over
        if tilt > 60:
            print(f"    TIPPED at t={s*DT:.1f}s (tilt={tilt:.1f}deg)")
            break

        # Check if we've reached 90 degrees (within 5 degree tolerance)
        if abs(error) < math.radians(5) and not phase1_done:
            phase1_done = True
            phase1_time = s * DT
            print(f"    >>> 90deg reached at t={phase1_time:.1f}s!")
            # Hold position for 2s to verify stability
            for ss in range(int(2.0 / DT)):
                robot.apply_wheel_actions(
                    ArticulationAction(joint_velocities=np.zeros(4)))
                world.step(render=render)
            break

        # PD control: positive error -> need CCW rotation
        # For CCW rotation (positive yaw): LEFT wheels BACKWARD, RIGHT wheels FORWARD
        # Wheel order: [FL, FR, RL, RR]
        #   FL, RL = left side  -> negative velocity for CCW
        #   FR, RR = right side -> positive velocity for CCW
        vel = np.clip(Kp * error + Kd * derror, -max_vel, max_vel)
        # Minimum velocity to overcome static friction
        if abs(vel) < min_vel and abs(error) > math.radians(5):
            vel = min_vel * np.sign(error)

        # CORRECT SIGN: [-vel, +vel, -vel, +vel] for CCW when vel > 0
        vels = np.array([-vel, vel, -vel, vel], dtype=np.float64)
        robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels))
        world.step(render=render)

    phase1_result = {
        "completed": phase1_done,
        "time_s": phase1_time,
        "final_yaw_deg": round(math.degrees(yaw_acc), 1),
        "max_tilt_deg": round(phase1_max_tilt, 1),
        "history": phase1_yaw_history,
    }

    if not phase1_done:
        print(f"    FAILED: only reached {math.degrees(yaw_acc):.1f}deg in 30s")
        return {"phase1": phase1_result, "phase2": None}

    # -- Phase 2: Drive forward in circle --
    print("\n  Phase 2: Driving in circle (30s)...")
    pos1, yaw1, _ = get_pose(chassis)
    orbit_start = math.atan2(pos1[1], pos1[0])
    prev_orbit = orbit_start
    orbit_acc = 0.0
    phase2_max_tilt = 0
    phase2_steps = int(30.0 / DT)
    phase2_time_to_circle = None

    # Differential drive: inner wheels slightly slower to follow circle
    # After 90deg CCW rotation, robot faces tangentially.
    # Left side (+Y in robot frame) is toward center (inner wheels).
    # For CCW orbital motion: all wheels forward, left slightly slower.
    inner_ratio = 0.75
    outer_ratio = 1.0
    fwd_vel = drive_max * 0.7  # Moderate forward speed

    for s in range(phase2_steps):
        # All wheels forward, inner slower for curved path
        vels = np.array([
            fwd_vel * inner_ratio,   # FL (left/inner)
            fwd_vel * outer_ratio,   # FR (right/outer)
            fwd_vel * inner_ratio,   # RL (left/inner)
            fwd_vel * outer_ratio,   # RR (right/outer)
        ], dtype=np.float64)
        robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels))
        world.step(render=render)

        if s % 30 == 0:
            pos, _, tilt = get_pose(chassis)
            cur_orbit = math.atan2(pos[1], pos[0])
            do = cur_orbit - prev_orbit
            if do > math.pi: do -= 2 * math.pi
            elif do < -math.pi: do += 2 * math.pi
            orbit_acc += do
            prev_orbit = cur_orbit
            phase2_max_tilt = max(phase2_max_tilt, tilt)

            r = math.sqrt(pos[0]**2 + pos[1]**2) / ES
            if phase2_time_to_circle is None and abs(orbit_acc) >= 2 * math.pi:
                phase2_time_to_circle = s * DT

            if s % 180 == 0:
                print(f"    t={s*DT:5.1f}s  orbit={math.degrees(orbit_acc):7.1f}deg  "
                      f"r={r:5.1f}\"  tilt={tilt:5.1f}deg")

        # Check tipping
        if phase2_max_tilt > 60:
            print(f"    TIPPED during circle drive at t={s*DT:.1f}s")
            break

    phase2_result = {
        "orbit_deg": round(math.degrees(orbit_acc), 1),
        "full_circle": abs(orbit_acc) >= 2 * math.pi,
        "time_to_circle_s": round(phase2_time_to_circle, 1) if phase2_time_to_circle else None,
        "max_tilt_deg": round(phase2_max_tilt, 1),
    }

    return {"phase1": phase1_result, "phase2": phase2_result}


# -- Sweep Configs -------------------------------------------------------------

def get_sweep_configs():
    """Generate parameter sweep configurations.

    Sweeps the key parameters that affect maneuverability:
    - tire_r: affects ground contact patch and torque arm
    - drive_max: max wheel velocity (with damping, determines peak torque)
    - mu: friction coefficient (rubber on graphite)
    - damping: motor torque constant (force = damping * velocity_error)
    - weight: robot mass (lighter = easier to rotate)
    """
    configs = []
    for tire_r in [2.0, 3.0, 4.0]:
        for drive_max in [15.0, 30.0, 60.0]:
            for mu in [0.5, 0.7, 1.0]:
                for damping in [10000, 50000]:
                    for weight in [25.0, 50.0]:
                        configs.append({
                            "tire_r": tire_r,
                            "length": 14, "width": 13, "height": 9,
                            "weight": weight,
                            "drive_max": drive_max,
                            "mu": mu,
                            "damping": damping,
                            "max_force": 100000.0,
                        })
    return configs


# -- Main ----------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 60)
    print("Phase 1b: Maneuver Optimizer (Radial->Tangential->Circle)")
    print("=" * 60)

    if cli.sweep:
        # Sweep mode: run each config as a separate subprocess
        configs = get_sweep_configs()
        print(f"\nSweep: {len(configs)} configs (subprocess isolation)")

        import subprocess
        script = os.path.abspath(__file__)
        python = os.path.join(r"E:\FORTIS\IsaacSim", "python.bat")
        os.makedirs(RESULTS_DIR, exist_ok=True)

        all_results = []
        for i, cfg in enumerate(configs):
            tag = (f"tr{cfg['tire_r']}_dm{cfg['drive_max']}_mu{cfg['mu']}"
                   f"_d{cfg['damping']}_w{cfg['weight']}")
            result_file = os.path.join(RESULTS_DIR, f"maneuver_{tag}.json")

            print(f"\n[{i+1}/{len(configs)}] {tag}")

            cmd = [python, script,
                   "--tire_r", str(cfg["tire_r"]),
                   "--drive_max", str(cfg["drive_max"]),
                   "--mu", str(cfg["mu"]),
                   "--weight", str(cfg["weight"]),
                   "--damping", str(cfg["damping"]),
                   "--max_force", str(cfg["max_force"]),
                   "--result_file", result_file]
            try:
                t0 = time.time()
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                                   env={**os.environ, "PYTHONIOENCODING": "utf-8"})
                elapsed = time.time() - t0

                if os.path.exists(result_file):
                    with open(result_file) as f:
                        sub = json.load(f)
                    if sub.get("results"):
                        sr = sub["results"][0]
                        all_results.append(sr)
                        p1 = sr.get("phase1", {})
                        p2 = sr.get("phase2")
                        turn_ok = p1.get("completed", False)
                        circle_ok = p2 and p2.get("full_circle", False)
                        print(f"  turn={'YES' if turn_ok else 'NO'}"
                              f"({p1.get('time_s','?')}s) "
                              f"yaw={p1.get('final_yaw_deg','?')}deg "
                              f"circle={'YES' if circle_ok else 'NO'} "
                              f"[{elapsed:.0f}s]")
                    else:
                        print(f"  No results in output file")
                        all_results.append({"config": cfg, "error": "no results"})
                else:
                    print(f"  No output file. stderr: {r.stderr[-500:] if r.stderr else 'none'}")
                    all_results.append({"config": cfg, "error": "no output"})

            except subprocess.TimeoutExpired:
                print(f"  TIMEOUT (300s)")
                all_results.append({"config": cfg, "error": "timeout"})
            except Exception as e:
                print(f"  ERROR: {e}")
                all_results.append({"config": cfg, "error": str(e)})

        # Save combined results
        outpath = os.path.join(RESULTS_DIR, "phase1b_maneuver_sweep.json")
        with open(outpath, "w") as f:
            json.dump({"sweep": True, "total": len(configs), "results": all_results},
                      f, indent=2, default=str)

        # Summary
        print(f"\n{'='*60}")
        print("Maneuver Sweep Summary")
        print(f"{'='*60}")
        turned = [r for r in all_results if r.get("phase1", {}).get("completed")]
        circled = [r for r in all_results
                   if r.get("phase2") and r["phase2"].get("full_circle")]
        print(f"  Tested: {len(all_results)}")
        print(f"  Completed 90deg turn: {len(turned)}/{len(all_results)}")
        print(f"  Completed full circle: {len(circled)}/{len(all_results)}")

        if turned:
            fastest = min(turned, key=lambda r: r["phase1"]["time_s"])
            print(f"\n  Fastest turn: {fastest['phase1']['time_s']}s")
            print(f"    Config: {fastest['config']}")

        if circled:
            best = min(circled, key=lambda r: r["phase1"]["time_s"])
            print(f"\n  Best full maneuver (turn+circle):")
            print(f"    Turn: {best['phase1']['time_s']}s")
            print(f"    Circle: {best['phase2'].get('time_to_circle_s', '?')}s")
            print(f"    Config: {best['config']}")

        print(f"\n  Saved: {outpath}")
        app.close()
        sys.exit(0)

    # -- Single config mode --
    cfg = {
        "tire_r": cli.tire_r, "length": cli.length, "width": cli.width,
        "height": cli.height, "weight": cli.weight,
        "drive_max": cli.drive_max, "mu": cli.mu,
        "damping": cli.damping, "max_force": cli.max_force,
    }

    print(f"\nLoading reactor: {SCENE_REACTOR}")
    if not os.path.exists(SCENE_REACTOR):
        print(f"ERROR: Not found: {SCENE_REACTOR}")
        app.close()
        sys.exit(1)
    omni.usd.get_context().open_stage(SCENE_REACTOR)
    for _ in range(10):
        app.update()

    stage = omni.usd.get_context().get_stage()
    setup_physics(stage)
    prepare_scene(stage, mu=cfg["mu"])
    for _ in range(5):
        app.update()

    chassis_path, joint_names = build_robot(stage, cfg)
    for _ in range(10):
        app.update()

    from isaacsim.core.api import World
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot

    world = World(stage_units_in_meters=1.0)
    robot = world.scene.add(WheeledRobot(
        prim_path=chassis_path,
        name="fortis",
        wheel_dof_names=joint_names,
        create_robot=False,
    ))
    world.reset()

    # Settle
    print("  Settling 3s...")
    for _ in range(180):
        world.step(render=cli.gui)

    # Run maneuver
    result = test_maneuver(world, robot, cfg["drive_max"], render=cli.gui)
    result["config"] = cfg

    p1 = result["phase1"]
    p2 = result.get("phase2")
    status = "PASS" if p1["completed"] and p2 and p2.get("full_circle") else "FAIL"
    turn_ok = "YES" if p1["completed"] else "NO"
    circle_ok = "YES" if p2 and p2.get("full_circle") else "NO"
    print(f"\n  >>> {status}  turn={turn_ok}({p1.get('time_s','?')}s)  "
          f"circle={circle_ok}")

    # Save results
    os.makedirs(RESULTS_DIR, exist_ok=True)
    outpath = cli.result_file or os.path.join(RESULTS_DIR, "phase1b_maneuver.json")
    with open(outpath, "w") as f:
        json.dump({"results": [result]}, f, indent=2, default=str)
    print(f"  Saved: {outpath}")

    if cli.gui:
        print("\nGUI open. Close to exit.")
        while app.is_running():
            app.update()

    world.clear()
    app.close()
