"""
Phase 1b: Skid-Steer Feasibility via Isaac Sim WheeledRobot API

Tests:
  A) 360-degree spin in place (skid-steer pivot)
  B) Drive a full circle around the reactor floor

Run:
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_vehicle_api.py
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_vehicle_api.py --reactor
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_vehicle_api.py --reactor --gui
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_vehicle_api.py --reactor --gui --mode circle
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_vehicle_api.py --tire_r 4 --drive_vel 60 --mu 0.7
  E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_vehicle_api.py --sweep
"""
import os, sys, math, json, argparse, subprocess
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--reactor", action="store_true", help="Test on reactor floor (else flat ground)")
parser.add_argument("--sweep", action="store_true", help="Parameter sweep (runs each config as subprocess)")
parser.add_argument("--mode", choices=["spin", "circle", "both"], default="both",
                    help="Test mode: spin in place, drive circle, or both")
parser.add_argument("--tire_r", type=float, default=3.0, help="Tire radius in inches")
parser.add_argument("--drive_vel", type=float, default=30.0, help="Drive angular velocity rad/s")
parser.add_argument("--mu", type=float, default=0.5, help="Wheel friction coefficient")
parser.add_argument("--duration", type=float, default=30.0, help="Test duration seconds")
parser.add_argument("--length", type=float, default=14.0, help="Chassis length inches")
parser.add_argument("--width", type=float, default=13.0, help="Chassis width (track) inches")
parser.add_argument("--height", type=float, default=9.0, help="Chassis height inches")
parser.add_argument("--weight", type=float, default=50.0, help="Total weight lbs")
parser.add_argument("--radius", type=float, default=None,
                    help="Radial position on floor (inches). Default: 55.5 for step, 63 for flat")
cli, _ = parser.parse_known_args()

# ── Sweep mode: dispatch configs as subprocesses ──────────────────────────────
if cli.sweep:
    script = os.path.abspath(__file__)
    python = os.path.join(r"E:\FORTIS\IsaacSim", "python.bat")
    all_results = []
    configs = []
    for tr in [2.0, 3.0, 4.0]:
        for dv in [10.0, 30.0, 60.0]:
            for mu in [0.3, 0.5, 0.7]:
                configs.append({"tire_r": tr, "drive_vel": dv, "mu": mu})

    print(f"Phase 1b Sweep: {len(configs)} configs")
    results_dir = os.path.join(r"E:\FORTIS\Optimizer\results\phase1")
    os.makedirs(results_dir, exist_ok=True)

    for i, cfg in enumerate(configs):
        print(f"\n[{i+1}/{len(configs)}] tire_r={cfg['tire_r']}  "
              f"drive={cfg['drive_vel']}  mu={cfg['mu']}")
        cmd = [python, script, "--reactor", "--mode", "spin",
               "--tire_r", str(cfg["tire_r"]),
               "--drive_vel", str(cfg["drive_vel"]),
               "--mu", str(cfg["mu"]),
               "--duration", "20"]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=120,
                               env={**os.environ, "PYTHONIOENCODING": "utf-8"})
            # Read the result JSON
            rpath = os.path.join(results_dir, "phase1b_skidsteer_vehicle.json")
            if os.path.exists(rpath):
                with open(rpath) as f:
                    data = json.load(f)
                if data.get("results"):
                    res = data["results"][0]
                    res["config"] = cfg
                    all_results.append(res)
                    print(f"  yaw={res['yaw_deg']}deg  tipped={res['tipped']}  "
                          f"full_360={res['full_360']}")
        except subprocess.TimeoutExpired:
            print(f"  TIMEOUT")
            all_results.append({"config": cfg, "error": "timeout"})
        except Exception as e:
            print(f"  ERROR: {e}")
            all_results.append({"config": cfg, "error": str(e)})

    # Save combined sweep results
    sweep_path = os.path.join(results_dir, "phase1b_sweep_results.json")
    with open(sweep_path, "w") as f:
        json.dump({"total": len(all_results), "results": all_results}, f, indent=2)
    print(f"\nSweep results saved to {sweep_path}")

    # Summary
    rotated = [r for r in all_results if r.get("full_360")]
    print(f"\nFull 360: {len(rotated)}/{len(all_results)}")
    if rotated:
        best = max(rotated, key=lambda r: abs(r.get("yaw_deg", 0)))
        print(f"Best: {best['yaw_deg']}deg  config={best['config']}")
    sys.exit(0)

# ── Single config mode (actual simulation) ────────────────────────────────────
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

ES = 0.024  # reactor.usd effective scale: 0.01 * 2.4
LBS_TO_KG = 0.453592
DT = 1.0 / 60.0

def i2w(inches):
    return inches * ES


def make_annular_ring(stage, path, z_in, r_inner_in, r_outer_in, mu=0.5, segments=72):
    """Create a smooth annular ring collision surface (floor plane)."""
    if stage.GetPrimAtPath(path).IsValid():
        return
    z = i2w(z_in)
    r_in = i2w(r_inner_in)
    r_out = i2w(r_outer_in)

    verts = []
    for i in range(segments + 1):
        angle = 2 * math.pi * i / segments
        c, s = math.cos(angle), math.sin(angle)
        verts.append(Gf.Vec3f(r_in * c, r_in * s, z))
        verts.append(Gf.Vec3f(r_out * c, r_out * s, z))

    faces_idx = []
    faces_cnt = []
    for i in range(segments):
        j = i * 2
        faces_idx.extend([j, j+1, j+3, j+2])  # CCW from above → normal points UP
        faces_cnt.append(4)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces_idx))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(faces_cnt))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    # Make invisible (collision only)
    mesh.GetVisibilityAttr().Set("invisible")

    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("none")

    # Floor friction material
    mat_path = path + "_Mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(mu))
    pm.CreateDynamicFrictionAttr(float(mu))
    pm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    print(f"  Floor ring: {path} Z={z_in}\" r=[{r_inner_in},{r_outer_in}]\"")


def prepare_reactor_floors(stage, mu=0.5):
    """
    Add smooth collision floors ABOVE the raw mesh floor.
    The reactor STL has millions of tiny triangles that create
    artificial roughness. Real reactor floor tiles are smooth.

    We keep the reactor mesh collision enabled for walls/structures,
    but add smooth floor planes 0.3" above the mesh so wheels ride on
    the smooth surface instead of the rough triangle mesh.
    """
    # Keep reactor mesh collision for walls — don't disable it
    # Add smooth floors slightly above mesh floor (0.3" offset)
    FLOOR_OFFSET = 0.3  # inches above mesh floor
    make_annular_ring(stage, "/World/InnerFloor",
                      Z_INNER + FLOOR_OFFSET, R_INNER_FLOOR_MIN, R_INNER_FLOOR_MAX, mu)
    make_annular_ring(stage, "/World/OuterFloor",
                      Z_OUTER + FLOOR_OFFSET, R_OUTER_FLOOR_MIN, R_OUTER_FLOOR_MAX, mu)


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
        print("  PhysicsScene added")


def create_ground(stage, mu=0.5):
    gp = "/World/Ground"
    if stage.GetPrimAtPath(gp).IsValid():
        return
    g = UsdGeom.Mesh.Define(stage, gp)
    s = 10.0
    g.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-s,-s,0), Gf.Vec3f(s,-s,0),
        Gf.Vec3f(s,s,0), Gf.Vec3f(-s,s,0)]))
    g.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0,1,2,3]))
    g.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    g.GetSubdivisionSchemeAttr().Set("none")
    prim = g.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("none")
    mat = UsdShade.Material.Define(stage, "/World/GroundMat")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(mu))
    pm.CreateDynamicFrictionAttr(float(mu))
    pm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    print(f"  Ground at Z=0, mu={mu}")


def build_robot(stage, tire_r_in, length_in, width_in, height_in,
                weight_lbs, cx, cy, floor_z, mu_wheel=0.5):
    """Build parametric 4-wheel skid-steer robot. Returns (chassis_path, joint_names)."""
    rp = "/World/Robot"
    if stage.GetPrimAtPath(rp).IsValid():
        stage.RemovePrim(Sdf.Path(rp))

    wr = i2w(tire_r_in)
    hl = i2w(length_in / 2)
    hw = i2w(width_in / 2)
    hh = i2w(height_in / 2)
    whw = i2w(1.0)
    mass = weight_lbs * LBS_TO_KG

    UsdGeom.Xform.Define(stage, rp)

    # Chassis mesh box
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

    cz = floor_z + wr + hh + 0.001
    UsdGeom.Xformable(cm).AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))

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
        wpm.CreateStaticFrictionAttr(float(mu_wheel))
        wpm.CreateDynamicFrictionAttr(float(mu_wheel))
        wpm.CreateRestitutionAttr(0.2)
    wmat = UsdShade.Material(stage.GetPrimAtPath(wmp))

    # 4 wheels + revolute joints
    wheels = [("FL", +hl, +hw), ("FR", +hl, -hw),
              ("RL", -hl, +hw), ("RR", -hl, -hw)]
    joint_names = []

    for name, dx, dy in wheels:
        wp = f"{rp}/{name}"
        wc = UsdGeom.Cylinder.Define(stage, wp)
        wc.GetRadiusAttr().Set(float(wr))
        wc.GetHeightAttr().Set(float(whw * 2))
        wc.GetAxisAttr().Set("Y")
        UsdGeom.Xformable(wc).AddTranslateOp().Set(
            Gf.Vec3d(cx + dx, cy + dy, floor_z + wr))

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
        d.CreateDampingAttr(10000.0)
        d.CreateStiffnessAttr(0.0)
        d.CreateMaxForceAttr(100000.0)

        joint_names.append(jn)

    print(f"  Robot: {length_in}x{width_in}x{height_in}\" {weight_lbs}lbs "
          f"tire_r={tire_r_in}\" mu={mu_wheel}")
    print(f"  Chassis at ({cx:.4f}, {cy:.4f}, {cz:.4f}), wheel_r={wr:.4f}")
    return cp, joint_names


def get_yaw(prim):
    """Extract yaw (Z rotation) from prim. Returns (pos, yaw_rad)."""
    tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = tf.ExtractTranslation()
    rot = tf.ExtractRotation()
    quat = rot.GetQuat()
    w = quat.GetReal()
    img = quat.GetImaginary()
    yaw = 2.0 * math.atan2(img[2], w)
    return pos, yaw


def get_tilt(prim):
    """Get tilt angle (non-Z rotation component) in degrees."""
    tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rot = tf.ExtractRotation()
    axis = rot.GetAxis()
    return rot.GetAngle() * math.sqrt(axis[0]**2 + axis[1]**2)


# ── Test: Spin in place ──────────────────────────────────────────────────────

def test_spin(world, robot, drive_vel, duration_s=30.0, render=False):
    """Spin in place: left wheels forward, right wheels backward."""
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(robot.prim_path)
    from isaacsim.core.utils.types import ArticulationAction

    pos0, yaw0 = get_yaw(chassis)
    init_z = pos0[2]
    prev_yaw = yaw0
    yaw_acc = 0.0
    max_tilt = 0.0
    max_zdrop = 0.0
    time_to_360 = None

    steps = int(duration_s / DT)
    vels = np.array([drive_vel, -drive_vel, drive_vel, -drive_vel], dtype=np.float64)

    for s in range(steps):
        robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels))
        world.step(render=render)

        if s % 30 == 0:
            pos, yaw = get_yaw(chassis)
            dy = yaw - prev_yaw
            if dy > math.pi: dy -= 2 * math.pi
            elif dy < -math.pi: dy += 2 * math.pi
            yaw_acc += dy
            prev_yaw = yaw

            zdrop = abs(init_z - pos[2])
            max_zdrop = max(max_zdrop, zdrop)
            tilt = get_tilt(chassis)
            max_tilt = max(max_tilt, tilt)

            if time_to_360 is None and abs(yaw_acc) >= 2 * math.pi:
                time_to_360 = s * DT

            if s % 180 == 0:
                print(f"    t={s*DT:5.1f}s  yaw={math.degrees(yaw_acc):8.1f}deg  "
                      f"zdrop={zdrop/ES:5.1f}\"  tilt={tilt:5.1f}deg")

    return {
        "test": "spin",
        "yaw_deg": round(math.degrees(yaw_acc), 1),
        "max_zdrop_in": round(max_zdrop / ES, 2),
        "max_tilt_deg": round(max_tilt, 1),
        "tipped": max_tilt > 30 or max_zdrop > i2w(4),
        "full_360": abs(yaw_acc) >= 2 * math.pi,
        "time_to_360_s": round(time_to_360, 1) if time_to_360 else None,
        "drive_vel": drive_vel,
        "duration_s": duration_s,
    }


# ── Test: Drive a circle ─────────────────────────────────────────────────────

def test_circle(world, robot, drive_vel, duration_s=60.0, render=False):
    """
    Drive the robot in a circle around the reactor.
    Uses differential drive: left wheels slightly slower than right to curve left
    (following the circular reactor floor CCW).
    """
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(robot.prim_path)
    from isaacsim.core.utils.types import ArticulationAction

    pos0, _ = get_yaw(chassis)
    init_angle = math.atan2(pos0[1], pos0[0])
    prev_angle = init_angle
    angle_acc = 0.0
    max_tilt = 0.0
    max_zdrop = 0.0
    init_z = pos0[2]
    time_to_360 = None

    steps = int(duration_s / DT)

    # Differential drive: inner wheels slower, outer wheels faster
    # The robot is on the outer floor, driving CCW around the reactor center.
    # Inner side = towards reactor center (left side if facing tangent CCW)
    # To curve left (towards center): right wheels faster than left
    # Velocity ratio depends on turning radius vs track width
    # For r=63" center, track=13": inner_r=56.5, outer_r=69.5
    # v_inner/v_outer = 56.5/69.5 ≈ 0.813
    inner_ratio = 0.81
    outer_ratio = 1.0

    # FL and RL are left (+Y), FR and RR are right (-Y)
    # For CCW circle: left wheels are inner (slower), right wheels are outer (faster)
    vels = np.array([
        drive_vel * inner_ratio,   # FL (left/inner)
        drive_vel * outer_ratio,   # FR (right/outer)
        drive_vel * inner_ratio,   # RL (left/inner)
        drive_vel * outer_ratio,   # RR (right/outer)
    ], dtype=np.float64)

    for s in range(steps):
        robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels))
        world.step(render=render)

        if s % 30 == 0:
            pos, _ = get_yaw(chassis)
            # Track angular position around reactor center
            cur_angle = math.atan2(pos[1], pos[0])
            da = cur_angle - prev_angle
            if da > math.pi: da -= 2 * math.pi
            elif da < -math.pi: da += 2 * math.pi
            angle_acc += da
            prev_angle = cur_angle

            r = math.sqrt(pos[0]**2 + pos[1]**2) / ES
            zdrop = abs(init_z - pos[2])
            max_zdrop = max(max_zdrop, zdrop)
            tilt = get_tilt(chassis)
            max_tilt = max(max_tilt, tilt)

            in_bounds = R_CENTER_POST < r < R_OUTER_WALL

            if time_to_360 is None and abs(angle_acc) >= 2 * math.pi:
                time_to_360 = s * DT

            if s % 300 == 0:
                print(f"    t={s*DT:5.1f}s  orbit={math.degrees(angle_acc):8.1f}deg  "
                      f"r={r:5.1f}\"  zdrop={zdrop/ES:5.1f}\"  "
                      f"tilt={tilt:5.1f}deg  {'OK' if in_bounds else 'OUT!'}")

    return {
        "test": "circle",
        "orbit_deg": round(math.degrees(angle_acc), 1),
        "max_zdrop_in": round(max_zdrop / ES, 2),
        "max_tilt_deg": round(max_tilt, 1),
        "tipped": max_tilt > 30 or max_zdrop > i2w(4),
        "full_circle": abs(angle_acc) >= 2 * math.pi,
        "time_to_circle_s": round(time_to_360, 1) if time_to_360 else None,
        "drive_vel": drive_vel,
        "duration_s": duration_s,
    }


# ── GUI interactive mode ─────────────────────────────────────────────────────

def run_gui_interactive(world, robot, drive_vel):
    """Keep running with continuous spin for GUI observation."""
    from isaacsim.core.utils.types import ArticulationAction
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(robot.prim_path)

    vels_spin = np.array([drive_vel, -drive_vel, drive_vel, -drive_vel], dtype=np.float64)
    vels_fwd = np.array([drive_vel, drive_vel, drive_vel, drive_vel], dtype=np.float64)

    print("\nGUI Interactive Mode")
    print("  Robot is spinning. Close the window to exit.")
    print("  (The robot alternates: 10s spin, 10s forward drive, repeat)")

    step = 0
    while app.is_running():
        cycle = (step // 600) % 2  # 10s cycles
        if cycle == 0:
            robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels_spin))
        else:
            robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels_fwd))
        world.step(render=True)
        step += 1

        if step % 300 == 0:
            pos, yaw = get_yaw(chassis)
            mode = "SPIN" if cycle == 0 else "FWD "
            print(f"  [{mode}] t={step*DT:.0f}s  "
                  f"pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})  "
                  f"yaw={math.degrees(yaw):.0f}deg")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("FORTIS Phase 1b: Skid-Steer Feasibility (WheeledRobot API)")
    print("=" * 60)

    # Load scene
    if cli.reactor:
        print(f"\nLoading reactor: {SCENE_REACTOR}")
        if not os.path.exists(SCENE_REACTOR):
            print(f"ERROR: Not found: {SCENE_REACTOR}")
            app.close()
            sys.exit(1)
        omni.usd.get_context().open_stage(SCENE_REACTOR)
    else:
        print("\nCreating flat ground scene...")
        omni.usd.get_context().new_stage()

    for _ in range(10):
        app.update()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("ERROR: No stage")
        app.close()
        sys.exit(1)

    if not cli.reactor:
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        xf = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(xf.GetPrim())
        create_ground(stage, mu=cli.mu)
    else:
        # Replace raw mesh collision with smooth floor planes
        prepare_reactor_floors(stage, mu=cli.mu)

    setup_physics(stage)
    for _ in range(5):
        app.update()

    # Placement
    if cli.reactor:
        # Default: step center (r=55.5") for spinning
        # The robot straddles inner (Z=-53.8") and outer (Z=-49.3") floors
        r_pos = cli.radius if cli.radius else 55.5
        floor_z = i2w(Z_OUTER + 0.3)  # Use outer floor height for chassis Z estimate
        robot_x = i2w(r_pos)
        robot_y = 0.0
        print(f"  Robot at r={r_pos}\" (x={robot_x:.4f})")
    else:
        floor_z = 0.0
        robot_x = 0.0
        robot_y = 0.0

    # Build robot
    chassis_path, joint_names = build_robot(
        stage, cli.tire_r, cli.length, cli.width, cli.height,
        cli.weight, robot_x, robot_y, floor_z, cli.mu)

    for _ in range(10):
        app.update()

    # Create World and WheeledRobot
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
    print("\n  Settling 2s...")
    for _ in range(120):
        world.step(render=cli.gui)

    # GUI mode: interactive
    if cli.gui:
        # Run one quick test first, then go interactive
        if cli.mode in ("spin", "both"):
            print(f"\n  Spin test ({cli.duration}s)...")
            spin_r = test_spin(world, robot, cli.drive_vel,
                               duration_s=cli.duration, render=True)
            print(f"\n  SPIN: yaw={spin_r['yaw_deg']}deg  "
                  f"full_360={spin_r['full_360']}  tipped={spin_r['tipped']}")

        if cli.mode in ("circle", "both"):
            world.reset()
            for _ in range(120):
                world.step(render=True)
            print(f"\n  Circle drive test ({cli.duration}s)...")
            circ_r = test_circle(world, robot, cli.drive_vel,
                                 duration_s=cli.duration, render=True)
            print(f"\n  CIRCLE: orbit={circ_r['orbit_deg']}deg  "
                  f"full_circle={circ_r['full_circle']}  tipped={circ_r['tipped']}")

        # Stay in interactive mode
        world.reset()
        for _ in range(120):
            world.step(render=True)
        run_gui_interactive(world, robot, cli.drive_vel)
        app.close()
        sys.exit(0)

    # Headless mode: run tests and save results
    results = {
        "scene": "reactor" if cli.reactor else "flat_ground",
        "config": {
            "tire_r": cli.tire_r, "length": cli.length, "width": cli.width,
            "height": cli.height, "weight": cli.weight,
            "drive_vel": cli.drive_vel, "mu": cli.mu,
        },
    }

    if cli.mode in ("spin", "both"):
        print(f"\n  Spin test ({cli.duration}s, drive_vel={cli.drive_vel})...")
        spin_r = test_spin(world, robot, cli.drive_vel,
                           duration_s=cli.duration, render=False)
        results["spin"] = spin_r
        print(f"\n  SPIN: yaw={spin_r['yaw_deg']}deg  "
              f"full_360={spin_r['full_360']}  "
              f"time_to_360={spin_r['time_to_360_s']}s  "
              f"tipped={spin_r['tipped']}")

    if cli.mode in ("circle", "both"):
        world.reset()
        for _ in range(120):
            world.step(render=False)
        print(f"\n  Circle drive test ({cli.duration}s)...")
        circ_r = test_circle(world, robot, cli.drive_vel,
                             duration_s=min(cli.duration, 60.0), render=False)
        results["circle"] = circ_r
        print(f"\n  CIRCLE: orbit={circ_r['orbit_deg']}deg  "
              f"full_circle={circ_r['full_circle']}  "
              f"time_to_circle={circ_r.get('time_to_circle_s')}s  "
              f"tipped={circ_r['tipped']}")

    # Clearance check
    diag = math.sqrt(cli.length**2 + cli.width**2)
    floor_w = R_OUTER_WALL - R_CENTER_POST
    results["rotation_diagonal_in"] = round(diag, 1)
    results["floor_width_in"] = round(floor_w, 1)
    results["clearance_ok"] = diag < floor_w - 2

    # For sweep compatibility, wrap in a list
    os.makedirs(RESULTS_DIR, exist_ok=True)
    outpath = os.path.join(RESULTS_DIR, "phase1b_skidsteer_vehicle.json")
    with open(outpath, "w") as f:
        json.dump({"results": [results]}, f, indent=2)

    print(f"\n{'='*60}")
    print(f"  Saved: {outpath}")
    print(f"  Clearance: {'OK' if results['clearance_ok'] else 'NO'} "
          f"(diag={diag:.1f}\" vs floor={floor_w:.1f}\")")
    print(f"{'='*60}")

    app.close()
