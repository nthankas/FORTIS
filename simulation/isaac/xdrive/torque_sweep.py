"""
FORTIS headless torque sweep across speeds and directions.

Fresh stage per test to avoid cumulative yaw corruption. GPU physics for speed.
Crashed before completing on first run (GPU memory from 288 sphere prims/test)
-- may need to reduce spheres_per_roller or run fewer tests per session.

Usage: IsaacSim\\python.bat torque_sweep.py
"""
import os, sys, math, json, subprocess
import numpy as np
os.environ["PYTHONIOENCODING"] = "utf-8"

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1280, "height": 720,
                     "physics_gpu": 0})

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
SPHERE_RADIUS = 12.7 * MM
SPHERE_CENTER_R = WHEEL_RADIUS - SPHERE_RADIUS
ROLLERS_PER_ROW = 12
SPHERES_PER_ROLLER = 2
ROW_OFFSET_DEG = 15.0
PHYSICS_HZ = 240

SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT
OCT_XY = [
    (SL, -(SW-C)), (SL, (SW-C)), ((SL-C), SW), (-(SL-C), SW),
    (-SL, (SW-C)), (-SL, -(SW-C)), (-(SL-C), -SW), ((SL-C), -SW)]
def mid(a, b): return ((a[0]+b[0])/2, (a[1]+b[1])/2)
WHEEL_XY = {
    "FR": mid(OCT_XY[0], OCT_XY[7]), "FL": mid(OCT_XY[1], OCT_XY[2]),
    "BL": mid(OCT_XY[3], OCT_XY[4]), "BR": mid(OCT_XY[5], OCT_XY[6])}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]


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
    px.CreateGpuFoundPairsCapacityAttr(1024 * 1024)
    px.CreateGpuTotalAggregatePairsCapacityAttr(1024 * 1024)

    gp = UsdGeom.Mesh.Define(stage, "/World/Ground")
    gs = 10.0
    gp.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-gs, -gs, 0), Gf.Vec3f(gs, -gs, 0),
        Gf.Vec3f(gs, gs, 0), Gf.Vec3f(-gs, gs, 0)]))
    gp.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    gp.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    gp.GetSubdivisionSchemeAttr().Set("none")
    UsdPhysics.CollisionAPI.Apply(gp.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(gp.GetPrim()).GetApproximationAttr().Set("none")
    gmat = UsdShade.Material.Define(stage, "/World/GroundMat")
    gpm = UsdPhysics.MaterialAPI.Apply(gmat.GetPrim())
    gpm.CreateStaticFrictionAttr(FRICTION_MU)
    gpm.CreateDynamicFrictionAttr(FRICTION_MU)
    gpm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(gp.GetPrim()).Bind(
        gmat, UsdShade.Tokens.weakerThanDescendants, "physics")


def build_sphere_roller_wheel(stage, wp, wheel_mat):
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


def build_robot(stage):
    rp = "/World/Robot"
    cp = rp + "/Chassis"
    UsdGeom.Xform.Define(stage, rp)
    UsdGeom.Xform.Define(stage, cp)
    p = stage.GetPrimAtPath(cp)
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(CHASSIS_MASS)
    UsdPhysics.MassAPI(p).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(p)
    PhysxSchema.PhysxArticulationAPI.Apply(p)

    half_h = CHASSIS_H / 2.0
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
    mesh = UsdGeom.Mesh.Define(stage, cp + "/body")
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).GetApproximationAttr().Set("convexHull")

    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    wheel_z = 0.0
    wo = WHEEL_WIDTH / 2.0 + 0.003
    for wn in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wn]
        ad = AXLE_ANGLES[wn]
        ar = math.radians(ad)
        nl = math.sqrt(cx*cx + cy*cy)
        wx, wy = cx + cx/nl*wo, cy + cy/nl*wo
        wp = rp + f"/Wheel_{wn}"
        UsdGeom.Xform.Define(stage, wp)
        wprim = stage.GetPrimAtPath(wp)
        UsdPhysics.RigidBodyAPI.Apply(wprim)
        UsdPhysics.MassAPI.Apply(wprim).CreateMassAttr(WHEEL_MASS)
        wxf = UsdGeom.Xformable(wprim)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(ad)
        build_sphere_roller_wheel(stage, wp, wheel_mat)
        jp = rp + f"/Joint_{wn}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.GetBody0Rel().SetTargets([Sdf.Path(cp)])
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

    sz = WHEEL_RADIUS + 0.01
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(rp))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, sz))
    return rp, cp, sz


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


def get_pose(stage, cp):
    mat = UsdGeom.Xformable(stage.GetPrimAtPath(cp)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    yaw = math.degrees(math.atan2(mat[1][0], mat[0][0]))
    return np.array([t[0], t[1], t[2]]), yaw


def run_single_test(name, vx, vy, omega, duration_s=5.0):
    """Fresh stage per test. Returns results dict."""
    ctx = omni.usd.get_context()
    ctx.new_stage()
    for _ in range(10): app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    build_scene(stage)
    rp, cp, sz = build_robot(stage)
    for _ in range(20): app.update()

    world = World(stage_units_in_meters=1.0)
    omni.timeline.get_timeline_interface().play()
    for _ in range(10): app.update()
    art = Articulation(cp)
    art.initialize()
    for _ in range(20): world.step(render=False)
    ndof = art.num_dof
    art.switch_control_mode("velocity", joint_indices=np.arange(ndof))

    # Settle 3s
    for _ in range(3 * PHYSICS_HZ): world.step(render=False)
    p0, yaw0 = get_pose(stage, cp)

    # Drive
    tv = xdrive_ik(vx, vy, omega)
    va = np.zeros(ndof)
    va[:4] = tv
    steps = int(duration_s * PHYSICS_HZ)

    torque_samples = []
    vel_samples = []
    for i in range(steps):
        if art.is_physics_handle_valid():
            art.set_joint_velocity_targets(va.reshape(1, -1))
        world.step(render=False)
        if i % 24 == 0:
            try:
                je = art.get_measured_joint_efforts().flatten()[:4]
                jv = art.get_joint_velocities().flatten()[:4]
                torque_samples.append(np.abs(je))
                vel_samples.append(jv)
            except: pass

    p1, yaw1 = get_pose(stage, cp)

    # Stop and coast 1s
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
    for _ in range(PHYSICS_HZ): world.step(render=False)
    p2, _ = get_pose(stage, cp)

    omni.timeline.get_timeline_interface().stop()

    dx = (p1[0] - p0[0]) / IN
    dy = (p1[1] - p0[1]) / IN
    dyaw = yaw1 - yaw0
    dist = math.sqrt(dx*dx + dy*dy)
    heading = math.degrees(math.atan2(dy, dx)) if dist > 0.1 else 0
    coast = np.linalg.norm(p2[:2] - p1[:2]) / IN

    torques = np.array(torque_samples) if torque_samples else np.zeros((1, 4))
    vels = np.array(vel_samples) if vel_samples else np.zeros((1, 4))

    peak_torque = torques.max(axis=0)
    avg_torque = torques.mean(axis=0)
    rms_torque = np.sqrt((torques**2).mean(axis=0))
    avg_vel = np.abs(vels).mean(axis=0)

    if vx != 0 and vy == 0 and omega == 0:
        drift_pct = abs(dy / dx * 100) if abs(dx) > 0.5 else 0
    elif vy != 0 and vx == 0 and omega == 0:
        drift_pct = abs(dx / dy * 100) if abs(dy) > 0.5 else 0
    else:
        drift_pct = 0

    return {
        "name": name, "vx": vx, "vy": vy, "omega": omega,
        "dx": dx, "dy": dy, "dyaw": dyaw, "dist": dist,
        "heading": heading, "drift_pct": drift_pct, "coast": coast,
        "peak_torque": peak_torque.tolist(),
        "avg_torque": avg_torque.tolist(),
        "rms_torque": rms_torque.tolist(),
        "avg_vel": avg_vel.tolist(),
    }


# ========== SWEEP ==========
print("=" * 100, flush=True)
print("FORTIS X-Drive Torque Verification Sweep (GPU, 240Hz, fresh reset per test)", flush=True)
print(f"Chassis: {CHASSIS_L/IN:.0f}x{CHASSIS_W/IN:.0f}x{CHASSIS_H/IN:.0f}\" "
      f"{CHASSIS_MASS:.1f}kg, mu={FRICTION_MU}", flush=True)
print(f"Wheels: {WHEEL_RADIUS*1000:.0f}mm, spheres: r={SPHERE_RADIUS*1000:.1f}mm "
      f"2x{ROLLERS_PER_ROW}x{SPHERES_PER_ROLLER}={2*ROLLERS_PER_ROW*SPHERES_PER_ROLLER}/wheel", flush=True)
print("=" * 100, flush=True)

tests = [
    # Forward at multiple speeds (5s each)
    ("fwd_0.05",  0.05,  0, 0),
    ("fwd_0.10",  0.10,  0, 0),
    ("fwd_0.15",  0.15,  0, 0),
    ("fwd_0.20",  0.20,  0, 0),
    ("fwd_0.30",  0.30,  0, 0),
    # Backward
    ("bwd_0.10", -0.10,  0, 0),
    ("bwd_0.20", -0.20,  0, 0),
    # Strafe
    ("stL_0.10",  0,  0.10, 0),
    ("stL_0.20",  0,  0.20, 0),
    ("stR_0.10",  0, -0.10, 0),
    ("stR_0.20",  0, -0.20, 0),
    # Rotation
    ("rotCW_0.3",  0, 0, -0.3),
    ("rotCW_0.5",  0, 0, -0.5),
    ("rotCCW_0.3", 0, 0,  0.3),
    ("rotCCW_0.5", 0, 0,  0.5),
    # Orbit (strafe + yaw, r≈1.4m ≈ 55")
    ("orb_CCW_0.1", 0, 0.10, 0.071),
    ("orb_CCW_0.2", 0, 0.20, 0.143),
    ("orb_CW_0.1",  0,-0.10,-0.071),
    ("orb_CW_0.2",  0,-0.20,-0.143),
]

results = []
print(f"\n{'Test':<15} {'dx':>7} {'dy':>7} {'dyaw':>6} {'Drift%':>6} "
      f"{'PeakT':>6} {'AvgT':>6} {'RmsT':>6} {'Coast':>6}", flush=True)
print("-" * 85, flush=True)

for name, vx, vy, w in tests:
    sys.stdout.write(f"  Running {name}...")
    sys.stdout.flush()
    r = run_single_test(name, vx, vy, w, duration_s=5.0)
    results.append(r)
    pt = max(r["peak_torque"])
    at = max(r["avg_torque"])
    rt = max(r["rms_torque"])
    print(f"\r{name:<15} {r['dx']:+7.1f} {r['dy']:+7.1f} {r['dyaw']:+6.1f} "
          f"{r['drift_pct']:6.1f} {pt:6.2f} {at:6.2f} {rt:6.2f} {r['coast']:6.2f}", flush=True)

# Summary
print(f"\n{'='*100}", flush=True)
print("TORQUE SUMMARY (Nm per wheel)", flush=True)
print(f"{'='*100}", flush=True)

fwd_results = [r for r in results if r["name"].startswith("fwd")]
if fwd_results:
    print("\nFORWARD DRIVE:", flush=True)
    print(f"  {'Speed':>8} {'Peak':>8} {'Avg':>8} {'RMS':>8} {'Drift%':>8}", flush=True)
    for r in fwd_results:
        spd = r["vx"]
        pt = max(r["peak_torque"])
        at = max(r["avg_torque"])
        rt = max(r["rms_torque"])
        print(f"  {spd:8.2f} {pt:8.2f} {at:8.2f} {rt:8.2f} {r['drift_pct']:8.1f}", flush=True)

orb_results = [r for r in results if r["name"].startswith("orb")]
if orb_results:
    print("\nORBIT:", flush=True)
    print(f"  {'Name':>15} {'Peak':>8} {'Avg':>8} {'RMS':>8} {'dyaw':>8}", flush=True)
    for r in orb_results:
        pt = max(r["peak_torque"])
        at = max(r["avg_torque"])
        rt = max(r["rms_torque"])
        print(f"  {r['name']:>15} {pt:8.2f} {at:8.2f} {rt:8.2f} {r['dyaw']:+8.1f}", flush=True)

# Overall recommendation
all_peaks = [max(r["peak_torque"]) for r in results]
all_avgs = [max(r["avg_torque"]) for r in results]
print(f"\nOVERALL:", flush=True)
print(f"  Max peak torque across all tests: {max(all_peaks):.2f} Nm", flush=True)
print(f"  Max avg torque across all tests:  {max(all_avgs):.2f} Nm", flush=True)
print(f"  Recommended continuous:           {max(all_avgs)*2:.1f} Nm (2x margin)", flush=True)
print(f"  Recommended peak:                 {max(all_peaks)*2:.1f} Nm (2x margin)", flush=True)
print(f"  NEO + 12:1 gearing delivers:      ~31 Nm (margin: {31/max(all_peaks):.1f}x peak)", flush=True)
print(f"  NEO + 20:1 gearing delivers:      ~52 Nm (margin: {52/max(all_peaks):.1f}x peak)", flush=True)

# Save results
out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "torque_sweep_results.json")
with open(out_path, "w") as f:
    json.dump(results, f, indent=2)
print(f"\nResults saved: {out_path}", flush=True)

app.close()
