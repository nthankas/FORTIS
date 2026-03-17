#!/usr/bin/env python3
"""
run_best_turn.py -- Run the best 90-degree straddle turn config with GUI.

Shows the robot performing the turn in the reactor scene.
Uses the best config found by step_sweep.py:
  R=3.5", W=2.0", WB=10", TW=8", Torque=100Nm

Run: E:\\FORTIS\\IsaacSim\\python.bat run_best_turn.py
"""

import sys
import os
import math
import time

os.environ["PYTHONUNBUFFERED"] = "1"

import builtins
_orig_print = builtins.print
def print(*a, **kw):
    kw["flush"] = True
    _orig_print(*a, **kw)
builtins.print = print

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="Run without GUI")
parser.add_argument("--wr", type=float, default=3.5, help="Wheel radius (inches)")
parser.add_argument("--ww", type=float, default=2.0, help="Wheel width (inches)")
parser.add_argument("--wb", type=float, default=10.0, help="Wheelbase (inches)")
parser.add_argument("--tw", type=float, default=8.0, help="Track width (inches)")
parser.add_argument("--torque", type=float, default=100.0, help="Motor torque (Nm)")
parser.add_argument("--vel", type=float, default=15.0, help="Drive velocity (rad/s)")
parser.add_argument("--mass", type=float, default=40.0, help="Chassis mass (lbs)")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})

import omni.usd
import omni.timeline
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema

# Constants
ES = 0.024
def i2w(inches):
    return inches * ES

Z_INNER = -53.8
Z_OUTER = -49.3
FLOOR_OFFSET = 0.3
STEP_R = 55.5
STEP_HEIGHT = Z_OUTER - Z_INNER
R_INNER_MAX = 54.8
R_OUTER_MIN = 56.2
CHASSIS_HEIGHT = 5.0
CHAMFER_SIZE = 1.0

REACTOR_USD = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenes", "reactor.usd"))

# Config from sweep results
WR = args.wr
WW = args.ww
WB = args.wb
TW = args.tw
TORQUE = args.torque
DRIVE_VEL = args.vel
MASS = args.mass

print(f"\n{'='*65}")
print(f"  FORTIS Best Turn Demo")
print(f"  R={WR}\" W={WW}\" WB={WB}\" TW={TW}\" T={TORQUE}Nm v={DRIVE_VEL}")
print(f"{'='*65}\n")

# Load reactor
omni.usd.get_context().open_stage(REACTOR_USD)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()

for stale in ["/fortis", "/fortis_robot", "/Robot", "/StepWall"]:
    p = stage.GetPrimAtPath(stale)
    if p and p.IsValid():
        stage.RemovePrim(stale)
for _ in range(5):
    app.update()

# Add chamfered step wall
def add_step_wall_chamfered(stage):
    step_prim = UsdGeom.Xform.Define(stage, "/StepWall")
    wall_thickness = i2w(1.4)
    main_wall_height = i2w(STEP_HEIGHT - CHAMFER_SIZE)
    chamfer_h = i2w(CHAMFER_SIZE)
    wall_r = i2w((R_INNER_MAX + R_OUTER_MIN) / 2)
    wall_z = i2w(Z_INNER + FLOOR_OFFSET) + main_wall_height / 2
    ramp_length = chamfer_h * math.sqrt(2)
    ramp_z = i2w(Z_INNER + FLOOR_OFFSET) + main_wall_height + chamfer_h / 2
    ramp_r_offset = chamfer_h / 2

    n_segments = 36
    for i in range(n_segments):
        angle = 2 * math.pi * i / n_segments
        seg_width = 2 * math.pi * wall_r / n_segments * 1.1

        seg_path = f"/StepWall/Wall_{i:02d}"
        seg = UsdGeom.Cube.Define(stage, seg_path)
        seg.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(seg).SetScale(Gf.Vec3f(wall_thickness, seg_width, main_wall_height))
        cx = wall_r * math.cos(angle)
        cy = wall_r * math.sin(angle)
        UsdGeom.XformCommonAPI(seg).SetTranslate(Gf.Vec3d(cx, cy, wall_z))
        UsdGeom.XformCommonAPI(seg).SetRotate(Gf.Vec3f(0, 0, math.degrees(angle)))
        UsdPhysics.CollisionAPI.Apply(seg.GetPrim())
        seg.GetPurposeAttr().Set("guide")

        ramp_path = f"/StepWall/Ramp_{i:02d}"
        ramp = UsdGeom.Cube.Define(stage, ramp_path)
        ramp.GetSizeAttr().Set(1.0)
        ramp_thick = i2w(0.3)
        UsdGeom.XformCommonAPI(ramp).SetScale(Gf.Vec3f(ramp_length, seg_width, ramp_thick))
        ramp_cx = (wall_r - ramp_r_offset) * math.cos(angle)
        ramp_cy = (wall_r - ramp_r_offset) * math.sin(angle)
        UsdGeom.XformCommonAPI(ramp).SetTranslate(Gf.Vec3d(ramp_cx, ramp_cy, ramp_z))
        UsdGeom.XformCommonAPI(ramp).SetRotate(Gf.Vec3f(0, -45, math.degrees(angle)))
        UsdPhysics.CollisionAPI.Apply(ramp.GetPrim())
        ramp.GetPurposeAttr().Set("guide")

    mat = UsdShade.Material.Define(stage, "/StepWall/GraphiteMat")
    phys_mat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    phys_mat.GetStaticFrictionAttr().Set(0.3)
    phys_mat.GetDynamicFrictionAttr().Set(0.3)
    phys_mat.GetRestitutionAttr().Set(0.05)
    for i in range(n_segments):
        for prefix in ["Wall", "Ramp"]:
            seg_prim = stage.GetPrimAtPath(f"/StepWall/{prefix}_{i:02d}")
            if seg_prim and seg_prim.IsValid():
                UsdShade.MaterialBindingAPI.Apply(seg_prim).Bind(
                    mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    print(f"Step wall added (chamfer={CHAMFER_SIZE}\")")

add_step_wall_chamfered(stage)
for _ in range(5):
    app.update()

# Build robot USD
import tempfile
tmp_usd = os.path.join(tempfile.gettempdir(), "fortis_best_turn.usd")

chassis_mass_kg = MASS * 0.4536
wheel_mass_kg = 1.13
chassis_width_in = TW - WW - 0.5
chassis_length_in = max(WB + 1.0, 10.0)

s = Usd.Stage.CreateInMemory()
s.SetMetadata("metersPerUnit", 1.0)
UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
s.SetDefaultPrim(s.DefinePrim("/Robot"))

def box_inertia(mass, lx, ly, lz):
    return (mass/12*(ly**2+lz**2), mass/12*(lx**2+lz**2), mass/12*(lx**2+ly**2))
def cyl_inertia(mass, r, h):
    Iyy = 0.5*mass*r**2; Ixx = mass/12*(3*r**2+h**2); return (Ixx, Iyy, Ixx)

def make_box(path, hx, hy, hz, mass, color, collision=True):
    prim = UsdGeom.Xform.Define(s, path)
    UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
    m = UsdPhysics.MassAPI.Apply(prim.GetPrim())
    m.GetMassAttr().Set(mass)
    ix, iy, iz = box_inertia(mass, hx*2, hy*2, hz*2)
    m.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ix, iy, iz))
    if collision:
        c = UsdGeom.Cube.Define(s, path+"/Col"); c.GetSizeAttr().Set(1.0)
        c.GetPurposeAttr().Set("guide")
        UsdGeom.XformCommonAPI(c).SetScale(Gf.Vec3f(hx*2, hy*2, hz*2))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    v = UsdGeom.Cube.Define(s, path+"/Vis"); v.GetSizeAttr().Set(1.0)
    UsdGeom.XformCommonAPI(v).SetScale(Gf.Vec3f(hx*2, hy*2, hz*2))
    v.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

def make_cyl(path, r, hh, mass, axis, color, collision=True):
    prim = UsdGeom.Xform.Define(s, path)
    UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
    m = UsdPhysics.MassAPI.Apply(prim.GetPrim())
    m.GetMassAttr().Set(mass)
    ix, iy, iz = cyl_inertia(mass, r, hh*2)
    m.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ix, iy, iz))
    if collision:
        c = UsdGeom.Cylinder.Define(s, path+"/Col")
        c.GetRadiusAttr().Set(r); c.GetHeightAttr().Set(hh*2); c.GetAxisAttr().Set(axis)
        c.GetPurposeAttr().Set("guide")
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    v = UsdGeom.Cylinder.Define(s, path+"/Vis")
    v.GetRadiusAttr().Set(r); v.GetHeightAttr().Set(hh*2); v.GetAxisAttr().Set(axis)
    v.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

def make_joint(path, parent, child, axis, anchor, stiffness=0, damping=1000, max_force=5):
    j = UsdPhysics.RevoluteJoint.Define(s, path)
    j.GetAxisAttr().Set(axis)
    j.GetBody0Rel().SetTargets([parent]); j.GetBody1Rel().SetTargets([child])
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*anchor)); j.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
    j.GetLocalRot0Attr().Set(Gf.Quatf(1,0,0,0)); j.GetLocalRot1Attr().Set(Gf.Quatf(1,0,0,0))
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.GetStiffnessAttr().Set(stiffness); d.GetDampingAttr().Set(damping)
    d.GetMaxForceAttr().Set(max_force); d.GetTypeAttr().Set("force")
    PhysxSchema.PhysxJointAPI.Apply(j.GetPrim())

robot = UsdGeom.Xform.Define(s, "/Robot")
UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0,0,0))

ch_hx = i2w(chassis_length_in)/2
ch_hy = i2w(chassis_width_in)/2
ch_hz = i2w(CHASSIS_HEIGHT)/2
make_box("/Robot/Chassis", ch_hx, ch_hy, ch_hz, chassis_mass_kg, (0.2,0.35,0.6))

wr = i2w(WR); wh = i2w(WW)/2
wz = -ch_hz
wx = i2w(WB)/2
wy = i2w(TW)/2

wheels = [("FL",+wx,+wy,wz), ("FR",+wx,-wy,wz), ("RL",-wx,+wy,wz), ("RR",-wx,-wy,wz)]
for name, x, y, z in wheels:
    make_cyl(f"/Robot/{name}", wr, wh, wheel_mass_kg, "Y", (0.15,0.15,0.15))
    UsdGeom.XformCommonAPI(UsdGeom.Xform(s.GetPrimAtPath(f"/Robot/{name}"))).SetTranslate(Gf.Vec3d(x,y,z))

# Arm base
ab_r = i2w(1.5); ab_hh = i2w(1.0)
make_cyl("/Robot/ArmBase", ab_r, ab_hh, 1.0, "Z", (0.6,0.3,0.1), collision=False)
UsdGeom.XformCommonAPI(UsdGeom.Xform(s.GetPrimAtPath("/Robot/ArmBase"))).SetTranslate(Gf.Vec3d(0,0,ch_hz+ab_hh))

UsdGeom.Scope.Define(s, "/Robot/Joints")
for name, x, y, z in wheels:
    jname = name[:2]+"_Drive"
    make_joint(f"/Robot/Joints/{jname}", "/Robot/Chassis", f"/Robot/{name}", "Y", (x,y,z), 0, 1000, TORQUE)
make_joint("/Robot/Joints/J1_Yaw", "/Robot/Chassis", "/Robot/ArmBase", "Z", (0,0,ch_hz), 500, 100, 20)

rmat = UsdShade.Material.Define(s, "/Robot/RubberMat")
rp = UsdPhysics.MaterialAPI.Apply(rmat.GetPrim())
rp.GetStaticFrictionAttr().Set(0.8); rp.GetDynamicFrictionAttr().Set(0.6); rp.GetRestitutionAttr().Set(0.2)
for name,_,_,_ in wheels:
    col = s.GetPrimAtPath(f"/Robot/{name}/Col")
    if col and col.IsValid():
        UsdShade.MaterialBindingAPI.Apply(col).Bind(rmat, UsdShade.Tokens.weakerThanDescendants, "physics")

s.GetRootLayer().Export(tmp_usd)
print(f"Robot built: {tmp_usd}")

# Load robot into scene
robot_prim = stage.DefinePrim("/Robot", "Xform")
robot_prim.GetReferences().AddReference(tmp_usd)
for _ in range(5):
    app.update()

z_floor = i2w(Z_OUTER + FLOOR_OFFSET)
robot_z = z_floor + wr + ch_hz + 0.02
robot_x = i2w(STEP_R)

xform = UsdGeom.Xformable(robot_prim)
xform.ClearXformOpOrder()
t_op = xform.AddTranslateOp()
t_op.Set(Gf.Vec3d(robot_x, 0, robot_z))
for _ in range(5):
    app.update()

# Start physics
from isaacsim.core.prims import XFormPrim, Articulation

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    app.update()

robot_art = Articulation("/Robot")
robot_art.initialize()
for _ in range(3):
    app.update()

dof_names = robot_art.dof_names
n_dofs = robot_art.num_dof
print(f"DOFs: {dof_names}")

widx = {}
for i, name in enumerate(dof_names):
    if "FL" in name and "Drive" in name: widx["FL"] = i
    elif "FR" in name and "Drive" in name: widx["FR"] = i
    elif "RL" in name and "Drive" in name: widx["RL"] = i
    elif "RR" in name and "Drive" in name: widx["RR"] = i

wji = list(widx.values())
robot_art.switch_control_mode("velocity", joint_indices=np.array(wji))

chassis_xfp = XFormPrim("/Robot/Chassis")

# Settle
print("Settling (2s)...")
for _ in range(120):
    app.update()

p0, r0 = chassis_xfp.get_world_poses()
pos0 = p0[0]
rot0 = r0[0]
w, x, y, z = float(rot0[0]), float(rot0[1]), float(rot0[2]), float(rot0[3])
yaw0 = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
print(f"Settled at ({pos0[0]/ES:.1f}, {pos0[1]/ES:.1f}, {pos0[2]/ES:.1f})\" yaw={yaw0:.1f}")

# Set up velocity targets
vel_targets = np.zeros(n_dofs)
for k in ["FL", "RL"]:
    if k in widx: vel_targets[widx[k]] = -DRIVE_VEL
for k in ["FR", "RR"]:
    if k in widx: vel_targets[widx[k]] = DRIVE_VEL

# Run turn
print(f"\nStarting 90-degree turn (drive_vel={DRIVE_VEL} rad/s, torque={TORQUE} Nm)...")
print(f"{'Frame':>6} {'Time':>6} {'Yaw':>8} {'CumYaw':>8} {'Tilt':>6} {'Pos_X':>8} {'Pos_Y':>8}")

prev_yaw = yaw0
cumulative_yaw = 0.0
max_tilt = 0
completed = False

for frame in range(int(20 * 60)):  # 20 seconds max
    robot_art.set_joint_velocity_targets(vel_targets.reshape(1, -1))
    app.update()

    if frame % 6 == 0:  # 10 Hz telemetry
        p, r = chassis_xfp.get_world_poses()
        pos = p[0]; rot = r[0]
        w, x, y, z = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
        yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        up_z = 1 - 2*(x*x + y*y)
        tilt = math.degrees(math.acos(min(1.0, max(-1.0, up_z))))

        dy = yaw - prev_yaw
        if dy > 180: dy -= 360
        elif dy < -180: dy += 360
        cumulative_yaw += dy
        prev_yaw = yaw
        if tilt > max_tilt: max_tilt = tilt

        if frame % 30 == 0:
            print(f"{frame:6d} {frame/60:5.1f}s {yaw:+7.1f} {cumulative_yaw:+7.1f} "
                  f"{tilt:5.1f} {float(pos[0])/ES:7.1f} {float(pos[1])/ES:7.1f}")

        if abs(cumulative_yaw) >= 90.0 and not completed:
            completed = True
            t90 = frame / 60.0
            print(f"\n*** 90 DEGREES REACHED at t={t90:.1f}s ***\n")
            # Keep running a bit more to show final state
            for _ in range(120):
                vel_targets[:] = 0
                robot_art.set_joint_velocity_targets(vel_targets.reshape(1, -1))
                app.update()
            break

# Final state
pf, rf = chassis_xfp.get_world_poses()
final_r = math.sqrt(float(pf[0][0])**2 + float(pf[0][1])**2) / ES
print(f"\nFinal state:")
print(f"  Position: ({pf[0][0]/ES:.1f}, {pf[0][1]/ES:.1f}, {pf[0][2]/ES:.1f})\"")
print(f"  Radial: {final_r:.1f}\" (drift: {abs(final_r - STEP_R):.1f}\")")
print(f"  Cumulative yaw: {cumulative_yaw:.1f}")
print(f"  Max tilt: {max_tilt:.1f}")
print(f"  Completed 90: {'YES' if completed else 'NO'}")

if not args.headless:
    print("\nGUI mode -- press Ctrl+C to exit")
    # Stop driving
    vel_targets[:] = 0
    try:
        while app.is_running():
            robot_art.set_joint_velocity_targets(vel_targets.reshape(1, -1))
            app.update()
    except KeyboardInterrupt:
        pass

if os.path.exists(tmp_usd):
    os.remove(tmp_usd)
timeline.stop()
app.close()
