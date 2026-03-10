#!/usr/bin/env python3
"""
test_scene.py -- Load reactor.usd + fortis_robot.usd, drop the robot onto the floor,
verify it settles correctly with wheels on the ground.

Run: E:\FORTIS\IsaacSim\python.bat test_scene.py [--gui]

With --gui: opens the Isaac Sim viewport so you can see the robot.
Without --gui: headless, prints telemetry only.
"""

import sys
import os
import argparse
import math

# Force unbuffered output so prints appear in Isaac Sim
os.environ["PYTHONUNBUFFERED"] = "1"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Open Isaac Sim with GUI viewport")
args = parser.parse_args()

import builtins
_orig_print = builtins.print
def print(*a, **kw):
    kw["flush"] = True
    _orig_print(*a, **kw)
builtins.print = print

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

import omni.usd
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, PhysxSchema

# ---------------------------------------------------------------------------
# Constants (must match build_robot.py and fortis_config.py)
# ---------------------------------------------------------------------------
ES = 0.024  # world units per inch
def i2w(inches):
    return inches * ES

Z_OUTER = -49.3   # outer floor Z in inches
FLOOR_OFFSET = 0.3  # smooth floor offset above STL mesh
STEP_R = 55.5     # radial position of step in inches

WHEEL_RADIUS = 3.0  # inches
CHASSIS_HEIGHT = 5.0  # inches

REACTOR_USD = os.path.join(os.path.dirname(__file__), "..", "scenes", "reactor.usd")
ROBOT_USD = os.path.join(os.path.dirname(__file__), "fortis_robot.usd")

REACTOR_USD = os.path.abspath(REACTOR_USD)
ROBOT_USD = os.path.abspath(ROBOT_USD)

print(f"Reactor USD: {REACTOR_USD}")
print(f"Robot USD:   {ROBOT_USD}")

# ---------------------------------------------------------------------------
# Load reactor scene
# ---------------------------------------------------------------------------
omni.usd.get_context().open_stage(REACTOR_USD)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()

# Remove any stale robot prims that might exist in reactor.usd
for stale_path in ["/fortis", "/fortis_robot", "/Robot"]:
    p = stage.GetPrimAtPath(stale_path)
    if p and p.IsValid():
        stage.RemovePrim(stale_path)
        print(f"Removed stale prim: {stale_path}")

for _ in range(5):
    app.update()

# ---------------------------------------------------------------------------
# Add robot as a reference
# ---------------------------------------------------------------------------
robot_prim = stage.DefinePrim("/Robot", "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USD)

for _ in range(5):
    app.update()

# Verify robot hierarchy
print("\n=== Robot Hierarchy ===")
for prim in Usd.PrimRange(robot_prim):
    indent = "  " * (len(str(prim.GetPath()).split("/")) - 2)
    typ = prim.GetTypeName()
    apis = []
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        apis.append("Articulation")
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        apis.append("RigidBody")
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        apis.append("Collision")
    if prim.IsA(UsdPhysics.Joint):
        apis.append("Joint")
    api_str = f"  [{', '.join(apis)}]" if apis else ""
    print(f"{indent}{prim.GetPath()} ({typ}){api_str}")

# ---------------------------------------------------------------------------
# Position robot on reactor floor
# ---------------------------------------------------------------------------
# Outer floor surface in world coordinates
z_floor = i2w(Z_OUTER + FLOOR_OFFSET)  # -49.3 + 0.3 = -49.0 inches -> -1.176

# Robot center should be at:
#   X = radial position of step (robot straddles the step)
#   Z = floor + wheel_radius + half_chassis_height + small drop gap
tire_r = i2w(WHEEL_RADIUS)       # 0.072
half_h = i2w(CHASSIS_HEIGHT) / 2  # 0.060
robot_z = z_floor + tire_r + half_h + 0.02  # small gap so it drops and settles

robot_x = i2w(STEP_R)  # 55.5 inches -> 1.332

print(f"\nPlacing robot at X={robot_x:.4f}, Z={robot_z:.4f}")
print(f"  Floor Z = {z_floor:.4f}")
print(f"  Tire radius = {tire_r:.4f}")
print(f"  Half chassis height = {half_h:.4f}")

xform = UsdGeom.Xformable(robot_prim)
xform.ClearXformOpOrder()
translate_op = xform.AddTranslateOp()
translate_op.Set(Gf.Vec3d(robot_x, 0, robot_z))

for _ in range(5):
    app.update()

# ---------------------------------------------------------------------------
# Run physics simulation -- let the robot settle
# ---------------------------------------------------------------------------
from isaacsim.core.prims import XFormPrim, Articulation

print("\nStarting physics simulation...")
print("Letting robot settle for 3 seconds...")

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Let physics initialize (need a few frames for articulation to be ready)
for _ in range(5):
    app.update()

# Immediately fold the arm to a safe stowed pose before it can hit anything.
# This teleports the joints -- no physics needed, instant.
try:
    robot_art = Articulation("/Robot")
    robot_art.initialize()
    for _ in range(3):
        app.update()

    # Get DOF names to find arm joint indices
    dof_names = robot_art.dof_names
    print(f"  DOF names: {dof_names}")

    # Build stowed joint positions: wheels at 0, arm folded
    # J1_Yaw=0, J2_Shoulder=-45, J3_Elbow=90, J4_Wrist=0
    # This folds the arm so upper arm tilts inward 45 deg, lower arm bends
    # back 90 deg from there -- keeps everything compact and low
    import numpy as np
    n_dofs = robot_art.num_dof
    stowed_positions = np.zeros(n_dofs)

    for i, name in enumerate(dof_names):
        if "J2_Shoulder" in name:
            stowed_positions[i] = np.radians(-45)  # tilt inward
        elif "J3_Elbow" in name:
            stowed_positions[i] = np.radians(90)    # bend back
        elif "J4_Wrist" in name:
            stowed_positions[i] = np.radians(-45)   # tuck

    robot_art.set_joint_positions(stowed_positions)

    # Also set drive targets to hold the stowed pose
    # Drive targets are in degrees for USD angular drives
    for i, name in enumerate(dof_names):
        joint_path = f"/Robot/Joints/{name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim and joint_prim.IsValid():
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if drive:
                drive.GetTargetPositionAttr().Set(float(np.degrees(stowed_positions[i])))

    print("  Arm teleported to stowed pose")

    for _ in range(5):
        app.update()
except Exception as e:
    print(f"  Warning: Could not stow arm: {e}")

# Read chassis prim pose (the actual rigid body, not the wrapper Xform)
chassis_xfp = XFormPrim("/Robot/Chassis")

settle_steps = int(3.0 * 60)  # 3 seconds at 60Hz
for i in range(settle_steps):
    app.update()
    if (i + 1) % 60 == 0:
        positions, rotations = chassis_xfp.get_world_poses()
        pos = positions[0]
        print(f"  t={((i+1)/60):.1f}s  chassis pos=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

# Final pose
positions, rotations = chassis_xfp.get_world_poses()
pos = positions[0]
print(f"\nFinal chassis position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
print(f"Expected Z ~ {z_floor + tire_r + half_h:.4f}")

# Check if robot fell through floor
if pos[2] < z_floor - 0.5:
    print("WARNING: Robot may have fallen through the floor!")
elif abs(pos[2] - (z_floor + tire_r + half_h)) < 0.1:
    print("Robot settled correctly on the floor.")
else:
    print(f"Robot Z differs from expected by {abs(pos[2] - (z_floor + tire_r + half_h)):.4f}")

# ---------------------------------------------------------------------------
# If GUI mode, keep running so user can inspect
# ---------------------------------------------------------------------------
if args.gui:
    print("\nGUI mode -- press Ctrl+C to exit")
    try:
        while app.is_running():
            app.update()
    except KeyboardInterrupt:
        pass

timeline.stop()
app.close()
