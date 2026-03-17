#!/usr/bin/env python3
"""
test_rotation.py -- Automated point-turn (spin-in-place) sweep.

Tests the robot's ability to do 360-degree point turns at various
wheel velocities and torque caps, recording:
  - Time to complete 360 degrees
  - Peak angular velocity (deg/s)
  - Wheel slip ratio
  - Chassis tilt during turn

Results saved to phase2/results/rotation_sweep.json

Run: E:\FORTIS\IsaacSim\python.bat test_rotation.py [--gui]
"""

import sys
import os
import math
import json
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
parser.add_argument("--gui", action="store_true", help="Open viewport")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

import omni.usd
import omni.timeline
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, PhysxSchema

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ES = 0.024
def i2w(inches):
    return inches * ES

Z_OUTER = -49.3
FLOOR_OFFSET = 0.3
STEP_R = 55.5
WHEEL_RADIUS = 3.0
CHASSIS_HEIGHT = 5.0

REACTOR_USD = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenes", "reactor.usd"))
ROBOT_USD = os.path.abspath(os.path.join(os.path.dirname(__file__), "fortis_robot.usd"))

# Sweep parameters
VELOCITIES = [5, 10, 15, 20, 30]   # rad/s wheel velocity targets
TORQUES = [3, 5, 7, 10]             # Nm torque caps
TIMEOUT = 30.0                       # seconds max per trial
SETTLE_TIME = 1.5                    # seconds to settle before each trial

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# ---------------------------------------------------------------------------
# Scene setup
# ---------------------------------------------------------------------------
print(f"Reactor USD: {REACTOR_USD}")
print(f"Robot USD:   {ROBOT_USD}")

omni.usd.get_context().open_stage(REACTOR_USD)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()

for stale_path in ["/fortis", "/fortis_robot", "/Robot"]:
    p = stage.GetPrimAtPath(stale_path)
    if p and p.IsValid():
        stage.RemovePrim(stale_path)

for _ in range(5):
    app.update()

robot_prim = stage.DefinePrim("/Robot", "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USD)

for _ in range(5):
    app.update()

# Position robot
z_floor = i2w(Z_OUTER + FLOOR_OFFSET)
tire_r = i2w(WHEEL_RADIUS)
half_h = i2w(CHASSIS_HEIGHT) / 2
robot_z = z_floor + tire_r + half_h + 0.02
robot_x = i2w(STEP_R)

xform = UsdGeom.Xformable(robot_prim)
xform.ClearXformOpOrder()
translate_op = xform.AddTranslateOp()
initial_pos = Gf.Vec3d(robot_x, 0, robot_z)
translate_op.Set(initial_pos)

for _ in range(5):
    app.update()

# ---------------------------------------------------------------------------
# Initialize physics
# ---------------------------------------------------------------------------
from isaacsim.core.prims import XFormPrim, Articulation
# Use set_joint_velocity_targets directly (ArticulationAction has API issues in 5.1)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    app.update()

robot_art = Articulation("/Robot")
robot_art.initialize()
for _ in range(5):
    app.update()

dof_names = robot_art.dof_names
n_dofs = robot_art.num_dof
print(f"DOF names ({n_dofs}): {dof_names}")

# Find wheel indices
wheel_idx = {}
for i, name in enumerate(dof_names):
    if "FL_Drive" in name:
        wheel_idx["FL"] = i
    elif "FR_Drive" in name:
        wheel_idx["FR"] = i
    elif "RL_Drive" in name:
        wheel_idx["RL"] = i
    elif "RR_Drive" in name:
        wheel_idx["RR"] = i

print(f"Wheel indices: {wheel_idx}")

# Switch wheel joints to velocity control mode
wheel_joint_indices = list(wheel_idx.values())
robot_art.switch_control_mode("velocity", joint_indices=np.array(wheel_joint_indices))
print(f"Switched wheels to velocity control: indices {wheel_joint_indices}")

# Stow arm
stowed = np.zeros(n_dofs)
for i, name in enumerate(dof_names):
    if "J2_Shoulder" in name:
        stowed[i] = np.radians(-45)
    elif "J3_Elbow" in name:
        stowed[i] = np.radians(90)
    elif "J4_Wrist" in name:
        stowed[i] = np.radians(-45)
robot_art.set_joint_positions(stowed)

chassis_xfp = XFormPrim("/Robot/Chassis")

# Initial settle
print("Initial settle...")
for _ in range(int(SETTLE_TIME * 60)):
    app.update()

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
def get_yaw_deg():
    """Get chassis yaw in degrees from world pose."""
    p, r = chassis_xfp.get_world_poses()
    rot = r[0]
    w, x, y, z = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
    yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return yaw, p[0]

def get_tilt_deg():
    """Get chassis tilt from vertical."""
    _, r = chassis_xfp.get_world_poses()
    rot = r[0]
    w, x, y, z = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
    up_z = 1 - 2*(x*x + y*y)
    return math.degrees(math.acos(min(1.0, max(-1.0, up_z))))

def set_torque_cap(torque):
    """Update max force on all wheel drive joints."""
    for wname in ["FL_Drive", "FR_Drive", "RL_Drive", "RR_Drive"]:
        jp = stage.GetPrimAtPath(f"/Robot/Joints/{wname}")
        if jp and jp.IsValid():
            d = UsdPhysics.DriveAPI.Get(jp, "angular")
            if d:
                d.GetMaxForceAttr().Set(float(torque))

def reset_robot():
    """Stop physics, reset position, restart and settle."""
    global robot_art, chassis_xfp
    timeline.stop()
    for _ in range(5):
        app.update()
    translate_op.Set(initial_pos)
    for _ in range(5):
        app.update()
    timeline.play()
    for _ in range(10):
        app.update()
    robot_art = Articulation("/Robot")
    robot_art.initialize()
    robot_art.switch_control_mode("velocity", joint_indices=np.array(wheel_joint_indices))
    robot_art.set_joint_positions(stowed)
    chassis_xfp = XFormPrim("/Robot/Chassis")
    for _ in range(int(SETTLE_TIME * 60)):
        app.update()

# ---------------------------------------------------------------------------
# Run sweep
# ---------------------------------------------------------------------------
results = []
trial_num = 0
total_trials = len(VELOCITIES) * len(TORQUES)

print(f"\n{'='*70}")
print(f"Rotation Sweep: {len(VELOCITIES)} velocities x {len(TORQUES)} torques = {total_trials} trials")
print(f"{'='*70}\n")

for vel in VELOCITIES:
    for torque in TORQUES:
        trial_num += 1
        print(f"Trial {trial_num}/{total_trials}: vel={vel} rad/s, torque={torque} Nm")

        # Reset robot
        reset_robot()
        set_torque_cap(torque)

        # Get initial yaw
        yaw0, pos0 = get_yaw_deg()
        cumulative_angle = 0.0
        prev_yaw = yaw0
        peak_rate = 0.0
        max_tilt = 0.0
        frame = 0
        completed = False
        completion_time = TIMEOUT

        # Spin: left wheels backward, right wheels forward
        vel_targets = np.zeros(n_dofs)
        for side_key, v in [("FL", -vel), ("RL", -vel), ("FR", vel), ("RR", vel)]:
            if side_key in wheel_idx:
                vel_targets[wheel_idx[side_key]] = v

        while frame < int(TIMEOUT * 60):
            robot_art.set_joint_velocity_targets(vel_targets.reshape(1, -1))
            app.update()
            frame += 1

            if frame % 3 == 0:  # sample at 20 Hz
                yaw_now, pos_now = get_yaw_deg()
                delta = yaw_now - prev_yaw
                if delta > 180:
                    delta -= 360
                elif delta < -180:
                    delta += 360
                cumulative_angle += delta
                prev_yaw = yaw_now

                rate = abs(delta) / (3.0 / 60.0)
                if rate > peak_rate:
                    peak_rate = rate

                tilt = get_tilt_deg()
                if tilt > max_tilt:
                    max_tilt = tilt

                if abs(cumulative_angle) >= 360.0 and not completed:
                    completion_time = frame / 60.0
                    completed = True
                    break

        # Get final wheel velocities for slip calculation
        jvel = robot_art.get_joint_velocities()
        avg_wheel_vel = 0
        for side_key in ["FL", "FR", "RL", "RR"]:
            if side_key in wheel_idx:
                avg_wheel_vel += abs(jvel[wheel_idx[side_key]])
        avg_wheel_vel /= 4.0

        slip_ratio = 1.0 - (avg_wheel_vel / vel) if vel > 0 else 0.0

        result = {
            "velocity_cmd": vel,
            "torque_cap": torque,
            "completed_360": completed,
            "time_360_s": round(completion_time, 2) if completed else None,
            "total_angle_deg": round(abs(cumulative_angle), 1),
            "peak_yaw_rate_dps": round(peak_rate, 1),
            "max_tilt_deg": round(max_tilt, 1),
            "avg_wheel_vel": round(avg_wheel_vel, 2),
            "wheel_slip_ratio": round(slip_ratio, 3),
        }
        results.append(result)

        status = f"360° in {completion_time:.1f}s" if completed else f"only {abs(cumulative_angle):.0f}°"
        print(f"  -> {status}, peak_rate={peak_rate:.0f}°/s, tilt={max_tilt:.1f}°, slip={slip_ratio:.3f}")

# ---------------------------------------------------------------------------
# Save results
# ---------------------------------------------------------------------------
os.makedirs(RESULTS_DIR, exist_ok=True)
output_path = os.path.join(RESULTS_DIR, "rotation_sweep.json")
output = {
    "test": "rotation_sweep",
    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    "parameters": {
        "wheel_radius_in": WHEEL_RADIUS,
        "chassis_height_in": CHASSIS_HEIGHT,
        "step_r_in": STEP_R,
        "velocities_tested": VELOCITIES,
        "torques_tested": TORQUES,
        "timeout_s": TIMEOUT,
    },
    "results": results,
}

with open(output_path, "w") as f:
    json.dump(output, f, indent=2)
print(f"\nResults saved to: {output_path}")

# Summary table
print(f"\n{'='*70}")
print(f"{'Vel':>5} {'Trq':>5} {'360?':>5} {'Time':>6} {'Rate':>8} {'Tilt':>6} {'Slip':>6}")
print(f"{'='*70}")
for r in results:
    t360 = f"{r['time_360_s']:.1f}s" if r['completed_360'] else "N/A"
    print(f"{r['velocity_cmd']:5d} {r['torque_cap']:5d} {'YES' if r['completed_360'] else 'NO':>5} "
          f"{t360:>6} {r['peak_yaw_rate_dps']:7.0f}° {r['max_tilt_deg']:5.1f}° {r['wheel_slip_ratio']:5.3f}")

timeline.stop()
app.close()
