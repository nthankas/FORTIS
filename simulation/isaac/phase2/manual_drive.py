#!/usr/bin/env python3
"""
manual_drive.py -- Interactive skid-steer driving in the reactor scene.

Run: E:\FORTIS\IsaacSim\python.bat manual_drive.py

Controls (hold keys for continuous input):
    W / UP      : Drive forward
    S / DOWN    : Drive backward
    A / LEFT    : Turn left (slow left side, fast right side)
    D / RIGHT   : Turn right (fast left side, slow right side)
    Q           : Spin left in place
    E           : Spin right in place
    SPACE       : Stop / brake
    +/=         : Increase drive speed
    -           : Decrease drive speed
    1-9,0       : Set torque cap (1-10 Nm)
    R           : Reset robot position
    P           : Print detailed telemetry
    ESC         : Quit

Telemetry printed every second.
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

from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

import omni.usd
import omni.timeline
import carb.input
import omni.appwindow
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, PhysxSchema

# ---------------------------------------------------------------------------
# Constants (must match build_robot.py and fortis_config.py)
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

# ---------------------------------------------------------------------------
# Drive parameters
# ---------------------------------------------------------------------------
DRIVE_SPEED = 10.0       # rad/s default wheel velocity target
TURN_FACTOR = 0.5        # inner wheel fraction during arc turns
SPEED_STEP = 2.0         # rad/s per +/- press

# ---------------------------------------------------------------------------
# Load scene
# ---------------------------------------------------------------------------
print(f"Reactor USD: {REACTOR_USD}")
print(f"Robot USD:   {ROBOT_USD}")

omni.usd.get_context().open_stage(REACTOR_USD)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()

# Remove stale robot prims
for stale_path in ["/fortis", "/fortis_robot", "/Robot"]:
    p = stage.GetPrimAtPath(stale_path)
    if p and p.IsValid():
        stage.RemovePrim(stale_path)
        print(f"Removed stale prim: {stale_path}")

for _ in range(5):
    app.update()

# Add robot as reference
robot_prim = stage.DefinePrim("/Robot", "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USD)

for _ in range(5):
    app.update()

# Position robot on the floor
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
# Start physics and initialize articulation
# ---------------------------------------------------------------------------
from isaacsim.core.prims import XFormPrim, Articulation

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
print(f"\nDOF names ({n_dofs}): {dof_names}")

# Find wheel and arm joint indices
wheel_idx = {}
arm_idx = {}
for i, name in enumerate(dof_names):
    if "FL_Drive" in name:
        wheel_idx["FL"] = i
    elif "FR_Drive" in name:
        wheel_idx["FR"] = i
    elif "RL_Drive" in name:
        wheel_idx["RL"] = i
    elif "RR_Drive" in name:
        wheel_idx["RR"] = i
    else:
        arm_idx[name] = i

print(f"Wheel indices: {wheel_idx}")

# Switch wheel joints to velocity control mode (CRITICAL for skid steer)
wheel_joint_indices = list(wheel_idx.values())
robot_art.switch_control_mode("velocity", joint_indices=np.array(wheel_joint_indices))
print(f"Switched wheels to velocity control: indices {wheel_joint_indices}")

# Stow arm joints
stowed = np.zeros(n_dofs)
for i, name in enumerate(dof_names):
    if "J2_Shoulder" in name:
        stowed[i] = np.radians(-45)
    elif "J3_Elbow" in name:
        stowed[i] = np.radians(90)
    elif "J4_Wrist" in name:
        stowed[i] = np.radians(-45)
robot_art.set_joint_positions(stowed)
for _ in range(5):
    app.update()

# Let robot settle
print("Settling robot (2s)...")
chassis_xfp = XFormPrim("/Robot/Chassis")
for i in range(120):
    app.update()

positions, rotations = chassis_xfp.get_world_poses()
pos = positions[0]
print(f"Settled at: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
print("Ready!\n")

# ---------------------------------------------------------------------------
# Keyboard input -- hold-to-drive style
# ---------------------------------------------------------------------------
key_state = {}

def on_keyboard_event(event, *args, **kwargs):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        key_state[event.input] = True
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        key_state[event.input] = False
    return True

app_window = omni.appwindow.get_default_app_window()
keyboard = app_window.get_keyboard()
input_iface = carb.input.acquire_input_interface()
sub = input_iface.subscribe_to_keyboard_events(keyboard, on_keyboard_event)

K = carb.input.KeyboardInput

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
print("=" * 65)
print(" FORTIS Manual Drive")
print("=" * 65)
print(" W/S or UP/DOWN    : Forward / Backward (both sides)")
print(" A/D or LEFT/RIGHT : Skid steer (left back + right fwd, or vice versa)")
print(" W+A, W+D          : Arc turn (one side slower)")
print(" SPACE             : STOP")
print(" +/= / -           : Speed up / down")
print(" 1-9, 0            : Set torque (1..10 Nm)")
print(" R                 : Reset robot to start")
print(" P                 : Print detailed telemetry")
print(" ESC               : Quit")
print("=" * 65)

drive_speed = DRIVE_SPEED
max_torque = 5.0
frame = 0
prev_speed_frame = -999
prev_torque_frame = -999

# Use set_joint_velocity_targets directly (ArticulationAction has API issues in 5.1)

try:
    while app.is_running():
        frame += 1

        # --- Read held keys ---
        fwd    = key_state.get(K.W, False) or key_state.get(K.UP, False)
        bck    = key_state.get(K.S, False) or key_state.get(K.DOWN, False)
        left   = key_state.get(K.A, False) or key_state.get(K.LEFT, False)
        right  = key_state.get(K.D, False) or key_state.get(K.RIGHT, False)
        spin_l = key_state.get(K.Q, False)
        spin_r = key_state.get(K.E, False)
        brake  = key_state.get(K.SPACE, False)
        quit_k = key_state.get(K.ESCAPE, False)

        if quit_k:
            break

        # Speed adjustment (debounced: once per 20 frames = 0.33s)
        if (key_state.get(K.EQUAL, False) or key_state.get(K.NUMPAD_ADD, False)):
            if frame - prev_speed_frame > 20:
                drive_speed = min(drive_speed + SPEED_STEP, 60.0)
                print(f"  Speed: {drive_speed:.0f} rad/s")
                prev_speed_frame = frame
        if (key_state.get(K.MINUS, False) or key_state.get(K.NUMPAD_SUBTRACT, False)):
            if frame - prev_speed_frame > 20:
                drive_speed = max(drive_speed - SPEED_STEP, 2.0)
                print(f"  Speed: {drive_speed:.0f} rad/s")
                prev_speed_frame = frame

        # Torque keys (debounced)
        torque_keys = {
            K.KEY_1: 1, K.KEY_2: 2, K.KEY_3: 3, K.KEY_4: 4, K.KEY_5: 5,
            K.KEY_6: 6, K.KEY_7: 7, K.KEY_8: 8, K.KEY_9: 9, K.KEY_0: 10,
        }
        for tk, tv in torque_keys.items():
            if key_state.get(tk, False) and frame - prev_torque_frame > 20:
                max_torque = float(tv)
                # Update USD drive maxForce on wheel joints
                for wname in ["FL_Drive", "FR_Drive", "RL_Drive", "RR_Drive"]:
                    jp = stage.GetPrimAtPath(f"/Robot/Joints/{wname}")
                    if jp and jp.IsValid():
                        d = UsdPhysics.DriveAPI.Get(jp, "angular")
                        if d:
                            d.GetMaxForceAttr().Set(max_torque)
                print(f"  Torque: {max_torque:.0f} Nm")
                prev_torque_frame = frame
                break

        # Reset (one-shot via release check)
        if key_state.get(K.R, False):
            key_state[K.R] = False
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
            for _ in range(90):
                app.update()
            chassis_xfp = XFormPrim("/Robot/Chassis")
            print("  Position reset!")
            continue

        # Print telemetry on P press (one-shot)
        if key_state.get(K.P, False):
            key_state[K.P] = False
            try:
                p2, r2 = chassis_xfp.get_world_poses()
                pp = p2[0]
                jpos = robot_art.get_joint_positions()
                jvel = robot_art.get_joint_velocities()
                print(f"\n  === Detailed Telemetry ===")
                print(f"  Position (world): ({pp[0]:.4f}, {pp[1]:.4f}, {pp[2]:.4f})")
                print(f"  Position (inches): ({pp[0]/ES:.1f}, {pp[1]/ES:.1f}, {pp[2]/ES:.1f})")
                r_in = math.sqrt(float(pp[0])**2 + float(pp[1])**2) / ES
                print(f"  Radial distance: {r_in:.1f}\"")
                print(f"  Drive speed: {drive_speed:.0f} rad/s, Torque cap: {max_torque:.0f} Nm")
                for i, name in enumerate(dof_names):
                    print(f"    {name}: pos={np.degrees(jpos[i]):7.1f}°  vel={jvel[i]:7.2f} rad/s")
                print(f"  ===========================\n")
            except Exception as e:
                print(f"  Telemetry error: {e}")

        # --- Compute wheel velocities ---
        v_left = 0.0
        v_right = 0.0

        # Skid steer: A/D = differential (one side fwd, other back)
        # W/S = both sides same direction
        # W+A or W+D = arc turn (one side faster)
        if brake:
            v_left = 0.0
            v_right = 0.0
        elif fwd and left:
            # Arc left while moving forward
            v_left = drive_speed * TURN_FACTOR
            v_right = drive_speed
        elif fwd and right:
            # Arc right while moving forward
            v_left = drive_speed
            v_right = drive_speed * TURN_FACTOR
        elif bck and left:
            v_left = -drive_speed * TURN_FACTOR
            v_right = -drive_speed
        elif bck and right:
            v_left = -drive_speed
            v_right = -drive_speed * TURN_FACTOR
        elif fwd:
            v_left = drive_speed
            v_right = drive_speed
        elif bck:
            v_left = -drive_speed
            v_right = -drive_speed
        elif left or spin_l:
            # Skid steer turn left: left wheels back, right wheels forward
            v_left = -drive_speed
            v_right = drive_speed
        elif right or spin_r:
            # Skid steer turn right: left wheels forward, right wheels back
            v_left = drive_speed
            v_right = -drive_speed

        # Apply velocity targets
        vel_targets = np.zeros(n_dofs)
        for side_key, vel in [("FL", v_left), ("RL", v_left),
                               ("FR", v_right), ("RR", v_right)]:
            if side_key in wheel_idx:
                vel_targets[wheel_idx[side_key]] = vel

        robot_art.set_joint_velocity_targets(vel_targets.reshape(1, -1))

        app.update()

        # --- Status line every 60 frames (1 second) ---
        if frame % 60 == 0:
            try:
                p2, r2 = chassis_xfp.get_world_poses()
                pp = p2[0]
                rot = r2[0]  # quaternion [w, x, y, z]
                w, x, y, z = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
                yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

                # Tilt from vertical
                up_z = 1 - 2*(x*x + y*y)
                tilt = math.degrees(math.acos(min(1.0, max(-1.0, up_z))))

                jvel = robot_art.get_joint_velocities()
                fl_v = jvel[wheel_idx["FL"]] if "FL" in wheel_idx else 0
                fr_v = jvel[wheel_idx["FR"]] if "FR" in wheel_idx else 0

                r_in = math.sqrt(float(pp[0])**2 + float(pp[1])**2) / ES
                pos_in = [float(pp[i])/ES for i in range(3)]

                driving = v_left != 0 or v_right != 0
                state = "DRIVING" if driving else "STOPPED"

                sys.stdout.write(
                    f"\r  [{state:7s}] "
                    f"Pos=({pos_in[0]:5.1f},{pos_in[1]:5.1f},{pos_in[2]:5.1f})\" "
                    f"r={r_in:5.1f}\" "
                    f"Yaw={yaw:+6.1f}° Tilt={tilt:4.1f}° "
                    f"Whl L={fl_v:+5.1f} R={fr_v:+5.1f} "
                    f"Spd={drive_speed:.0f} Trq={max_torque:.0f}    "
                )
                sys.stdout.flush()
            except Exception:
                pass

except KeyboardInterrupt:
    print("\nInterrupted")

print("\nShutting down...")
input_iface.unsubscribe_to_keyboard_events(keyboard, sub)
timeline.stop()
app.close()
