# FORTIS Phase 2: Controlled Chassis Movement -- Claude Code Prompt

## Execution Mode

Run this entire task autonomously. Do not stop to ask questions or for confirmation. Write all code first, then execute. If something errors, fix it and re-run. Do not ask "should I proceed?" at any point.

All simulation and visualization must be done in Isaac Sim only. Do NOT use pyvista, matplotlib 3D, standalone viewers, or any visualization outside Isaac Sim. The human is working directly on the RTX 4080 PC and views everything through the Isaac Sim GUI viewport locally.

---

## Phase 0: Environment Discovery

Before writing any code, explore the local filesystem:

1. Run `dir E:\FORTIS /s /b` to map the full directory tree.
2. Find the existing Isaac Sim scene files (USD files), Python scripts, and any Phase 1 outputs in `E:\FORTIS\optimizer\` or nearby directories.
3. Identify the reactor USD scene that was built in Phase 1 (contains the half-reactor mesh with triangle mesh collision and graphite physics material).
4. Identify the existing chassis articulation (box body + 4 wheels, skid-steer configuration) and the 4-DOF arm articulation.
5. Check the Isaac Sim installation path (likely `E:\FORTIS\` or an NVIDIA default location).
6. Check what Python environment is being used (Isaac Sim's bundled Python, or a local venv).
7. Report back what you found before proceeding.

Do NOT rebuild the reactor environment, chassis, or arm from scratch. Phase 1 already created these. You are modifying and extending the existing scene.

---

## Context: What Exists from Phase 1

Phase 1 built the following in Isaac Sim (all of this should already exist on disk):

**Reactor Environment:**
- Half-reactor STL imported as a USD scene
- Triangle mesh collision (NOT convex hull -- the reactor interior is concave, convex hulls break collision)
- Scale factor: 2.54x (the STL was in inches, Isaac Sim uses centimeters)
- Graphite physics material applied: static friction 0.3, dynamic friction 0.3, restitution 0.1
- The reactor floor has two flat surfaces separated by a ~4.5-inch step (with a ~0.5-inch gap that is filled/negligible). The sim step height is whatever the STL mesh produces -- the mesh is ground truth.

**Chassis:**
- Parametric box body with 4 wheels in skid-steer configuration
- Fat deflated rubber tires configured via Isaac Sim's Vehicle Wheel API
- The chassis straddles the floor step permanently, meaning two wheels sit on the higher surface and two on the lower surface
- This creates a constant tilt of roughly 7-10 degrees (depends on chassis width and step height)
- The chassis is an articulation with revolute joints at each wheel

**Wheel/Tire Setup (Critical -- Phase 1 Lesson):**
Phase 1 initially built wheels as simple cylinder collision shapes on revolute joints. This caused problems: PhysX approximates cylinders as n-gons for collision, which produces bumpy, unrealistic contact with triangle mesh floors. The fix that was applied during Phase 1 was switching to Isaac Sim's Vehicle Wheel API and wheeled robot infrastructure:
- `isaacsim.robot.wheeled_robots.robots.WheeledRobot` class wrapping the chassis articulation
- `isaacsim.robot.wheeled_robots.controllers.DifferentialController` for converting (linear_velocity, angular_velocity) commands into per-wheel joint velocities
- PhysX's smooth cylinder-vs-triangle-mesh collision (enabled by default in Isaac Sim, verify it is NOT disabled via `--/physics/collisionApproximateCylinders=true` -- that flag DISABLES smooth collision and should stay at its default of false)

Phase 2 MUST continue using this `WheeledRobot` + `DifferentialController` pattern. Do NOT go back to raw joint velocity commands on cylinder geoms. The `DifferentialController` takes `[linear_speed, angular_speed]` and computes per-wheel velocities accounting for wheel radius and wheelbase. For a 4-wheel skid-steer, the left pair and right pair each get the same velocity (left_vel, right_vel) derived from the differential controller math.

**4-DOF Arm:**
- Mounted on the chassis
- Joints: J1 (base yaw), J2 (shoulder pitch), J3 (elbow pitch), J4 (wrist pitch)
- Built as an articulation chain
- Arm link lengths were derived in Phase 1 for 100% coverage of the lower-half reactor interior (below ~55 inches from floor)

**The Problem Phase 1 Revealed:**
- Skid-steer rotation on the step technically works in PhysX
- BUT the rotation is completely uncontrolled: violent, too fast, chassis spins freely and collides with the inner cylinder wall
- This means the current sim is useless for extracting real engineering parameters
- The root cause: the wheel drive parameters (stiffness, damping, max force) were not configured for controlled velocity tracking -- they allowed unconstrained torque which made the wheels spin at whatever speed PhysX computed, with no realistic motor model limiting them

---

## What Phase 2 Must Accomplish

Phase 2 transforms the existing sim from an uncontrolled physics demo into a controlled engineering testbed that produces real, usable parameters for motor selection, gear ratio sizing, and tire specification.

### Goal 1: Controlled Driving via WheeledRobot + DifferentialController

The tire geometry and physics material already exist from Phase 1. The problem is that the driving was uncontrolled. Phase 2 must use Isaac Sim's proper wheeled robot APIs for controlled movement.

**Required setup (using Isaac Sim's wheeled robot extension):**

```python
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

# Wrap the existing chassis articulation as a WheeledRobot
# Identify the 4 wheel joint names from the Phase 1 scene
chassis = WheeledRobot(
    prim_path="/path/to/chassis",  # Find actual path in Phase 0
    name="fortis_chassis",
    wheel_dof_names=["left_front_wheel", "left_rear_wheel", "right_front_wheel", "right_rear_wheel"]
    # Adjust names to match actual joint names from Phase 1
)

# DifferentialController converts (linear_vel, angular_vel) to wheel speeds
controller = DifferentialController(
    name="fortis_drive",
    wheel_radius=0.038,   # READ FROM SCENE -- ~3 inch dia tire
    wheel_base=0.25,      # READ FROM SCENE -- distance between left/right centers
    max_linear_speed=0.3, # m/s conservative limit
    max_angular_speed=1.0 # rad/s conservative limit
)

# Command: [linear_speed (m/s), angular_speed (rad/s)]
actions = controller.forward([0.1, 0.0])  # Slow forward
chassis.apply_wheel_actions(actions)
```

**IMPORTANT for 4-wheel skid-steer:** The DifferentialController outputs 2 velocities (left, right). For 4-wheel skid-steer, map to all 4 wheels: left_front = left_rear = left_vel, right_front = right_rear = right_vel. If `apply_wheel_actions` only handles 2 DOFs, manually apply:

```python
actions = controller.forward([linear_speed, angular_speed])
left_vel = actions.joint_velocities[0]
right_vel = actions.joint_velocities[1]
full_action = ArticulationAction(
    joint_velocities=np.array([left_vel, left_vel, right_vel, right_vel]),
    joint_indices=np.array([idx_FL, idx_RL, idx_FR, idx_RR])
)
chassis.apply_action(full_action)
```

For each wheel joint's Angular Drive parameters (set these for velocity control):
- **Stiffness:** 0 (required for velocity control)
- **Damping:** Start with 1e3, tune if sluggish or oscillatory
- **Max Force (effort limit):** Start with 5 Nm. This torque cap is what prevents the violent spinning from Phase 1.

The controller's `max_linear_speed` and `max_angular_speed` provide an additional software-level velocity clamp.

### Goal 2: Controlled 360-Degree Rotation Test

With the velocity-controlled wheels, test a controlled in-place rotation (point turn) while the chassis straddles the step:

1. Command left-side wheels backward and right-side wheels forward (or vice versa) at a LOW target velocity (start with 0.05 m/s, which is very slow).
2. Measure: Does the chassis complete a full 360-degree rotation without colliding with the inner cylinder or outer wall?
3. Measure: What is the actual turning rate (degrees per second)?
4. Measure: What torque does each wheel actually require (read from the PhysX joint force/torque telemetry)?
5. Measure: Does the chassis remain stable (no tipping, no excessive bouncing, no loss of contact)?
6. Sweep the target velocity from 0.01 m/s to 0.2 m/s in steps. Find the maximum safe turning speed.
7. Sweep the effort limit from 1 Nm to 10 Nm. Find the minimum torque that achieves a controlled full rotation.

### Goal 3: Toroidal Arc Driving Test

Test driving the chassis in a curved path around the toroidal floor (following the curvature of the reactor):

1. Command all four wheels forward at matched speed to drive straight (which, because the floor curves toroidally, produces an arc).
2. To follow the toroidal curve more tightly, use differential wheel speeds (slightly faster outer wheels, slightly slower inner wheels -- standard skid-steer curve following).
3. Measure: Maximum safe driving speed on the tilted step-straddling surface.
4. Measure: Required wheel torque for sustained forward driving at various speeds.
5. Measure: Does the chassis maintain its step-straddling position, or does it drift laterally off the step?
6. Measure: Minimum turning radius achievable without collision (how tight can the arc be?).

### Goal 4: Stability with Arm in Coverage Poses

With the chassis stationary (wheels braked / zero velocity target), command the arm into several representative coverage poses:

1. **Scorpion reach to near wall:** Arm curls over the chassis body to reach the nearest wall surface. This loads J2 (shoulder) heavily.
2. **Extended reach to far wall:** Arm extends outward to reach the farthest surface within its range. This shifts the CG and tests tipping stability.
3. **Upward reach to upper surfaces:** Arm extends upward toward the half-height target. Tests maximum moment arm on the chassis.
4. **Stowed configuration:** Arm folded flat on/against the chassis, height under 5.5 inches.

For each pose, measure:
- Is the chassis stable (does it tip or slide)?
- What is the CG shift relative to the wheelbase?
- What are the normal forces at each wheel (any wheel losing contact = instability)?
- What joint torques does each arm joint require to hold the pose?

### Goal 5: Extract Engineering Parameters

After running Goals 2-4, compile a summary of all extracted parameters:

**Chassis/Drive Parameters:**
- Minimum wheel torque for controlled 360-degree rotation on the step (Nm)
- Maximum safe rotation speed (deg/s)
- Minimum wheel torque for forward driving at 0.1 m/s, 0.2 m/s (Nm)
- Maximum safe forward speed before loss of traction (m/s)
- Effective turning radius at various differential speeds (meters)
- Minimum friction coefficient required for reliable operation (compare against the 0.3 graphite baseline)

**Stability Parameters:**
- CG position relative to wheelbase for each arm pose
- Minimum stability margin (distance from CG projection to support polygon edge) for each pose
- Worst-case arm pose for tipping risk
- Safety factor for each pose (ratio of restoring moment to tipping moment)

**Arm Parameters (from pose holds):**
- Required holding torque at J1, J2, J3, J4 for each coverage pose (Nm)
- Peak torque during arm motion to each pose (Nm) -- this is higher than static hold due to acceleration
- Confirm that the Phase 1 arm link lengths achieve the intended coverage from the straddling chassis position

**Derived Motor/Gearbox Requirements:**
- Based on the torque and speed data, recommend a motor + gearbox specification for the wheel drives
- Example: "Wheels require minimum 3.2 Nm at 15 RPM. A 12V DC motor producing 0.08 Nm at 6000 RPM through a 50:1 planetary gearbox (output: 3.4 Nm at 120 RPM, providing 0.19 m/s with 3-inch tire) would satisfy this."
- Flag if the current chassis dimensions or wheel placement makes any test physically impossible

### Goal 6: Interactive Manual Control Script

In addition to the automated sweeps, create a standalone script `manual_drive.py` that the human can run to manually test any combination of parameters in real time through the Isaac Sim viewport. This script must:

1. Open the Phase 2 USD scene with the Isaac Sim GUI visible (NOT headless).
2. Present an on-screen control panel using Isaac Sim's OmniUI extension (or at minimum, keyboard controls printed to console) with the following adjustable parameters:
   - **Left wheel velocity** (adjustable in real time, range -0.5 to +0.5 m/s)
   - **Right wheel velocity** (adjustable in real time, range -0.5 to +0.5 m/s)
   - **Max torque per wheel** (adjustable, range 0.5 to 20 Nm)
   - **Wheel damping** (adjustable, range 100 to 10000)
   - **Drive mode presets:** buttons/keys for "Point Turn CW", "Point Turn CCW", "Drive Forward", "Drive Backward", "Arc Left", "Arc Right", "Stop/Brake"
3. Display a live telemetry readout (updated every sim step or at 10 Hz minimum):
   - Current chassis position (x, y, z) and yaw angle
   - Actual velocity of each wheel
   - Actual torque being applied at each wheel
   - Chassis tilt (roll, pitch)
   - Estimated turning rate (deg/s)
4. Include a "Record" toggle that, when enabled, logs all telemetry to a CSV file with timestamps. The human can start/stop recording at will.
5. Include a "Reset" function that teleports the chassis back to its starting position and zeroes all velocities.

If OmniUI sliders are too complex to implement reliably, fall back to keyboard controls:
- W/S = increase/decrease forward speed (both sides)
- A/D = differential steer (adjust left/right speed difference)
- Q/E = point turn left/right
- 1-9 = set max torque (1=1Nm, 2=2Nm, ... 9=9Nm, 0=10Nm)
- +/- = adjust damping up/down
- SPACE = emergency stop (all wheels to zero)
- R = toggle recording
- T = reset position
- P = print current telemetry snapshot to console

This script is for the human to explore parameter space interactively and build intuition for what settings produce controlled behavior, complementing the automated sweeps.

---

## Implementation Plan

### Token Efficiency Principle

Claude Code tokens are expensive. Use them ONLY for tasks that require automated iteration: parameter sweeps, multi-run simulations, data logging across many configurations, and report generation. Everything that can be done faster by the human clicking in the Isaac Sim GUI should be listed as a manual instruction, not scripted.

### Step 1: Human Manual Setup (no Claude Code tokens)

The human will do these steps in the Isaac Sim GUI before running any scripts. Claude Code should print these instructions clearly during Phase 0 and then wait for confirmation:

1. Open the Phase 1 USD scene in Isaac Sim.
2. Verify the reactor, chassis, and arm are all present and look correct.
3. For each of the 4 wheel joints, check that an Angular Drive API exists:
   - Select the wheel joint in the stage tree
   - In Properties panel, look for "Angular Drive" section
   - If missing: click "+ Add" > "Physics" > "Angular Drive"
4. For each wheel joint's Angular Drive, set initial values manually:
   - Stiffness: 0
   - Damping: 1000
   - Max Force: 5
   - Target Velocity: 0
5. Save the scene as `reactor_phase2.usd` (File > Save As) in `E:\FORTIS\optimizer\phase2\`
6. Confirm to Claude Code that the scene is ready.

### Step 2: Interactive Manual Control Script (Claude Code writes, human runs with GUI open)

Write `manual_drive.py` (see Goal 6 above). This is the FIRST script Claude Code produces because:
- It lets the human immediately verify that velocity-controlled driving works
- The human can manually find rough "good" parameter ranges before wasting tokens on sweeps
- It validates that the Phase 2 drive configuration actually fixes the violent spinning problem

This script runs WITH the Isaac Sim GUI visible (not headless). The human will visually confirm the chassis behaves sanely before proceeding to automated sweeps.

### Step 3: Rotation Test Sweep (Claude Code -- heavy compute, many runs)

Write `test_rotation.py` that:
1. Opens the Phase 2 USD scene (can run headless for speed)
2. Starts the simulation
3. Lets the chassis settle for 2 seconds (zero velocity on all wheels)
4. Records initial chassis yaw angle
5. Commands a point turn: left wheels at -V, right wheels at +V
6. Runs until either:
   - Chassis completes 360 degrees of yaw rotation, OR
   - Collision detected with inner cylinder or outer wall, OR
   - 60 seconds of sim time elapsed (timeout / stall detection)
7. Logs per-timestep: sim time, chassis position (x,y,z), chassis orientation (quaternion or euler), each wheel's actual velocity, each wheel's applied torque (from joint force sensors), contact forces
8. Repeats for a sweep of target velocities: [0.01, 0.02, 0.05, 0.1, 0.15, 0.2] m/s
9. Repeats for a sweep of max effort limits: [1, 2, 3, 5, 7, 10] Nm
10. Writes all logged data to a CSV or JSON file for post-processing
11. Prints a summary table to console: for each (velocity, effort) pair, reports success/fail, completion time, peak torque, peak contact force

### Step 4: Driving Test Sweep (Claude Code -- heavy compute, many runs)

Write `test_driving.py` that:
1. Opens the Phase 2 USD scene (can run headless for speed)
2. Starts simulation, settles for 2 seconds
3. Test A -- Straight driving: all 4 wheels at matched velocity V. Run for 10 seconds or until collision. Log position, velocity, wheel torques.
4. Test B -- Arc following: outer wheels at V, inner wheels at 0.8*V (creating a gentle arc). Run for 20 seconds. Log turning radius, lateral drift.
5. Test C -- Tight turn: outer wheels at V, inner wheels at 0 (zero-radius turn on one side). Run for 10 seconds. Log turning behavior.
6. Sweep V over [0.05, 0.1, 0.15, 0.2, 0.3] m/s for each test.
7. Write all data to CSV/JSON.
8. Print summary: max safe speed for each maneuver, required torque, turning radius achieved.

### Step 5: Arm Stability Test (Claude Code -- multiple poses, data logging)

Write a standalone Python script `test_arm_stability.py` that:
1. Opens the Phase 2 USD scene
2. Locks all wheels (zero velocity target, effectively braking)
3. For each arm pose (define 4-6 representative poses as joint angle arrays):
   - Command the arm joints to the target angles using position control on the arm joints
   - Wait 3 seconds for the arm to reach the pose and settle
   - Measure and log: arm joint torques, chassis tilt change, wheel normal forces, CG estimate
4. Write all data to CSV/JSON.
5. Print summary: stability margin for each pose, worst-case pose identification.

### Step 6: Parameter Report

Write a Python script `generate_report.py` that:
1. Reads all CSV/JSON output files from Steps 3-5
2. Computes the derived engineering parameters listed in Goal 5
3. Prints a formatted summary report to console
4. Saves the report as a text file `phase2_results.txt`

---

## Isaac Sim API Notes

Phase 1 used Isaac Sim's Python scripting API with the `WheeledRobot` class and `DifferentialController`. Phase 2 continues this pattern. Key references:

**Primary control pattern (WheeledRobot + DifferentialController):**
```python
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

# Wrap existing chassis articulation
chassis = WheeledRobot(
    prim_path="/path/to/chassis",
    name="fortis_chassis",
    wheel_dof_names=["FL_wheel", "RL_wheel", "FR_wheel", "RR_wheel"]
)

# Create controller with physical params from the scene
controller = DifferentialController(
    name="fortis_drive",
    wheel_radius=WHEEL_RADIUS,  # Read from scene geometry
    wheel_base=WHEEL_BASE,      # Read from scene geometry
    max_linear_speed=0.3,
    max_angular_speed=1.0
)

# Command: [linear_vel_m/s, angular_vel_rad/s]
actions = controller.forward([0.1, 0.0])  # slow forward
chassis.apply_wheel_actions(actions)

# For 4-wheel skid-steer, if apply_wheel_actions only maps 2 joints,
# manually map left/right velocities to all 4 wheels:
left_vel = actions.joint_velocities[0]
right_vel = actions.joint_velocities[1]
full_action = ArticulationAction(
    joint_velocities=np.array([left_vel, left_vel, right_vel, right_vel]),
    joint_indices=np.array([idx_FL, idx_RL, idx_FR, idx_RR])
)
chassis.apply_action(full_action)
```

**Reading joint states:**
```python
joint_positions = chassis.get_joint_positions()
joint_velocities = chassis.get_joint_velocities()
# For applied efforts/torques, use the articulation's force sensor data
# or read from the PhysX articulation API directly
```

**Drive parameter configuration via USD (for setting max force, damping):**
```python
from pxr import UsdPhysics, PhysxSchema

joint_prim = stage.GetPrimAtPath("/path/to/wheel_joint")
drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
drive_api.GetStiffnessAttr().Set(0.0)     # zero for velocity control
drive_api.GetDampingAttr().Set(1e3)        # high damping tracks target velocity
drive_api.GetMaxForceAttr().Set(5.0)       # max torque in Nm -- the key limiter
```

**Important units:** USD angular values are in degrees. The ArticulationAction API and DifferentialController use radians. Be careful with conversions.

**Smooth cylinder collision (verify this is active):**
PhysX has special smooth cylinder-vs-triangle-mesh collision enabled by default. Verify it is NOT disabled. The flag `--/physics/collisionApproximateCylinders` should be `false` (default). If set to `true`, it DISABLES smooth collision and wheels will be bumpy on triangle mesh floors.

**Collision detection:**
Monitor contact reports or check if the chassis position has moved outside expected bounds. Isaac Sim's PhysX contact reporting can be configured through the scene's contact report API.

---

## What NOT to Do (Lessons from Phase 1 and Previous Attempts)

1. **Do NOT rebuild the reactor environment.** It already exists as a USD scene with correct triangle mesh collision and graphite material. Load it.
2. **Do NOT rebuild the chassis or arm from scratch.** They exist as articulations in the scene. Modify their drive parameters, do not recreate them.
3. **Do NOT use any visualization outside Isaac Sim.** No pyvista, no matplotlib 3D plots, no standalone OpenGL viewers. All visual verification happens through the Isaac Sim viewport on the local machine.
4. **Do NOT set wheel torque to unlimited/very high values.** That is what caused the violent uncontrolled spinning in Phase 1. The entire point of Phase 2 is torque-limited, velocity-controlled driving.
5. **Do NOT model the arm as always fully extended horizontally.** The arm operates in scorpion configuration for close targets, extended for far targets. Test realistic poses, not worst-case-only horizontal extension.
6. **Do NOT assume the step height is exactly any particular value.** Read it from the actual simulation geometry or from the chassis tilt when settled on the floor. The mesh is ground truth.
7. **Do NOT output 3D plots or render images from Python.** If you need to show spatial data (like a path trace), either log it as coordinates in a CSV or overlay it in the Isaac Sim viewport using debug drawing APIs (e.g., `omni.debugdraw`).
8. **Do NOT use em dashes (--) in any output text.** Use double hyphens (--) or reword.

---

## Manual vs. Scripted Task Division

Claude Code tokens are limited. Only use them for multi-run automated work. The human is sitting at the RTX 4080 PC with Isaac Sim open.

**Human does manually in Isaac Sim GUI (Step 1 -- no tokens spent):**
- Open Phase 1 scene, verify it looks correct
- Add Angular Drive APIs to any wheel joints missing them (select joint > + Add > Physics > Angular Drive)
- Set initial drive parameters on each wheel joint (Stiffness: 0, Damping: 1000, Max Force: 5, Target Velocity: 0)
- Adjust camera view and lighting
- Reposition chassis if needed to start centered on the step
- Save as `reactor_phase2.usd` in `E:\FORTIS\optimizer\phase2\`

**Claude Code writes, human runs interactively (Step 2):**
- `manual_drive.py` -- the interactive control script with keyboard controls and live telemetry
- Human uses this to visually confirm controlled driving works and explore rough parameter ranges

**Claude Code runs autonomously (Steps 3-6 -- this is where tokens are justified):**
- Automated parameter sweeps (rotation test: 36 configurations, driving test: 15 configurations)
- Arm stability tests across multiple poses
- Data logging and aggregation
- Final parameter report generation
- These can run headless for speed since the human already verified behavior visually in Step 2

**Human verifies visually after automated sweeps:**
- Review the parameter report
- Use `manual_drive.py` to sanity-check the recommended parameter ranges
- Confirm arm poses look physically correct by running `test_arm_stability.py` with GUI visible if needed

---

## File Organization

Put all Phase 2 files in: `E:\FORTIS\optimizer\phase2\`

```
E:\FORTIS\optimizer\phase2\
    manual_drive.py          -- Step 2: Interactive control with keyboard + live telemetry
    test_rotation.py         -- Step 3: Automated 360-degree rotation sweep
    test_driving.py          -- Step 4: Automated toroidal arc driving sweep
    test_arm_stability.py    -- Step 5: Automated arm pose stability tests
    generate_report.py       -- Step 6: Compile parameter report
    data/                    -- Output directory for CSVs and JSONs
        rotation_results.json
        driving_results.json
        arm_stability_results.json
        manual_recordings/   -- CSV files from manual_drive.py recording sessions
    phase2_results.txt       -- Final parameter summary
```

---

## Execution Order

1. **Phase 0:** Explore E:\FORTIS, find Phase 1 USD scene and existing scripts. Report findings. Print manual setup instructions for the human (Step 1) and wait for confirmation that `reactor_phase2.usd` is saved.
2. **Step 2:** Write `manual_drive.py`. Human runs it with Isaac Sim GUI visible to verify controlled driving works and explore parameter space.
3. **Step 3:** Run automated rotation test sweep (headless OK). Save data. Print summary.
4. **Step 4:** Run automated driving test sweep (headless OK). Save data. Print summary.
5. **Step 5:** Run automated arm stability tests. Save data. Print summary.
6. **Step 6:** Generate final parameter report from all data.

Start with Phase 0 and report findings before proceeding.