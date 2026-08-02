# fortis_drive

The X-drive command pipeline for FORTIS: operator velocity commands in, per-wheel
velocity commands out, with the mission-state gate and the dead-man stops in between.

## Nodes

| Executable | What it does |
|---|---|
| `drive_node` | Converts `/cmd_vel` into per-wheel angular velocities via the X-drive IK from `fortis_comms`, gated by the mission state. Publishes explicit zeros on rejection, on leaving a driving state, and when `/cmd_vel` goes stale (0.5 s dead-man watchdog). Bench-verified against the real drivetrain — treat its behaviour (signs, gating, watchdog timing) as frozen unless a hardware session says otherwise. |
| `drive_enable_node` | UI-facing enable/disable of the wheel drive. A `std_msgs/Bool` on `/fortis/commands/drive_enable` activates / deactivates `wheel_velocity_controller` through controller_manager's `switch_controller` service (activation puts the four ODrive S1s into CLOSED_LOOP_CONTROL); the *result* is reported on latched `/fortis/drive/armed` so the UI reflects what actually happened, not just the request. |
| `heading_hold_node` | Closed-loop heading hold. Arbitrates the operator's yaw command against a PID on the EKF yaw from `/odometry/filtered`, republishing the corrected Twist on `/cmd_vel_heading`. While the operator is actively turning, their omega passes through verbatim and the hold target tracks the live yaw; on release the controller holds the heading they stopped at. Enabled by the bringup arg `heading_hold:=true`, which remaps `drive_node`'s input. |
| `orbit_node` | Hold-to-run, face-the-center orbit generator. A held direction command on `/fortis/commands/orbit_dir` becomes a streamed constant `/cmd_vel` (tangential strafe plus coupled yaw, omega = v / R); releasing the button stops the stream and one explicit zero goes out. Open-loop on purpose: for a holonomic base a face-center orbit is a constant body twist, so no planner or localization is needed. |

## Hardware reference

The drivetrain (per `FORTIS_FINAL_BOM`):

- **Wheels:** 4x AndyMark 8" Dualie plastic omni (am-0463), no bearings, 80A
  durometer, 120 lb load cap each.
- **Drive motors:** 4x ODrive M8325s 100 KV outrunner brushless, **direct
  drive** (no gearbox), 48 V direct.
- **Motor controllers:** 4x ODrive S1 FOC controllers, sold as the combined
  KIT-S1-M8325s-01 from ODrive Robotics.
- **Bus:** CAN, daisy-chained across the four S1s; ODrive USB-CAN adapter
  attaches to **Jetson USB Port 2**. The adapter has a built-in 120 ohm
  termination, so no external terminator is required.

The nodes here consume the kinematics from `fortis_comms` and are independent
of the motor-driver layer, so the ODrive specifics above are hardware
reference only. Motor-side integration lives in `src/fortis_control/` via
`odrive_ros2_control`; `drive_node` publishes into the velocity controller's
command topic — see Topics below.

## Topics

| Topic | Type | Node / direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `drive_node` subscribes; `orbit_node` publishes | desired chassis velocity (`linear.x` = forward, `linear.y` = strafe, `angular.z` = yaw rate) |
| `/fortis/mission_state` | `std_msgs/String` | `drive_node` subscribes | latched (TRANSIENT_LOCAL + RELIABLE) so we get the latest state on connect |
| `/fortis/drive/wheel_velocities` | `fortis_msgs/WheelVelocities` | `drive_node` publishes | per-wheel rad/s, one per accepted `/cmd_vel`. Kept during ros2_control bring-up; slated for retirement once the controller path is fully trusted. |
| `/fortis/drive/zero_velocities` | `fortis_msgs/WheelVelocities` | `drive_node` publishes | explicit zeros: one per rejected `/cmd_vel`, gate-close, or watchdog stop |
| `/wheel_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `drive_node` publishes | `[fl, fr, rl, rr]` rad/s; consumed by `velocity_controllers/JointGroupVelocityController` from `fortis_control`. Published on BOTH accepted and rejected `/cmd_vel` (zeros on reject) so the controller never coasts at the last accepted setpoint. |
| `/fortis/commands/drive_enable` | `std_msgs/Bool` | `drive_enable_node` subscribes | True = enable (activate controller), False = disable |
| `/fortis/drive/armed` | `std_msgs/Bool` | `drive_enable_node` publishes | latched armed state, updated from the `switch_controller` result |
| `/odometry/filtered` | `nav_msgs/Odometry` | `heading_hold_node` subscribes | EKF estimate; only its yaw is read. The EKF yaw is IMU-gyro-dominated (`fortis_localization`), so the hold does not chase wheel slip. |
| `/cmd_vel_heading` | `geometry_msgs/Twist` | `heading_hold_node` publishes | operator Vx/Vy verbatim, `angular.z` arbitrated (pass-through while turning, PID hold otherwise) |
| `/fortis/commands/orbit_dir` | `geometry_msgs/Twist` | `orbit_node` subscribes | held command from the Foxglove Teleop panel; only the SIGN of `angular.z` is read |

`drive_enable_node` also calls the `/controller_manager/switch_controller`
service (`controller_manager_msgs/srv/SwitchController`).

## Gating and safety stops

The drive accepts `/cmd_vel` only while the mission state is `ORBIT` or
`RETURN_HOME`. Anything else — including the bring-up window before any state
has been received — causes the command to be rejected, zeros to be published,
and a warning to be logged at most once per second per state name.

Three mechanisms guarantee the wheels stop independent of the teleop client:

1. **Rejection zeros.** Every rejected `/cmd_vel` publishes explicit zeros; a
   downstream stop is never inferred from the *absence* of a message.
2. **Gate-close stop.** Entering any non-driving state (e.g. STOP → `IDLE`)
   commands zeros immediately, without waiting for a next `/cmd_vel` that may
   never come if the operator has released teleop.
3. **Dead-man watchdog.** If no `/cmd_vel` arrives within 0.5 s
   (`CMD_VEL_TIMEOUT_S`), zeros are commanded once per stale episode. Teleop
   clients stop publishing on release without sending a zero, and the velocity
   controller latches its last setpoint — without this the robot keeps moving.

`orbit_node` layers its own, shorter (0.3 s) hold dead-man on top so
release-to-stop is crisp instead of waiting out the drive watchdog.

## Parameters

`drive_node` declares exactly one:

| Param | Default | Meaning |
|---|---|---|
| `cmd_vel_timeout_s` | 0.5 | `/cmd_vel` dead-man watchdog window, s (see Gating and safety stops above). Live-overridable from `bringup_params.yaml`. |

Everything else on `drive_node` — the set of allowed states, the throttle
interval, the topic names — stays a module-level constant in `drive_node.py`
to avoid the failure mode of "node silently does the wrong thing because
someone overrode a parameter at launch". Any future parametrisation should
follow `cmd_vel_timeout_s`: `declare_parameter` with an explicit default that
matches the constant.

`heading_hold_node` and `orbit_node` also declare their knobs with
`declare_parameter`; the in-code defaults below are live-overridable from
`fortis_bringup/config/bringup_params.yaml`.

`heading_hold_node`:

| Param | Default | Meaning |
|---|---|---|
| `kp` / `ki` / `kd` | 2.0 / 0.0 / 0.1 | PID gains on the wrapped heading error (rad). |
| `max_omega` | 1.5 | Clamp on the hold correction, rad/s. Pass-through omega is the operator's own command and is not clamped. |
| `turn_deadband` | 0.05 | \|omega_cmd\| above this = "actively turning": pass through and re-arm the target. |
| `i_clamp` | 0.5 | Symmetric clamp on the integral TERM (`ki * integral`), rad/s (anti-windup). |
| `yaw_sign` | 1.0 | Gyro-direction correction. BENCH-VERIFY before trusting: rotate the robot CCW by hand and confirm `/odometry/filtered` yaw INCREASES; set -1.0 only if it decreases. A wrong sign turns the hold into positive feedback. |

`orbit_node`:

| Param | Default | Meaning |
|---|---|---|
| `orbit_speed` | 0.1 | Tangential speed, m/s. Deliberately slow: open-loop omni motion drifts. |
| `orbit_radius` | 1.0 | Orbit radius, m (must be > 0). Smaller R = faster yaw (omega = v / R). |
| `omega_sign` | -1.0 | Boundary sign so the FRONT (cameras) faces the orbit center. Bench-verified 2026-06-11: +1.0 orbited facing outward. |
| `cmd_timeout_s` | 0.3 | Held-command dead-man; shorter than drive_node's 0.5 s watchdog. |
| `publish_rate_hz` | 20.0 | `/cmd_vel` stream rate while orbiting. |

## Building

From `/workspace`:

```bash
colcon build --packages-select fortis_msgs fortis_comms fortis_drive
source install/setup.bash
```

`fortis_msgs` must build first (it generates the `WheelVelocities` Python
class). `colcon` orders this automatically because `fortis_drive`'s
`package.xml` declares `<depend>fortis_msgs</depend>`.

## Running

Normally everything here is launched by `fortis_bringup/bringup.launch.py`
(with `heading_hold:=true` / `orbit:=true` opting in the helper nodes). To run
the drive by hand:

```bash
ros2 run fortis_drive drive_node
```

In a second terminal, watch the gating in action:

```bash
ros2 topic echo /fortis/drive/wheel_velocities
ros2 topic echo /fortis/drive/zero_velocities
```

In a third terminal, drive the gate:

```bash
# put the mission state into ORBIT (allows motion)
ros2 run fortis_safety event_console
fortis> event start_orbit

# in a fourth terminal, send a Twist
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# you should see WheelVelocities messages
# now in event_console: `event stop`  (back to IDLE)
# Twists now produce zero_velocities instead, with one warning per second
```

## Testing

```bash
cd /workspace
colcon build --packages-select fortis_msgs fortis_comms fortis_drive
source install/setup.bash
colcon test --packages-select fortis_drive
colcon test-result --verbose
```

- `test/test_drive_node.py` — a real `DriveNode` plus a helper publisher /
  subscriber over DDS, asserting on what comes out of the wire (not internal
  state): gating per state, wheel/controller-array values against the
  canonical pipeline, the dead-man watchdog, and the gate-close stop.
- `test/test_heading_hold_node.py` — the pure PID law (`HeadingController`,
  no rclpy) plus a ROS round trip for the plumbing.
- `test/test_orbit_node.py` — the pure orbit kinematics (`orbit_twist`), with
  convention-independent asserts (magnitudes, 1/R coupling, reversal symmetry).

Tests pin `ROS_DOMAIN_ID=91` via `test/conftest.py` (the per-package
test-domain registry) so parallel `colcon test` processes cannot cross-talk.

## fortis_comms dependency

The X-drive kinematics (`xdrive_ik_solver`, `WHEEL_RADIUS`) and the shared
latched QoS profile (`latched_qos_profile`) come from the `fortis_comms`
ament_python package under `src/fortis_comms`. It is declared as a `<depend>`
in `package.xml`; colcon puts it on `PYTHONPATH` automatically after
`source install/setup.bash`.
