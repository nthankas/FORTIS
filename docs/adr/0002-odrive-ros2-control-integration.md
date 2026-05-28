# ADR 0002: ODrive integration via `odrive_ros2_control`, not the standalone `odrive_node`

- **Status:** Accepted
- **Date:** 2026-05-24
- **Deciders:** FORTIS team

## Context

`fortis_drive` produces wheel velocities, but those need a consumer
that turns them into ODrive S1 motor torque over CAN. We need to pick
the right integration path for that consumer.

Two upstream options exist in `odriverobotics/ros_odrive`:

1. **`odrive_node`** — a standalone ROS 2 node that talks CAN to one
   ODrive. Publishes `/odrive_status` and `/controller_status` and
   accepts `/control_message` for setpoints. Per-axis: one node
   instance per S1.
2. **`odrive_ros2_control`** — a `ros2_control` `SystemInterface` plugin.
   One controller_manager loads the plugin once and manages all four
   S1s as joints of a single hardware system.

These are mutually exclusive (the upstream README is explicit about
this).

FORTIS also has prior decisions that constrain the choice:

- `fortis_description/README.md` and `legacy/README.md` both name
  `odrive_ros2_control` + `ros2_control` as the planned replacement.
- The dev container (both CPU and GPU variants) already preinstalls
  `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, and
  `ros-humble-controller-manager`, so the runtime cost is already paid.
- Downstream needs that haven't landed yet (`robot_localization` with
  wheel odometry, MoveIt for the arm) consume `ros2_control` interfaces
  natively. Going `odrive_node` would mean writing adapters from
  `/controller_status` per-axis into `joint_states`.

## Decision

Adopt **`odrive_ros2_control`** as the FORTIS drive-side ODrive integration.

The wiring lives in three places:

1. **`src/fortis_description/urdf/fortis_chassis.ros2_control.xacro`** —
   `<ros2_control>` system block. Binds the four wheel joints to
   `odrive_ros2_control_plugin/ODriveHardwareInterface` with hardware
   param `can:=<interface>` and per-joint param `node_id:=0/1/2/3` for
   `FL/FR/RR/RL` (chain order — see `project_can_chain_order` memory
   note and the table in `tools/odrive_calibrate.md`). Parametrised by
   `wheels` so the bench can load fewer joints. Mocks out via
   `use_mock_hardware:=true` for CI.
2. **`src/fortis_control/`** — controller_manager YAMLs + launch files.
   The chassis-wide YAML loads a
   `velocity_controllers/JointGroupVelocityController` over the four
   joints. The bench YAML loads it over only FL.
3. **`src/fortis_drive/fortis_drive/drive_node.py`** — adds a publisher
   on `/wheel_velocity_controller/commands` (`std_msgs/Float64MultiArray`)
   in the controller's `[fl, fr, rl, rr]` joint order. The existing
   `WheelVelocities` publishers are kept in parallel during bring-up;
   retirement is a follow-up after the new path is bench-verified.

### Why `JointGroupVelocityController` and not `DiffDriveController`

`DiffDriveController` (used by the `odrive_botwheel_explorer` example
upstream) is for two-wheel differential drives. FORTIS is an X-drive
(four omnis at 45°): it needs three-DOF kinematics (Vx, Vy, ωz → 4
wheel speeds). No off-the-shelf controller in `ros2_controllers` handles
that. The existing `fortis_comms.xdrive_kinematics` already does the IK,
so the minimum-friction path is a group velocity controller as the
transport: `drive_node` does the IK, the controller relays the four
numbers to the four S1s.

A custom `XDriveController` could fold the IK into ros2_control's
realtime loop, but it's strictly more code for the same behaviour and
the IK is already covered by a URDF-sync drift regression test in
`fortis_comms`. Defer until there's a specific reason (e.g. realtime
guarantees the host-side IK can't meet).

### Why ros_odrive is vendored, not apt

`odriverobotics/ros_odrive` is not in any apt repo. Brought in as a
source dep via `tools/vendor_repos.yaml` (vcs format), pinned to a
specific commit hash. Bumping the pin is a deliberate review-time
decision; a moving branch can break our build on any upstream push.

### Why bench-one-motor + full-chassis launches are separate files

A single launch with a `wheels:` arg would work, but two files make the
operator-facing intent obvious. The bench launch hardcodes the
single-wheel YAML and the `wheels:=fl` xacro arg as a matched pair; the
production launch hardcodes the four-wheel YAML and the four-wheel
xacro arg. Misuse — loading the wrong controller against the wrong
hardware — would be a silent failure mode where the wrong motor moves.
Pairing the YAML and the xacro arg in the same file removes the
misuse surface.

## Consequences

**Benefits:**

- One config surface (`ros2_control`) for the chassis + the arm (when
  the Teensy SystemInterface lands), the same one MoveIt consumes.
- `joint_states` is published as a standard topic, ready for
  `robot_localization` + the rest of the stack.
- Calibration stays out of the repo. `tools/odrive_calibrate.md`
  documents the one-time `odrivetool` workflow.
- Mock-hardware path keeps CI passing on machines without a CAN bus.

**Costs:**

- One more dev dependency to vendor (`ros_odrive` at a pinned commit).
- Operators have to remember to bring up SocketCAN before launching.
  Documented in `src/fortis_control/README.md`.
- Per the upstream README, error feedback from the S1 (e.g. undervoltage
  disarm) does not surface into the ros2_control state. FORTIS plumbs
  this via `fortis_safety/odrive_health_monitor_node`, which consumes a
  FORTIS-internal `fortis_msgs/OdriveHealth` snapshot (translated from
  upstream `/odrive_status` by a small bridge node) and publishes
  `/fortis/context/drive_healthy` plus `/fortis/events/fault` on
  `True→False` edges. The mission FSM reacts via the existing FAULT
  state.

**What this does not commit us to:**

- The IK staying in `fortis_drive` forever. A future
  `XDriveController` plugin remains an option if the host-side IK loop
  proves insufficient.
- The single-S1 termination jumper convention. Multi-S1 termination
  policy will be a separate hardware decision when the full four-wheel
  bench comes up.
