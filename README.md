# FORTIS

Remotely operated inspection robot for the DIII-D tokamak fusion reactor at General Atomics. ECE 129 senior design at UCSC, target ship date mid-June 2026.

The robot enters the reactor through a 15.75" square access tunnel, descends via tether through the R0 port (22" x 35.5"), and drives on the reactor floor on four omni wheels. It carries a 4-DOF arm for camera inspection of plasma-facing components and for thermocouple application.

## Hardware (current)

| Subsystem | Selection |
|---|---|
| Chassis | ~15" x 9" x 6" octagonal prism, 3" corner chamfers, ~40 lbs |
| Drive | X-drive: 4 omni wheels at 45 degree corners |
| Wheels | AndyMark 8" Dualie Plastic Omni (am-0463) |
| Drive motors | ODrive M8325s brushless x4 |
| Motor controllers | ODrive S1 x4 (CAN bus) |
| Arm | 4-DOF parallel-link, ~36" reach |
| Arm motors | NEMA 17 stepper x4 + Cricket Drive MK II 25:1 gearboxes |
| Arm drivers | TMC5160-TA x4 |
| Compute | Jetson Orin Nano Super |
| Cameras | Orbbec Gemini 2 (depth, on arm), chassis-mounted RGB |
| Power | Tether-supplied (no onboard battery) |

Hardware selections are tracked in the OnShape model and BOM, which live outside this repo.

## Repo layout

```
.devcontainer/      VSCode dev container config
.gitattributes      Force LF line endings on all text files (cross-machine hygiene)
docker/             Dockerfile.dev + docker-compose.yml (ROS 2 Humble desktop)
src/                ROS 2 packages (colcon workspace)
  fortis_safety/    Mission-level state machine + ROS node + REPL console
  fortis_msgs/      Custom message + action types (ChassisCamClick, GraspCandidate, MissionState, WheelVelocities, MoveToPose)
  fortis_comms/     Motor abstractions, ODrive S1 wrapper, X-drive kinematics, EKF (ament_python library)
  fortis_drive/     X-drive ROS node wrapping fortis_comms kinematics, gated by mission state
  fortis_arm/       Arm controller seam (action server + gripper services), gated by mission state; kinematics deferred
  fortis_integration_tests/  Cross-package launch_testing integration tests (test-only, no runtime code)
sim/                Simulation work (Isaac Sim 5.1, runs on Windows host outside the dev container)
  isaac/xdrive/     Canonical chassis + arm sims, tools, docs, results, changelog
analysis/           Drivetrain and arm analysis writeups
legacy/             Reference code from earlier design iterations
```

## Getting started (dev container)

The supported workflow is VSCode "Reopen in Container". The container is ROS 2 Humble desktop with MoveIt 2, robot_localization, ros2_control, foxglove_bridge, and xacro.

1. Install Docker, the VSCode Dev Containers extension, and (on Windows) WSL 2.
2. Open the repo in VSCode.
3. F1 -> "Dev Containers: Reopen in Container". First build takes ~10 min.
4. Inside the container the repo is mounted at `/workspace`.

Outside the container the same image can be brought up with `docker compose -f docker/docker-compose.yml up -d` and entered with `docker exec -it fortis-dev bash`.

## Building the ROS 2 workspace

From `/workspace` inside the container:

```bash
colcon build --packages-select fortis_safety
source install/setup.bash
```

Re-run after touching anything under `src/`. `source install/setup.bash` has to happen once per shell.

## Running the existing pieces

### Mission state machine (fortis_safety)

Start the node:

```bash
ros2 run fortis_safety mission_state_node
```

In a second terminal, drive it interactively with the REPL console:

```bash
ros2 run fortis_safety event_console
```

The console prints the current state in the prompt. Useful commands:

```
event start_orbit                  # publish an Event
set target_pose_valid true         # set a context Bool
state                              # print current state
help                               # full command list
```

End-to-end round trip (verified):
`IDLE -> ORBIT -> TARGETING -> ARM_AT_VIEW -> PICK -> HOLDING -> RETURN_HOME -> IDLE`

Topics:

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/fortis/mission_state` | `std_msgs/String` | published by node | latched (TRANSIENT_LOCAL) |
| `/fortis/events/<name>` | `std_msgs/Empty` | subscribed | one per Event enum value |
| `/fortis/context/<field>` | `std_msgs/Bool` | subscribed | one per known guard field |

### Full workspace test pass

```bash
cd /workspace
source install/setup.bash
colcon build --symlink-install --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm fortis_integration_tests
colcon test --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm fortis_integration_tests
colcon test-result --verbose
```

Expected: 113 tests, 0 failures, 2 skipped. The 2 skipped are copyright-header lint scaffolds that intentionally don't run yet.

### Single-package pytest

For tighter feedback on one package while iterating:

```bash
cd /workspace && source install/setup.bash
colcon test --packages-select fortis_arm   # or fortis_drive, fortis_comms, etc.
colcon test-result --verbose
```

### Cross-package integration test

The seam between `fortis_safety` and `fortis_drive` (the state machine actually gating motion) is exercised end-to-end by `fortis_integration_tests`:

```bash
launch_test src/fortis_integration_tests/test/test_safety_drive_integration.py
```

See `src/fortis_integration_tests/README.md` for what it asserts.

## Status

| Subsystem | State |
|---|---|
| Dev container | working |
| `fortis_safety` (mission state machine) | working, unit tests pass, end-to-end ROS round trip verified |
| `fortis_msgs` (custom messages + actions) | working, 4 messages + 1 action (`MoveToPose`) |
| `fortis_comms` (motor abstractions, kinematics, EKF) | ament_python library, consumed by `fortis_drive` as a regular `<depend>`; tests pass in isolation |
| `fortis_drive` (X-drive ROS node) | working, gated by mission state, parametrized rclpy tests pass |
| `fortis_arm` (arm controller seam) | scaffold, gated by mission state, action + gripper services in place; **kinematics, trajectory, and Teensy serial deferred** |
| `fortis_integration_tests` (cross-package launch_testing) | seam between safety + drive verified end-to-end (gate latches motion on / off within 200 ms of state change) |
| Isaac Sim — chassis (`xdrive_realwheel.py`) | working, real Kaya omni-wheel meshes, supports `--reactor` |
| Isaac Sim — arm (`xdrive_reactor_arm_v3.py`) | **active build target**: 30" CF, NEMA 17 + Cricket J1/J3, NEMA 23 + EG23 J2, Hitec D845WP J4, always 3 lb loaded |
| Isaac Sim — R0 port entry | not started |
| `fortis_description` (URDF) | blocked on Adrian's OnShape model |
| `fortis_perception`, `fortis_localization`, `fortis_bringup` | planned, not started |

The full set of intended ROS 2 packages is `fortis_description`, `fortis_drive`, `fortis_arm`, `fortis_perception`, `fortis_localization`, `fortis_msgs`, `fortis_bringup`, `fortis_safety`, plus `fortis_comms` and `fortis_integration_tests` for the library and test-only roles. Today: `fortis_safety`, `fortis_msgs`, `fortis_comms`, `fortis_drive`, `fortis_arm`, and `fortis_integration_tests` exist.

## Documentation

- `sim/isaac/xdrive/CHANGELOG.md` and `sim/isaac/xdrive/docs/` - simulation history and specs
- `analysis/` - drivetrain rationale (x-drive vs skid-steer, orbit, torque, pivot)
- Per-package READMEs under `src/<pkg>/README.md` cover topic / service / action contracts and per-package test instructions.
