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
docker/             Dockerfile.dev + docker-compose.yml (ROS 2 Humble desktop)
src/                ROS 2 packages (colcon workspace)
  fortis_safety/    Mission-level state machine + ROS node + REPL console
  fortis_msgs/      Custom message + action types (ChassisCamClick, GraspCandidate, MissionState, WheelVelocities, MoveToPose)
  fortis_comms/     Motor abstractions, ODrive S1 wrapper, X-drive kinematics, EKF
  fortis_drive/     X-drive ROS node wrapping fortis_comms kinematics, gated by mission state
  fortis_arm/       Arm controller seam (action server + gripper services), gated by mission state; kinematics deferred
sim/                Simulation work (Isaac Sim)
  isaac/xdrive/     Canonical chassis + arm sims, tools, docs, results
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

### State-machine unit tests (no ROS required)

```bash
cd /workspace
source install/setup.bash
python3 -m pytest src/fortis_safety/test/ -v
```

### fortis_comms tests (pure Python, no ROS)

```bash
cd /workspace
colcon test --packages-select fortis_comms
colcon test-result --verbose
```

See `src/fortis_comms/README.md` for the integration assessment.

## Status

| Subsystem | State |
|---|---|
| Dev container | working |
| `fortis_safety` (mission state machine) | working, 27 unit tests pass, end-to-end ROS round trip verified |
| `fortis_msgs` (custom messages) | working, 4 message types (ChassisCamClick, GraspCandidate, MissionState, WheelVelocities) |
| `fortis_drive` (X-drive ROS node) | working, gated by mission state, 5 rclpy tests pass |
| `fortis_arm` (arm controller seam) | scaffold, gated by mission state, action + gripper services in place, kinematics deferred |
| `fortis_comms` (motor abstractions, kinematics, EKF) | ament_python package under `src/`, consumed by `fortis_drive` as a regular `<depend>` |
| Isaac Sim 1 (xdrive flat ground + reactor) | working in `sim/isaac/xdrive/canonical/` |
| Isaac Sim 2 (R0 port entry) | not started |
| `fortis_description` (URDF) | blocked on Adrian's OnShape model |
| `fortis_arm` ROS package | blocked on URDF |
| `fortis_perception`, `fortis_localization`, `fortis_bringup` | planned, not started |

The full set of intended ROS 2 packages is `fortis_description`, `fortis_drive`, `fortis_arm`, `fortis_perception`, `fortis_localization`, `fortis_msgs`, `fortis_bringup`, `fortis_safety`. As of this branch, `fortis_safety`, `fortis_msgs`, and `fortis_drive` exist.

## Documentation

- `sim/isaac/xdrive/CHANGELOG.md` and `sim/isaac/xdrive/docs/` - simulation history and specs
- `analysis/` - drivetrain rationale (x-drive vs skid-steer, orbit, torque, pivot)
