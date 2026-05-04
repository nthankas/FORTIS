# FORTIS

Remotely operated inspection robot for the DIII-D tokamak fusion reactor at General Atomics. ECE 129 senior design at UCSC, target ship date mid-June 2026.

The robot enters the reactor through a 15.75" square access tunnel, descends via tether through the R0 port (22" x 35.5"), and drives on the reactor floor on four omni wheels. It carries a 4-DOF arm for camera inspection of plasma-facing components and for thermocouple application.

## Hardware

OnShape model and BOM are the source of truth; this table is a snapshot.

| Subsystem | Selection |
|---|---|
| Chassis | `13.082" x 8.54" x 6"` octagonal prism w/ 3" chamfered face diagonals at the four 45-degree corners; 40 lb total robot |
| Drive | X-drive: 4 omni wheels at the chamfered corners |
| Wheels | AndyMark 8" Dualie Plastic Omni (am-0463) |
| Drive motors | ODrive M8325s brushless x4, **direct drive** (no gearbox) |
| Motor controllers | ODrive S1 x4 over CAN |
| Arm | 4-DOF parallel-link, 30" CF, always 3 lb payload (build target: v3) |
| Arm motors | NEMA 17 + Cricket 25:1 (J1/J3); NEMA 23 + EG23-G20-D10 20:1 (J2); Hitec D845WP servo (J4) |
| Compute | Jetson Orin Nano Super |
| Cameras | OAK-D Pro (depth, on arm L4 midpoint), chassis-mounted RGB |
| Power | Tether-supplied (no onboard battery) |

## Repo layout

```
.devcontainer/      VSCode dev container
.gitattributes      Force LF on text files (cross-machine hygiene)
docker/             Dockerfile.dev + docker-compose.yml (ROS 2 Humble desktop)
docs/adr/           Architectural decision records
src/                ROS 2 packages (colcon workspace)
  fortis_safety/    Mission state machine + ROS node + REPL console
  fortis_msgs/      Custom message + action types
  fortis_comms/     X-drive kinematics library (+ interim test helpers)
  fortis_drive/     X-drive ROS node, gated by mission state
  fortis_arm/       Arm controller seam (action + gripper services), scaffold
  fortis_integration_tests/   Cross-package launch_testing
sim/                Isaac Sim 5.1 work (Windows host, outside the container)
  isaac/xdrive/     Canonical chassis + arm sims, tools, deprecated/, results
analysis/           Drivetrain and arm analysis writeups
legacy/             Reference code from earlier design iterations
```

## Dev container

VSCode "Dev Containers: Reopen in Container" is the supported workflow. The image is ROS 2 Humble desktop with MoveIt 2, ros2_control, robot_localization, foxglove_bridge, and xacro pre-installed. First build is ~10 min.

Outside VSCode:

```bash
docker compose -f docker/docker-compose.yml up -d
docker exec -it fortis-dev bash
```

The repo mounts at `/workspace` inside the container.

## Workspace test pass

Inside the container:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm fortis_integration_tests
source install/setup.bash
colcon test --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm fortis_integration_tests
colcon test-result --verbose
```

Expect 0 failures. Per-package run / test instructions live in `src/<pkg>/README.md`. The cross-package seam between safety and drive is exercised by `launch_test src/fortis_integration_tests/test/test_safety_drive_integration.py`.

## Status

| Subsystem | State |
|---|---|
| Dev container | working |
| `fortis_safety` | working; mission FSM + REPL console; end-to-end ROS round trip verified |
| `fortis_msgs` | working; 4 messages + 1 action |
| `fortis_comms` | X-drive kinematics in production; `odrive_s1.py` / `motor_base.py` / `ekf.py` are interim helpers slated for replacement by upstream packages (see `docs/adr/`) |
| `fortis_drive` | working; gated by mission state |
| `fortis_arm` | scaffold; action + gripper services gated by mission state. **IK / trajectory / Teensy serial deferred** -- planned shape is a thin gate over MoveIt 2's `MoveGroup`, see `docs/adr/` |
| `fortis_integration_tests` | safety-drive seam verified end-to-end |
| Isaac Sim — chassis + v3 arm | working, see `sim/README.md` |
| Isaac Sim — R0 port entry | not started |
| `fortis_description` (URDF) | not started; needed for MoveIt 2 + ros2_control |
| `fortis_perception`, `fortis_localization`, `fortis_bringup` | planned |

## Documentation

- `docs/adr/` -- architectural decision records (state representation, MoveIt 2 + state-gate shape, ros2_control wiring)
- `sim/README.md` and `sim/isaac/xdrive/CHANGELOG.md` -- simulation state and history
- `analysis/` -- drivetrain rationale (x-drive vs skid-steer, orbit, torque, pivot)
- `src/<pkg>/README.md` -- per-package contracts (topics, services, actions, gating rules)
