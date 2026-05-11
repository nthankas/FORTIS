# FORTIS

Remotely operated inspection robot for the DIII-D tokamak fusion reactor at General Atomics. ECE 129 senior design at UCSC, target ship date mid-June 2026.

The robot enters the reactor through a 15.75" square access tunnel, descends via tether through the R0 port (22" x 35.5"), and drives on the reactor floor on four omni wheels. It carries a 4-DOF arm for camera inspection of plasma-facing components and for thermocouple application.

## Hardware

OnShape model and `FORTIS_FINAL_BOM` are the source of truth; this table is a snapshot.

| Subsystem | Selection |
|---|---|
| Chassis | `13.082" x 8.54" x 6"` octagonal prism w/ 3" chamfered face diagonals at the four 45-degree corners; ~45 lb total robot (chassis + arm + wheels per the v4 sim mass budget) |
| Chassis endoskeleton | 6061-T6 aluminum 0.5" square tube (McMaster 6546K49) |
| Chassis heavy mounting | 6061 aluminum 0.25" square sheet, 6"x48" (McMaster 9246K424) |
| Chassis skin panels | 6061-T6 aluminum 0.040" sheet, 12"x24" (McMaster 89015K22); Plastidip rubber coating |
| Drive | X-drive: 4 omni wheels at the chamfered corners |
| Wheels | 4x AndyMark 8" Dualie Plastic Omni (am-0463), no bearings, 80A durometer, 120 lb load cap each |
| Drive motors | 4x ODrive M8325s 100KV outrunner brushless, **direct drive** (no gearbox), 48V direct |
| Motor controllers | 4x ODrive S1 FOC controllers (KIT-S1-M8325s-01 from ODrive Robotics) |
| Drive bus | CAN, daisy-chained; ODrive USB-CAN Adapter at Jetson USB Port 2; built-in 120 ohm termination |
| Arm | 4-DOF parallel-link, 30" carbon fiber (Clearwater Composites 1" ID x 1 1/8" OD pultruded square tube), always 3 lb payload. Link lengths TBD — confirm with user. |
| Arm J1 (base yaw) | NEMA 17 closed-loop stepper 65 Ncm (17HS24-2104-ME1K, 2.1 A, 1000 PPR magnetic encoder, 3ch differential) + Cricket Drive MK II 25:1 cycloidal |
| Arm J2 (shoulder pitch) | NEMA 23 stepper + 50:1 high-precision planetary (CN-23HS22-2804-HG50-ME1K, StepperOnline) with 1000 PPR (4000 CPR) magnetic encoder + Ruland FHD-MCL-14-F shaft collar |
| Arm J3 (elbow pitch) | NEMA 17 closed-loop stepper (17HS24-2104-ME1K) + Cricket Drive MK II 25:1 cycloidal (same as J1) |
| Arm J4 (wrist) | Hitec D845WP servo (180 oz-in @ 7.4V, waterproof), PWM from Teensy |
| Gripper | ServoCity 3219-0001-0002 servo-driven parallel gripper kit (75 oz-in @ 6V, steel gears), PWM from Teensy |
| Arm stepper drivers | 3x StepperOnline CL57T-V41 closed-loop (0-8 A, 18-50 VDC, reads 1000 PPR encoder, HW closed-loop pos/vel/current). Powered from 48V bus. |
| Arm step/dir logic | 8-ch bi-directional bus transceiver (DIP-20) lifts Teensy 3.3V step/dir to 5V for CL57T optocouplers |
| Arm motion controller | Teensy 4.1 (600 MHz Cortex-M7), TeensyStep for 3-axis coordinated motion (J1/J2/J3), HW PWM for J4 + gripper, HW quadrature for encoder verify. USB serial to Jetson Port 4. Replaces the earlier Pololu Tics + PCA9685 plan. |
| Compute | Jetson Orin Nano Super Dev Kit (945-13766-0000-000), 8GB LPDDR5, 67 TOPS, 4x USB 3.2, GbE, 40-pin GPIO |
| Storage | Crucial T500 1TB NVMe (M.2 2280, PCIe Gen 4 x4) + 64GB Verbatim Premium MicroSDXC Class 10 / UHS-1 boot card |
| Cameras (5x) | 4x Luxonis OAK-D Lite (Active Focus) A00483 — left/right toroidal depth (primary + secondary VIO with redundant BMI270 6-axis IMUs and bilateral depth), rear depth (outward, extraction obstacle), front depth (angled 30-45 deg up at center column). 1x Luxonis OAK-D Pro (Active Stereo IR, Active Focus, Standard FOV) A00546 on arm L4 midpoint, BNO085 9-axis IMU, IP66, includes Y-adapter + USB cable. All 4 TOPS each. |
| Jetson USB assignment | Port 1: OAK-D Pro; Port 2: USB-CAN (drive); Port 3: Coolgear hub (4x OAK-D Lites); Port 4: Teensy 4.1 (arm) |
| Power | Tether-supplied, no onboard battery. Power BOM is a separate sheet. |
| Cable / winch | Deferred — TBD pending Sim4 |
| Idler hardware | 20x idler rollers + 20x shoulder screws (McMaster) |

## Repo layout

```
.devcontainer/      VSCode dev container
.gitattributes      Force LF on text files (cross-machine hygiene)
docker/             Dockerfile.dev + docker-compose.yml (ROS 2 Humble desktop); GPU variant in progress
docs/adr/           Architectural decision records
firmware/teensy/    Teensy 4.1 arm-motion firmware (main.ino, PROTOCOL.md, HANDOFF.md)
src/                ROS 2 packages (colcon workspace)
  fortis_safety/    Mission state machine + ROS node + REPL console
  fortis_msgs/      Custom message + action types
  fortis_comms/     X-drive kinematics library (+ interim test helpers)
  fortis_drive/     X-drive ROS node, gated by mission state
  fortis_arm/       Arm controller seam (action + gripper services), scaffold
  fortis_bringup/   Top-level launch composition (scaffold, stub launches only)
  fortis_description/  URDF / xacro + meshes + RViz config (scaffold, OnShape export pending cleanup)
  fortis_integration_tests/   Cross-package launch_testing
sim/                Isaac Sim 5.1 work (Windows host, outside the container)
  isaac/xdrive/     Canonical chassis + arm sims, tools, deprecated/, results
analysis/           Drivetrain and arm analysis writeups
legacy/             Reference code from earlier design iterations
```

## Dev environment

The dev environment ships as two container variants. Pick by machine, not by personal preference -- the CPU variant is the default for every teammate.

| Variant | Base image | Use when |
|---|---|---|
| `fortis-dev` | ROS 2 Humble desktop (CPU only, universal) | Default for all teammates regardless of whether a GPU is present. Runs on both x86_64 and aarch64 Docker hosts. |
| `fortis-dev-gpu` | Isaac ROS Common (NVIDIA, opt-in) | Requires an NVIDIA GPU and `nvidia-container-toolkit`. Used on the FORTIS PC, the IdeaPad (RTX 5050 Laptop), and the Jetson Orin Nano Super. |

Both images carry ROS 2 Humble desktop with MoveIt 2, ros2_control, robot_localization, foxglove_bridge, and xacro pre-installed. First build is ~10 min.

VSCode "Dev Containers: Reopen in Container" is the supported workflow for the CPU variant.

Outside VSCode:

```bash
docker compose -f docker/docker-compose.yml up -d
docker exec -it fortis-dev bash
```

The repo mounts at `/workspace` inside the container.

### `./stack` (preferred entry point)

`./stack` at the repo root is a small bash wrapper around `git` + `docker
compose` that turns the workflows above into one-line commands. It is the
preferred way to bring the dev environment up; the raw `docker compose`
calls still work and are the documented fallback.

```bash
cp .env.example .env
./stack up
./stack exec
./stack status
./stack switch feat/foo
./stack down
```

See [`tools/stack/README.md`](tools/stack/README.md) for the full command
reference and Jetson workflow.

### Pre-commit hooks

Lint (flake8 + pydocstyle + standard hygiene hooks) runs on every
commit via pre-commit. After cloning, install once on the host:

```bash
pip install pre-commit
pre-commit install
```

Run on the whole repo any time with `pre-commit run --all-files`.
Config is `.pre-commit-config.yaml`; flake8 reads workspace `.flake8`.

### CI

GitHub Actions runs `colcon build` + `colcon test` on every PR to
`main` (and every push to `main`). Lint runs in CI via
`pre-commit/action`, so the same checks that fire locally also gate
the PR. ROS 2 Humble + `ROS_DOMAIN_ID=42` (matches docker-compose).
Workflow lives in `.github/workflows/ci.yml`.

### Isaac ROS integrations (staged in `fortis-dev-gpu`, not yet wired into nodes)

The GPU image preloads these packages so they are available the moment a node consumes them. They are NOT currently invoked from any package under `src/`:

- `isaac_ros_visual_slam` -- cuVSLAM for the OAK-D Pro VIO front-end.
- `isaac_ros_nvblox` -- 3D scene reconstruction; output feeds the Nav2 costmap.
- `isaac_ros_cumotion` + `isaac_ros_cumotion_moveit` -- GPU motion planner with the MoveIt 2 adapter.
- `isaac_ros_image_proc` -- GPU-accelerated rectify / debayer / resize for the OAK-D streams.

## Workspace test pass

Inside the container:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm \
    fortis_bringup fortis_description fortis_integration_tests
source install/setup.bash
colcon test --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm \
    fortis_bringup fortis_description fortis_integration_tests
colcon test-result --verbose
```

`colcon test` now runs functional pytest / launch_testing only; lint
(flake8, pep257, copyright, xmllint) moved to pre-commit + the
`pre-commit` job in `.github/workflows/ci.yml`. Current baseline on
`main`: **68 tests, 0 failures, 6 skipped** (down from 126/0/8 prior
to the lint-out refactor). Per-package run / test instructions live
in `src/<pkg>/README.md`.
The cross-package seam between safety and drive is exercised by
`launch_test src/fortis_integration_tests/test/test_safety_drive_integration.py`.

## Status

| Subsystem | State |
|---|---|
| Dev environment | working; CPU `fortis-dev` is the default, opt-in `fortis-dev-gpu` (Isaac ROS Common base) staged for FORTIS PC / IdeaPad / Jetson |
| `fortis_safety` | working; mission FSM + REPL console; end-to-end ROS round trip verified |
| `fortis_msgs` | working; 4 messages + 1 action |
| `fortis_comms` | X-drive kinematics in production; `odrive_s1.py` / `motor_base.py` / `ekf.py` are interim helpers slated for replacement by upstream packages (see `docs/adr/`) |
| `fortis_drive` | working; gated by mission state |
| `fortis_arm` | scaffold; action + gripper services gated by mission state. **IK / trajectory / Teensy serial deferred** -- planned shape is a thin gate over MoveIt 2's `MoveGroup`, see `docs/adr/`. Firmware-side skeleton + protocol live under `firmware/teensy/`. |
| `fortis_bringup` | scaffold; stub launch files for `bringup` / `sim` / `teleop`, no node includes yet (`chore/package-scaffolding` merged) |
| `fortis_description` | scaffold; first OnShape URDF export landed but requires cleanup before integration (95 links / 94 joints, naming + topology issues; Adrian + Carlos have the fix list). URDF authoring planned as a dual track: chassis from OnShape cleanup, arm hand-authored xacro |
| `fortis_integration_tests` | safety-drive seam verified end-to-end |
| Isaac Sim | chassis (`xdrive_realwheel.py`) + v3 arm canonical script + v4 Monte Carlo sweep tooling -- see `sim/README.md` |
| Isaac Sim — R0 port entry | not started |
| `fortis_perception`, `fortis_localization` | planned |
| Cable / winch design | deferred pending Sim4 |

## Documentation

- `docs/adr/` -- architectural decision records (state representation, MoveIt 2 + state-gate shape, ros2_control wiring)
- `sim/README.md` and `sim/isaac/xdrive/CHANGELOG.md` -- simulation state and history
- `analysis/` -- drivetrain rationale (x-drive vs skid-steer, orbit, torque, pivot)
- `src/<pkg>/README.md` -- per-package contracts (topics, services, actions, gating rules)
