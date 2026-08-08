# FORTIS — Project Handoff

**Audience:** the General Atomics engineers who will own, run, and extend FORTIS after the UCSC ECE 129 team steps off.
**Repository at handoff:** `https://github.com/nthankas/FORTIS` (single repo, no submodules, ~6.3 MiB packed).
**Prepared:** 2026-08-08, against `main` @ `2764593`.

This document is the single entry point. It answers four questions:

1. [What is FORTIS and what does it actually do today?](#1-what-fortis-is)
2. [What state is every subsystem in, and what has been proven on hardware?](#3-current-state-by-subsystem)
3. [How do I stand the whole thing up on a computer that has never seen it?](#6-standing-up-a-new-workstation) (and [on the robot](#7-standing-up-the-jetson-robot-compute))
4. [How do we integrate it with General Atomics' network](#8-networking-configuration-and-remote-access) and [move the code across](#9-github-transfer)?

Everything else in the repo hangs off the [documentation index](#11-documentation-index). Where this document and a package README disagree, **this document is newer** — see [§12 Known gaps](#12-known-gaps-risks-and-open-items) for the specific stale files.

---

## 1. What FORTIS is

FORTIS is a remotely operated inspection robot for the **DIII-D tokamak** at General Atomics. It enters the reactor through a 15.75" square access tunnel, descends on a tether through the R0 port (22" × 35.5"), and drives on the reactor floor on four omni wheels. A 4-DOF arm carries a camera for inspection of plasma-facing components and for thermocouple application.

Design constraints that explain most of the architecture:

- **No onboard battery.** All power is tether-supplied.
- **The robot straddles a 4.5" step** between the inner and outer reactor floor. Skid-steer point turns while straddling that step were tested in 60 simulated configurations; only 3 completed (5%), all with dangerous tilt and drift. That rejection is why the drivetrain is a **holonomic X-drive** (four omni wheels at the chamfered 45° corners), documented in `sim/analysis/skid_steer_rejection.md`.
- **The operator is remote, over a cable.** The robot is tethered; the control surface is Foxglove Studio on the operator station talking to a `foxglove_bridge` WebSocket on the robot over the wired link. No ROS traffic crosses machines — see [§8.6](#86-why-dds-does-not-cross-the-network).
- **Nothing moves unless the mission state machine says it can.** A single latched topic, `/fortis/mission_state`, gates both drive and arm motion. This is the safety spine.

Ship target for the student team was mid-June 2026.

---

## 2. System architecture in one page

### Compute topology

```
  Operator laptop                         Robot (Jetson Orin Nano Super)
  ───────────────                         ─────────────────────────────
  Foxglove Studio  ──── WebSocket :8765 ──▶  foxglove_bridge
  (layouts in                                     │
   foxglove/*.json)                               │  (all ROS nodes co-located
                                                  │   in one Docker container,
  ssh / tailscale ssh ─────────────────────▶      │   ROS_LOCALHOST_ONLY=1)
                                                  ▼
                                   ┌──────────────────────────────┐
                                   │ fortis_safety  (mission FSM) │
                                   └───────┬──────────────┬───────┘
                                  gates    │              │  gates
                                   ┌───────▼──────┐  ┌────▼─────────┐
                                   │ fortis_drive │  │ fortis_arm   │
                                   └───────┬──────┘  └────┬─────────┘
                                           │              │
                            ros2_control / │              │ USB-CDC @ 1 Mbaud
                            odrive plugin  │              │
                                           ▼              ▼
                                  CAN @ 250 kbit      Teensy 4.1
                                  4× ODrive S1        (J1-J3 steppers,
                                  4× M8325s motors     J4 + gripper servos)
                                           │
                        5× Luxonis OAK cameras over USB ──▶ fortis_perception
```

### The safety spine

`fortis_safety/mission_state_machine.py` is a **pure-Python, table-driven FSM with no ROS or hardware dependencies**, wrapped by a thin ROS node. Nine states:

| State | Meaning | Drive allowed | Arm allowed |
|---|---|---|---|
| `IDLE` | Powered on, no mission active | no | no |
| `ORBIT` | Driving the floor, scanning for targets | **yes** | no |
| `TARGETING` | Target clicked; solving IK for an approach pose | no | no |
| `ARM_AT_VIEW` | Arm parked at view pose; operator picks observe vs grasp | no | **yes** |
| `INSPECT` | Camera-only inspection, no contact | no | **yes** |
| `PICK` | Closing on a grasp target | no | **yes** |
| `HOLDING` | Object grasped, awaiting operator decision | no | **yes** |
| `RETURN_HOME` | Driving back to home pose; arm stowing | **yes** | **yes** |
| `FAULT` | Fault latched. Only `RESET` **with operator ack** leaves | no | no |

Three rules an incoming engineer must internalise:

1. **Any state can fault.** A wildcard transition takes any state to `FAULT` on the `FAULT` event, no guard. `odrive_health_monitor_node` emits exactly this event on a drive-health True→False edge.
2. **Recovery is never automatic.** Leaving `FAULT` requires `RESET` *and* `ctx["operator_ack"] == True`. A drive that comes back healthy on its own does **not** clear the fault.
3. **No state received yet == not allowed.** During the bring-up window before the first `/fortis/mission_state` arrives, every gated request is rejected. This race is explicitly unit-tested in both `fortis_drive` and `fortis_arm`.

The state diagram is generated from the live transition table:

```bash
python3 -m fortis_safety.mission_state_machine > mission_states.mmd   # Mermaid
```

### Cross-cutting conventions

| Convention | Value | Why it matters |
|---|---|---|
| Latched QoS | `TRANSIENT_LOCAL` + `RELIABLE`, depth 1, via `fortis_comms.qos_profiles.latched_qos_profile()` | A QoS mismatch makes DDS **silently refuse to match**. Never hand-roll this profile. |
| Runtime DDS domain | `ROS_DOMAIN_ID=42` | Set in `docker-compose.yml`, `.env`, and CI. Every FORTIS machine on one network must agree. |
| DDS scope | `ROS_LOCALHOST_ONLY=1` | All nodes are co-located in one container; host networking would otherwise expose `tailscale0`/`docker0`/wifi and break multicast discovery. |
| Test DDS domains | drive 91, localization 95, perception 96, sim_support 97 (per-package `test/conftest.py`) | Lets `colcon test` run packages in parallel without cross-talk. |
| Wheel order | always `fl, fr, rl, rr` | Message fields, controller arrays, config YAML. |
| CAN node_id | chain position: FL=0, FR=1, RR=2, RL=3 | Wiring order **is** the addressing. Note RR before RL. |
| Robot "front" | `base_link` **−X** | The face carrying the front camera; orbit faces the center with −X. |

---

## 3. Current state by subsystem

Legend: **HW** = exercised on real hardware · **SIM** = verified against synthetic/recorded data only · **DES** = designed and unit-tested, not yet exercised end to end.

| Subsystem | State | Evidence |
|---|---|---|
| Dev environment | **HW** — CPU `fortis-dev` is the default for everyone; `fortis-dev-gpu` (Isaac ROS base) opt-in on GPU machines and the Jetson | `docker/`, `./stack`, CI |
| `fortis_safety` | **HW** — mission FSM + REPL console + ODrive health monitor + `odrive_status → OdriveHealth` bridge; end-to-end ROS round trip verified | 4 test files, integration tests |
| `fortis_msgs` | **HW** — 8 messages, 2 services, 1 action, all consumed by live nodes | `ros2 interface show` |
| `fortis_comms` | **HW (combined path)** — X-drive IK/FK + shared QoS. See the H-matrix caveat in [§12](#12-known-gaps-risks-and-open-items) | URDF-sync drift test |
| `fortis_drive` | **HW** — bench-verified against the real drivetrain. Gating, watchdog timing, and signs are **frozen** absent a new hardware session | `tools/xdrive_bringup.md`, orbit `omega_sign` bench-verified 2026-06-11 |
| `fortis_control` | **HW** — `<ros2_control>` xacro + controller YAMLs + bench-one-motor and full-chassis launches via `odrive_ros2_control` | `tools/odrive_calibrate.md` runbook |
| `fortis_arm` | **Mixed** — `teensy_bridge` (USB-CDC link, heartbeat, `ArmStatus`, services) and `arm_controller` gripper gating are **HW** (gripper verified live); analytic 4-DOF IK + `MoveToPose` action server are **DES** (full-mission acceptance test passes, not yet driven against a fully wired arm) | `firmware/teensy/HANDOFF.md`, `tools/mock_teensy.py` |
| `fortis_bringup` | **HW** — `bringup` / `perception` / `sim` / `teleop` / `oak_chassis_cameras` / `chassis_orbit` compositions all live | launch tests |
| `fortis_description` | **DES** — 26 links / 25 joints, hand-written xacro primitives, clean tree (no loop closures, no orphans). Built from CAD, **not** the OnShape native exporter | `src/fortis_description/README.md` |
| `fortis_perception` | **SIM** — full chain (clouds → fusion → voxel map + cross-run diff → VO → detection → targeting → health) runs hardware-free against the synthetic rig. **VO benchmarked on real data:** TUM `freiburg1_xyz`, ATE RMSE 0.052 m over an 8.01 m path (**0.7%**), zero tracking losses | `test_rgbd_vo_tum.py` |
| `fortis_localization` | **SIM** — wheel odometry + IMU gyro debias + `robot_localization` EKF (`ekf.yaml` / `ekf_vio.yaml`) | 3 test files |
| `fortis_sim_support` | **SIM** — procedural RGBD scene + raycaster + `oak_replayer` publishing the exact depthai-v3 topic contract for 1–4 chassis mounts | the hardware-free source behind all perception tests |
| `fortis_integration_tests` | **SIM** — 6 cross-package `launch_testing` suites: safety↔drive, safety↔arm, bringup launch, perception chain, multicam fusion, full mission | `src/fortis_integration_tests/test/` |
| Teensy firmware | **Partial HW** — compiles (Arduino IDE 2.3.8, 83 KB flash), flashed to a real Teensy 4.1, `EVT_BOOT` + watchdog-fault frames observed on USB serial @ 1 Mbaud. **Bench bring-up with drivers and servos wired is not complete** | `firmware/teensy/HANDOFF.md` §7, §10 |
| Isaac Sim | **SIM** — canonical chassis (`xdrive_realwheel.py`) + v3 arm script + v4 5k-sample Monte Carlo torque/tipping sweep | `sim/README.md`, `sim/isaac/xdrive/CHANGELOG.md` |
| Isaac Sim — R0 port entry | **Not started** | plan in `sim/isaac/xdrive/docs/R0_ENTRY_PLAN.md` |
| Isaac ROS (cuVSLAM, nvblox, cuMotion) | **Staged, not wired** — preinstalled in `fortis-dev-gpu`, invoked by **no** package under `src/` | `docker/README.md` |

### Test baseline

The documented baseline on `main` is **68 tests, 0 failures, 6 skipped** (down from 126/0/8 before lint moved out of `colcon test` into pre-commit). Lint — flake8, pydocstyle, xmllint, copyright — runs via pre-commit locally and as a gating CI job, **not** via `colcon test`.

> This number is carried from the repo's own README; it could not be re-run while preparing this document (no ROS 2 install in the authoring environment). **Re-run it as the first acceptance step on the new machine** — see [§6.5](#65-build-and-test).

---

## 4. Repository map

```
.devcontainer/       VS Code dev container (CPU profile)
.github/workflows/   ci.yml — pre-commit gate, then colcon build + test on Humble
.gitattributes       Forces LF on all text files (cross-OS hygiene; do not remove)
.pre-commit-config.yaml
docker/              Dockerfile.dev (CPU) + Dockerfile.dev-gpu (Isaac ROS) + compose + can-up.sh
firmware/teensy/     Teensy 4.1 arm firmware: teensy.ino, PROTOCOL.md, HANDOFF.md,
                     vendored TeensyStep4, per-joint bench sketches under tests/
foxglove/            Operator layouts: xdrive_teleop, chassis_cams, perception, arm
sim/                 Isaac Sim 5.1 work (runs on a Windows host, OUTSIDE the container)
  isaac/xdrive/      canonical/ · tools/ · lib/ · assets/ · results/ · docs/
  analysis/          drivetrain + arm rationale writeups
src/                 ROS 2 Humble colcon workspace — 12 packages
  fortis_safety/          mission FSM, health monitor, odrive status bridge, REPL console
  fortis_msgs/            8 msgs, 2 srvs, 1 action (ament_cmake)
  fortis_comms/           X-drive kinematics + shared QoS profiles
  fortis_drive/           drive_node, drive_enable, heading_hold, orbit
  fortis_arm/             teensy_bridge, arm_controller, arm_motion (IK + MoveToPose)
  fortis_bringup/         all top-level launch composition + params
  fortis_description/     URDF/xacro + RViz config
  fortis_control/         ros2_control wiring for the X-drive
  fortis_perception/      clouds, fusion, voxel map + diff, RGBD VO, detection, targeting, health
  fortis_localization/    wheel odometry, IMU debias, robot_localization EKF
  fortis_sim_support/     synthetic RGBD scene + raycaster + OAK replayer
  fortis_integration_tests/  cross-package launch_testing
stack                Operator CLI (bash) — the preferred entry point
tools/               odrive_calibrate.md, xdrive_bringup.md, odrive_autotune.py,
                     mock_teensy.py, kinematic_calibration.py, vendor_import.sh,
                     vendor_repos.yaml, stack/ (docs + tests + .env.example)
docs/                This handoff document
```

**Not in git, by design:** `.env` (per-machine secrets/config), `src/ros_odrive/` (vendored by pin, see [§6.4](#64-vendor-the-upstream-odrive-packages)), `build/ install/ log/`, `.scratch/` (datasets), STL/OBJ/FBX binaries.

---

## 5. Hardware inventory

The **OnShape model and `FORTIS_FINAL_BOM` are the source of truth.** This table is a snapshot for orientation.

| Subsystem | Selection |
|---|---|
| Chassis | 13.082" × 8.54" × 6" octagonal prism, 3" chamfered face diagonals at the four 45° corners; ~45 lb total robot |
| Endoskeleton | 6061-T6 aluminium 0.5" square tube (McMaster 6546K49); 0.25" sheet for heavy mounting; 0.040" skin panels, Plastidip coated |
| Drive | X-drive, 4 omni wheels at the chamfered corners |
| Wheels | 4× AndyMark 8" Dualie Plastic Omni (am-0463), 80A durometer, 120 lb each |
| Drive motors | 4× ODrive M8325s 100 KV outrunner BLDC, **direct drive, no gearbox**, 48 V |
| Motor controllers | 4× ODrive S1 FOC (KIT-S1-M8325s-01) |
| Drive bus | CAN daisy-chain, 250 kbit; ODrive USB-CAN adapter on Jetson USB Port 2; adapter has built-in 120 Ω termination |
| Arm | 4-DOF parallel-link, 30" carbon fibre (1" ID × 1⅛" OD pultruded square tube), 3 lb payload |
| J1 base yaw | NEMA 17 closed-loop 65 Ncm (17HS24-2104-ME1K) + Cricket Drive MK II 25:1 cycloidal |
| J2 shoulder | NEMA 23 (CN-23HS22-2804-HG50-ME1K) + 50:1 planetary + Ruland FHD-MCL-14-F collar |
| J3 elbow | NEMA 17 closed-loop (same as J1) + Cricket Drive MK II 25:1 cycloidal |
| J4 wrist | Hitec D845WP servo (180 oz-in @ 7.4 V, waterproof), PWM from Teensy |
| Gripper | ServoCity 3219-0001-0002 parallel gripper kit (75 oz-in @ 6 V), PWM from Teensy |
| Stepper drivers | 3× StepperOnline CL57T-V41 closed-loop (0–8 A, 18–50 VDC), powered from the 48 V bus |
| Level shifting | TI TXS0108E 8-ch bidirectional (3.3 V ↔ 5 V) for STEP/DIR/ENA and ALM |
| Arm controller | Teensy 4.1 (600 MHz Cortex-M7), TeensyStep4 for coordinated J1/J2/J3, HW PWM for J4 + gripper, USB serial to Jetson Port 4 |
| Compute | Jetson Orin Nano Super Dev Kit (945-13766-0000-000), 8 GB, 67 TOPS |
| Storage | Crucial T500 1 TB NVMe + 64 GB microSDXC boot card |
| Cameras | 4× Luxonis OAK-D Lite A00483 (left/right toroidal depth, rear, front angled 30–45° up) + 1× OAK-D Pro A00546 on arm L4 midpoint |
| Jetson USB map | 1: OAK-D Pro · 2: USB-CAN (drive) · 3: Coolgear hub (4× OAK-D Lite) · 4: Teensy 4.1 |
| Power | Tether-supplied, no onboard battery |

### Teensy pin map (keep in sync with `teensy.ino` and `PROTOCOL.md` §1)

| Function | Pin |
|---|---:|
| J1 STEP / DIR / ALM | 2 / 3 / 22 |
| J2 STEP / DIR / ALM | 4 / 5 / 23 |
| J3 STEP / DIR / ALM | 6 / 7 / 20 |
| Shared driver ENABLE (active-low) | 9 |
| J4 servo (FlexPWM3.1.B) | 28 |
| Gripper servo (FlexPWM3.1.A) | 29 |
| Level-shifter OE (active-HIGH) | 14 |
| External E-STOP (active-low, pull-up) | 21 |
| Status LED | 13 |

---

## 6. Standing up a new workstation

This is the path for a General Atomics engineer's own laptop or desktop. It gets you a fully building, fully testing workspace and a **live perception demo with no FORTIS hardware attached** in under an hour, most of which is the first container build.

### 6.1 Prerequisites

| Requirement | Notes |
|---|---|
| **Linux, macOS, or Windows with WSL2** | The CPU container is multi-arch (x86_64 + aarch64). WSL2 works for everything except live OAK camera streaming — see the WSL2 caveat below. |
| **Docker 24+ with the Compose v2 plugin** | `docker compose version` must work. `docker-compose` (hyphenated v1) is not supported. |
| **git** | Any recent version. |
| **Python 3 + pip on the host** | Only for `pre-commit`. |
| *(optional)* **NVIDIA GPU + Container Toolkit** | Only if you want the `gpu` profile. Not required for anything currently wired into `src/`. |
| **Foxglove Studio** | **Required — this is the operator UI.** Free desktop download. Not optional: there is no other way to drive the robot. |

> **WSL2 caveat:** OAK-D cameras re-enumerate from a USB2 bootloader to a USB3 SuperSpeed runtime device during firmware upload. That transition currently breaks under Windows/WSL2 Docker, so **live camera streaming is a Jetson-only path**. This is exactly why `fortis_sim_support` exists — every perception node is developed and tested against synthetic sources first.

### 6.2 Clone and configure

```bash
git clone https://github.com/nthankas/FORTIS.git      # ← update to the GA URL after §9
cd FORTIS

cp fortis.env.example .env      # per-machine config; .env is gitignored
```

Edit `.env`. The values that matter on a fresh workstation:

```ini
FORTIS_REF=main            # git ref this machine tracks
COMPOSE_PROFILE=cpu        # cpu (default, universal) | gpu (NVIDIA + toolkit only)
ROS_DOMAIN_ID=42           # do not change casually — must match every FORTIS machine
COMPOSE_PROJECT_NAME=fortis
FORTIS_JETSON_HOST=        # fill in after §8 (Tailscale) — e.g. fortis@fortis-jetson
NGC_API_KEY=               # only for the gpu profile; leave blank and `docker login nvcr.io` by hand
```

Install the lint hooks on the host (one time per machine):

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files      # should pass clean on main
```

### 6.3 Bring the container up

```bash
./stack up        # builds on first run (~10 min for the CPU image), then starts it
./stack exec      # interactive bash inside the container; repo is mounted at /workspace
./stack status    # .env state, FORTIS_REF vs HEAD, tree cleanliness, docker compose ps
```

`./stack` is a bash wrapper around `git` + `docker compose`. It is a convenience layer, **not** a hard dependency — every command has a documented raw equivalent in `tools/stack/README.md` under "Fallback". The raw CPU equivalents are:

```bash
docker compose --env-file .env -f docker/docker-compose.yml up -d
docker exec -it fortis-dev bash
```

To run `stack` from anywhere, symlink it once: `ln -s "$(pwd)/stack" ~/.local/bin/stack` (it resolves symlinks to find the real repo root).

VS Code users can instead use **Dev Containers: Reopen in Container**, which uses `.devcontainer/devcontainer.json` → the same `cpu` compose service.

### 6.4 Vendor the upstream ODrive packages

Run once after cloning, and again any time `tools/vendor_repos.yaml` changes:

```bash
# inside the container
cd /workspace
./tools/vendor_import.sh
```

This wraps `vcs import src < tools/vendor_repos.yaml`, cloning `odriverobotics/ros_odrive` at a **pinned commit** (`8ed510b`, 2026-04-21) into `src/ros_odrive/`, then prunes the two packages FORTIS does not build (`odrive_node`, `odrive_botwheel_explorer`). What remains is `odrive_ros2_control` (the ros2_control SystemInterface plugin `fortis_control` loads) and `odrive_base`.

**Pin policy: every entry pins a commit hash, never a branch.** Bumping the pin is a deliberate review-time decision and should follow a bench retest, not be done opportunistically. `src/ros_odrive/` is gitignored — the pin in the YAML is the source of truth, and committing the clone would freeze it twice.

> **GA network note:** this step requires reaching `github.com`. If outbound git is restricted, mirror `odriverobotics/ros_odrive` at that commit to an internal remote and change the `url:` in `tools/vendor_repos.yaml`.

### 6.5 Build and test

```bash
# inside the container, from /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

**Expected: 68 tests, 0 failures, 6 skipped.** The skips are dataset- and hardware-gated (e.g. the TUM RGB-D benchmark skips unless the sequence is present under `.scratch/tum/`).

To scope the build to the core stack without perception:

```bash
colcon build --symlink-install --packages-select \
    fortis_msgs fortis_comms fortis_safety fortis_drive fortis_arm \
    fortis_bringup fortis_description fortis_control fortis_integration_tests
```

`fortis_control` ships no tests of its own; it is in the build set so its YAML and launch install paths are exercised on every build.

### 6.6 Prove it works — hardware-free demos

**(a) Perception chain against the synthetic rig.** No cameras, no robot:

```bash
ros2 launch fortis_bringup perception.launch.py
# Foxglove → Open connection → Foxglove WebSocket → ws://localhost:8765
# Import foxglove/fortis_perception.json
```

An `oak_replayer` renders a procedural RGBD scene along a 1 m orbit and publishes the exact depthai-v3 topic contract of a real OAK-D Lite. Downstream, you get per-camera clouds, a fused cloud in `base_link`, a persistent voxel map in `odom`, HSV-blob detection, click-to-target, and `/diagnostics`.

**(b) The cross-run change detector** — this is the mission-relevant capability (find what changed in the reactor between inspections):

```bash
# Run 1 — baseline scene. Let the map accumulate ~1 min, then save:
ros2 launch fortis_bringup perception.launch.py scene:=baseline
ros2 service call /voxel_map/save_map fortis_msgs/srv/SaveMap "{path: '/tmp/baseline.npz'}"

# Run 2 — modified scene, diffed against the saved reference:
ros2 launch fortis_bringup perception.launch.py scene:=modified reference_map:=/tmp/baseline.npz
# /fortis/perception/map_diff/markers  → green = added, red = removed
# /fortis/perception/map_diff/summary  → voxel counts + volumes
```

The synthetic scenes differ by exactly one added 0.4 m box (0.064 m³) and one removed 0.35 m box (0.042875 m³), both module constants, so the diff volumes are assertable.

**(c) Drive the mission FSM by hand.** This is how you learn the safety spine:

```bash
ros2 run fortis_safety mission_state_node          # terminal 1
ros2 topic echo /fortis/mission_state              # terminal 2
ros2 run fortis_safety event_console               # terminal 3
[IDLE] fortis> event start_orbit
[ORBIT] fortis> set target_pose_valid true
[ORBIT] fortis> event chassis_cam_click
[TARGETING] fortis> set ik_ok true
[TARGETING] fortis> event arm_at_view_pose
[ARM_AT_VIEW] fortis>
```

**(d) Arm stack with no Teensy attached:**

```bash
python3 tools/mock_teensy.py --verbose        # prints a slave pty path
ros2 launch fortis_bringup perception.launch.py arm:=true serial_port:=/dev/pts/N
```

`mock_teensy.py` is a pure-Python pty simulator implementing the same wire protocol (0xA5 start, CRC-16-CCITT over `[SEQ, TYPE, PAYLOAD]`, 0x5A end). It supports `--inject-fault` for exercising host-side fault handling.

**(e) Drive path with no CAN hardware** — a `vcan` dry run proves the whole Foxglove → plugin → CAN transmit chain:

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 launch fortis_bringup drive_test.launch.py can_interface:=vcan0
candump vcan0     # real Set_Input_Vel frames (0x00D 0x02D 0x04D 0x06D) tracking your input
```

### 6.7 Foxglove layouts

Four ready-made layouts live in `foxglove/`. Import via **Studio → Layouts → Import from file…**; they then persist in Studio and can be re-imported on any machine.

| Layout | Contents |
|---|---|
| `fortis_xdrive_teleop.json` | Translate pad (fwd/back + strafe), Rotate pad, START ORBIT / STOP, wheel-velocity plot, mission-state readout, 3D view |
| `fortis_chassis_cams.json` | One camera per tab (only the active tab streams — deliberate bandwidth control) |
| `fortis_perception.json` | Clouds, fused cloud, voxel map, map-diff markers, detections, annotations |
| `fortis_arm.json` | Arm status, joint states, gripper controls |

---

## 7. Standing up the Jetson (robot compute)

The Jetson Orin Nano Super is the robot's only computer. Everything runs **in the container on the Jetson**; the operator laptop runs only Foxglove and ssh.

Repo path convention on the Jetson: **`/data/fortis_ws/src/FORTIS`** (the 1 TB NVMe, not the microSD boot card).

### 7.1 Base OS and Docker

1. Flash **JetPack 6.x** per NVIDIA's SDK Manager instructions. Boot from the microSDXC card; mount the Crucial T500 NVMe at `/data`.
2. Install Docker 24+ with the Compose plugin; add the operator user to the `docker` group.
3. Install the **NVIDIA Container Toolkit** and configure it:
   ```bash
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi   # verify
   ```
4. **NGC login** — required before the first GPU build. The Isaac ROS Dev Base image lives on `nvcr.io` and needs an account even though it is nominally public:
   ```bash
   docker login nvcr.io      # username: $oauthtoken, password: your NGC API key
   ```
   Free account at `ngc.nvidia.com`. Without this the GPU build fails at the `FROM` stage with `unauthorized`. **This is currently the blocker on GPU enablement for the Jetson.**

### 7.2 Repo and profile

```bash
sudo mkdir -p /data/fortis_ws/src && sudo chown -R $USER /data/fortis_ws
cd /data/fortis_ws/src
git clone https://github.com/nthankas/FORTIS.git    # ← GA URL after §9
cd FORTIS
cp fortis.env.example .env
```

In the Jetson's `.env`:

```ini
FORTIS_REF=main            # or a release tag for demo/production machines
COMPOSE_PROFILE=gpu        # the Jetson uses the Isaac ROS base image
ROS_DOMAIN_ID=42
FORTIS_JETSON_HOST=        # leave BLANK on the Jetson itself — `./stack ssh` no-ops correctly
```

Then:

```bash
./stack up        # builds fortis-dev-gpu (arm64-jetpack Isaac ROS base; 8–12 GB, 10–20 min)
./stack exec
./tools/vendor_import.sh && colcon build --symlink-install && colcon test
```

The Isaac ROS base is **hash-pinned** in `docker/Dockerfile.dev-gpu` (`ISAAC_ROS_HASH`), not semver-tagged. NGC garbage-collects old hashes on its own schedule, so a stale pin will eventually 404 at build time. The re-pin procedure is in that Dockerfile's header comment; treat it as a quarterly hygiene tick.

### 7.3 CAN bus

The committed interface is the **ODrive USB-CAN adapter** (`gs_usb`). JetPack 6's stock kernel lacked `gs_usb`; it is now installed on the Jetson via slamcore-dkms and auto-loads on plug-in, so the adapter enumerates as `canX` — **`can1` on the current Jetson**. The onboard Tegra CAN (`can0`, driver `mttcan`) also works if a transceiver is wired to the GPIO CAN pins, but it is **not** the committed path and is **not** wired to the ODrive chain.

The container brings CAN up automatically. `docker/can-up.sh` is the compose entrypoint; because the container is `network_mode: host` + `privileged`, configuring `can1` inside it configures the host interface. It is deliberately tolerant — a missing adapter logs a note and does not block startup. Override with env vars `FORTIS_CAN_IF` (default `can1`) and `FORTIS_CAN_BITRATE` (default `250000`).

Manual equivalent:

```bash
ip -br link                                        # find the interface
sudo ip link set can1 up type can bitrate 250000
sudo ip link set can1 txqueuelen 1000
candump can1                                       # expect 10 Hz heartbeats at 0x001 0x021 0x041 0x061
```

All four heartbeat IDs present == bus good. **A bitrate mismatch shows as zero frames, not an error** — that is the single most common CAN bring-up trap here.

### 7.4 ODrive calibration — required before any wheel turns

Each of the four S1s must be calibrated (motor + encoder, node_id assigned, cyclic CAN messages enabled, `save_configuration()` committed). **The `odrive_ros2_control` plugin refuses to close the loop on an uncalibrated motor.**

Full runbook: **`tools/odrive_calibrate.md`** (~5 min per motor, run `odrivetool` directly on the Jetson over ssh — native Linux means `/dev/ttyACM0` and udev rules just work, no Windows USB-over-WSL, no container, no GUI).

Smoke test per drive:

```bash
odrivetool
> odrv0.axis0.motor.is_calibrated     # True
> odrv0.axis0.encoder.is_ready        # True
> odrv0.axis0.config.can.node_id      # 0..3 per chain position
```

Gain tuning aids: `tools/odrive_autotune.py`, `tools/odrive_apply_gains.py`, `tools/odrive_vel_tune.md`.

### 7.5 First drive — wheels OFF the ground

Follow **`tools/xdrive_bringup.md`** end to end. Condensed:

```bash
ros2 launch fortis_bringup drive_test.launch.py can_interface:=can1
```

This brings up controller_manager + the ODrive plugin for all four wheels, the mission FSM, `drive_node`, the health monitor, and the Foxglove bridge. **No motor arms on launch** — the wheel controller loads `inactive` on purpose.

Verify, then arm from the Foxglove layout:

```bash
ros2 control list_hardware_interfaces   # 4 wheels: velocity command + pos/vel/effort state, all available
ros2 control list_controllers           # joint_state_broadcaster [active], wheel_velocity_controller [inactive]
```

- **ENABLE DRIVE** → `drive_enable_node` activates the controller; the four S1s enter `CLOSED_LOOP_CONTROL`; `/fortis/drive/armed` goes true.
- **START ORBIT** → FSM `IDLE` → `ORBIT`; `drive_node` now accepts `/cmd_vel`.

> **THE CRITICAL CHECK — do this with the wheels off the ground.** Command a slow pure rotation (ωz). The X-drive H-matrix's ω-column signs were inherited from the senior-design module and have **never been re-derived** from first principles against the URDF wheel yaws. The round-trip IK/FK unit tests **cannot** catch a sign-only error because it cancels through the pseudo-inverse. Expected directions:
>
> | Command | FL | FR | RL | RR |
> |---|---|---|---|---|
> | +Vx (fwd) | + | + | + | + |
> | +Vy (left) | + | − | − | + |
> | +ωz (CCW) | + | − | + | − |
>
> If it spins the wrong way or tries to translate, correct it at the **hardware boundary** (`WHEEL_DIRECTION` / `CMD_VEL_FRAME_SIGN` in `fortis_drive`), as the existing code does — **keep the H-matrix frozen**.

Teardown:

```bash
ros2 control switch_controllers --deactivate wheel_velocity_controller   # idles motors
# Ctrl-C the launch
sudo ip link set can1 down
```

### 7.6 Cameras

```bash
ros2 launch fortis_bringup oak_chassis_cameras.launch.py   # all connected OAK-D Lites (primary)
ros2 launch fortis_bringup oak_chassis_front.launch.py     # single front camera (debug)
ros2 launch fortis_bringup chassis_orbit.launch.py         # 4 cameras + drive stack + orbit + bridge
```

Cameras are **serial-pinned** to front/rear/left/right, run as independent depthai-ros v3 `Driver` nodes at 640×400 @ 15 fps, on-device MJPEG RGB + rgb-aligned 16UC1 depth, IMU on, NN off. Shared capture config: `src/fortis_bringup/config/oak_chassis_cameras.yaml`.

Each roster camera also gets a static identity TF (`<pos>_camera_link → oak_chassis_<pos>`) attaching its calibration TF tree to the URDF. **Unknown serials stay detached** — if you swap a camera, update the serial roster in the launch file or its TF will not attach.

### 7.7 Arm

```bash
ros2 launch fortis_bringup bringup.launch.py arm:=true serial_port:=/dev/ttyACM0
```

`teensy_bridge` owns the USB-CDC link (heartbeat, `ArmStatus`/`JointState`/diagnostics out; joint targets, enable/disable/home/clear_faults services in). `arm_controller` gates `open_gripper` / `close_gripper` by mission state. `arm_motion` solves analytic 4-DOF IK, publishes the latched `/fortis/context/ik_ok` flag, and serves the `MoveToPose` action.

**Before wiring drivers to motors**, work through `firmware/teensy/HANDOFF.md` §10 in order. Two items are non-negotiable:

- **Verify `PIN_DRV_ENABLE` polarity with a multimeter.** Whether the CL57T-V41 ENA optocoupler is common-anode or common-cathode determines it, and it is not yet confirmed.
- **Scope STEP/DIR at the TXS0108E B-side outputs before connecting any driver.** The TXS0108E B-side is open-drain; if edges look soft under capacitive load, add ~10 kΩ pull-ups on the 5 V side per TI's datasheet. This catches level-shifter wiring bugs without smoking a driver.

---

## 8. Networking, configuration, and remote access

### 8.1 What the stack actually requires from a network

Almost nothing. This is the single most important fact for anyone integrating FORTIS into an existing corporate network.

The robot needs **exactly two inbound TCP ports** from the operator's machine:

| Port | Protocol | Purpose |
|---|---|---|
| **22** | ssh | Launching, calibration, log pulls, `./stack` remote commands |
| **8765** | WebSocket (`foxglove_bridge`) | The entire operator interface — telemetry, teleop, click-to-target, service calls |

Nothing else on the robot has a network identity at all. The reason is structural, not incidental:

- **DDS never leaves the machine.** The container sets `ROS_LOCALHOST_ONLY=1`; all ROS nodes are co-located in one container. There is no multi-machine ROS graph to configure, no discovery peers, no DDS tuning, no multicast requirement. See [§8.6](#86-why-dds-does-not-cross-the-network).
- **Every sensor and actuator is a local bus.** Cameras are USB, the drivetrain is CAN, the arm is USB-CDC serial. None of them is IP-attached.
- **No cloud dependency at runtime.** Network access is needed to *build* (apt, Docker registries, the pinned `ros_odrive` clone) and never to *run*.

So integrating FORTIS with any network — Tailscale, a GA corporate VPN, a jump host, or a bare Ethernet cable — is the same job: **make TCP 22 and 8765 reachable, then set two strings.**

### 8.2 The complete configuration surface

All per-machine configuration lives in **one file at the repo root**: copy the tracked template `fortis.env.example` to `.env` (gitignored) and edit it. That single file carries every address, port, device node, and mode, and it feeds all three consumers — `./stack`, the compose files, and the ROS launch-argument defaults.

There are four configuration layers in total, and only the first contains anything machine- or network-specific.

| Layer | File | Scope | In git? |
|---|---|---|---|
| **1. Per-machine environment** | `.env` (from `fortis.env.example`) | Which ref this machine runs, cpu/gpu profile, DDS domain, **the Jetson's address** | **No** — gitignored |
| **2. Container runtime** | `docker/docker-compose.yml` (+ `.gpu.yml`) | `network_mode: host`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`, `DISPLAY`, `/dev` passthrough, CAN entrypoint | Yes |
| **3. ROS node parameters** | `src/fortis_bringup/config/bringup_params.yaml` | Per-node tunables (heading-hold PID, orbit speed/radius, IMU debias). Mixed live/documentation — the file header says which blocks are actually read | Yes |
| **4. Launch arguments** | CLI at `ros2 launch` time | `can_interface`, `port`, `serial_port`, `cameras`, `detector`, `synthetic`, `vio`, … | Yes (defaults) |

Layers 1 and 4 are connected: `can_interface`, `port`, and `serial_port` take their defaults from `FORTIS_CAN_IF`, `FORTIS_FOXGLOVE_PORT`, and `FORTIS_TEENSY_PORT` in `.env`, so setting them once per machine is enough. Precedence is *built-in default < `.env` < explicit `arg:=value`*, so a one-off override on the command line always wins without editing anything.

> **Raw `docker compose` needs `--env-file .env`.** Compose resolves its project directory to the compose file's parent (`docker/`) and therefore does not auto-discover the repo-root `.env`; without the flag it silently uses built-in defaults. `./stack` exports the values itself and is unaffected.

**Every network-addressable value in the entire repo:**

| Value | Where it lives | Default | Notes |
|---|---|---|---|
| `FORTIS_JETSON_HOST` | `.env` (layer 1) | *(blank)* | **The only host/address string the codebase reads.** Consumed solely by `./stack ssh`. Accepts anything ssh accepts: bare IP, DNS name, `user@host`, or a `~/.ssh/config` alias. Blank on the Jetson itself, where `./stack ssh` correctly no-ops. |
| Foxglove bridge port | `drive_test.launch.py`, `sim.launch.py` → `port:=` launch arg | `8765` | Overridable at launch. |
| Foxglove bridge port | `perception.launch.py` → `port:=` launch arg, default from `FORTIS_FOXGLOVE_PORT` | `8765` | Was a hardcoded module constant; now a launch arg like the others. |
| Foxglove bridge bind address | `perception.launch.py` `_foxglove()` | `0.0.0.0` | Binds **all** interfaces. See the security note in [§8.4](#84-before-you-expose-the-bridge-on-a-corporate-network). |
| Foxglove Studio connection URL | Typed into Studio by the operator | — | **Not in the repo at all.** |
| `ROS_DOMAIN_ID` | `.env` + both compose files + CI | `42` | A DDS domain, **not** a network address. Irrelevant across machines given `ROS_LOCALHOST_ONLY=1`. |
| `FORTIS_CAN_IF` / `FORTIS_CAN_BITRATE` | `.env` → compose → `docker/can-up.sh`; also the `can_interface` launch-arg default | `can1` / `250000` | SocketCAN, not IP. |
| `FORTIS_TEENSY_PORT` | `.env` → compose → `serial_port` launch-arg default | `/dev/ttyACM0` | USB-CDC device node, not IP. |

That is the whole list. **Two strings** carry the deployment's networking: `FORTIS_JETSON_HOST` in `.env`, and the URL the operator types into Foxglove Studio.

### 8.3 The deployed path: a direct wired link

**FORTIS is a tethered robot. In the deployed configuration the Jetson is physically cabled to the operator station, so there is no VPN, no overlay, and no remote-access product in the operating path at all.** Everything in [§8.5](#85-tailscale-optional-for-remote-development) is a development convenience, not a deployment dependency.

On a direct link, "networking" is one decision — how the two ends get addresses:

| Approach | Setup | When to use |
|---|---|---|
| **Static IPs on both ends** | e.g. Jetson `192.168.10.2/24`, operator station `192.168.10.1/24`, no gateway, no DNS | **Recommended.** Deterministic, survives reboots in any order, no DHCP server to fail mid-mission. |
| Link-local (APIPA/mDNS) | Both ends auto-assign `169.254.x.x`; reach the robot as `fortis-jetson.local` | Zero config, but mDNS resolution is the flakiest part and adds a failure mode you do not need. |
| DHCP reservation | GA's network hands the Jetson a pinned lease | Only if the robot lives on a managed GA subnet rather than a point-to-point cable. |

With a static pair, the entire configuration is:

```ini
# operator station .env
FORTIS_JETSON_HOST=fortis@192.168.10.2
```
```
# Foxglove Studio → Open connection → Foxglove WebSocket
ws://192.168.10.2:8765
```

That is the whole networking integration. Give the operator station a second NIC (or a USB-Ethernet dongle) dedicated to the robot link, so the robot subnet stays separate from GA's corporate LAN — that also resolves the exposure concern in [§8.4](#84-before-you-expose-the-bridge-on-a-corporate-network), because the unauthenticated bridge is then only reachable from the one directly-cabled machine.

> **Open item — the tether's data path is not documented in this repo.** The BOM records the tether as *power only* ("Tether-supplied, no onboard battery"), and the Jetson's GbE port is listed but never described as tethered. Confirm with the mechanical team whether the tether carries an Ethernet pair, whether it is a separate umbilical, and what the run length is — Cat5e/Cat6 tops out at 100 m, and a reactor-to-console run plus service loop can approach that. If it does not fit, the options are a fibre media converter in the tether or a PoE extender chain. Flagged in [§12](#12-known-gaps-risks-and-open-items).

### 8.3.1 If the operator station is *not* the cabled machine

Only relevant if someone needs to reach the robot from elsewhere on GA's network — a second engineer, a remote observer, bring-up from a desk. Then:

1. **Make the Jetson reachable** on TCP 22 and 8765 from that machine, by whatever means GA's network team prefers. A static DHCP reservation or a DNS A record for the Jetson is worth asking for — it makes step 2 stable.
2. **Set `FORTIS_JETSON_HOST`** in each operator machine's `.env` to whatever now works:
   ```ini
   FORTIS_JETSON_HOST=fortis@fortis-jetson.robotics.ga.com   # corporate DNS
   FORTIS_JETSON_HOST=fortis@10.4.12.31                      # static lease
   FORTIS_JETSON_HOST=fortis-jetson                          # ~/.ssh/config alias
   ```
   The `~/.ssh/config` alias form is the most robust — it lets GA encode a `ProxyJump` bastion, a non-standard port, an identity file, or a `HostName` that changes, **without touching the repo at all**:
   ```sshconfig
   Host fortis-jetson
       HostName    10.4.12.31
       User        fortis
       ProxyJump   bastion.ga.com
       IdentityFile ~/.ssh/id_fortis
   ```
3. **Point Foxglove Studio** at `ws://<same-host>:8765`.

Three deployment shapes, all supported with no code change:

| Shape | `FORTIS_JETSON_HOST` | Foxglove URL | Notes |
|---|---|---|---|
| Direct cable / same VLAN | `fortis@10.4.12.31` | `ws://10.4.12.31:8765` | Simplest. Works in an air-gapped reactor area. |
| Corporate VPN | `fortis@fortis-jetson.ga.com` | `ws://fortis-jetson.ga.com:8765` | Operator joins the VPN; robot is a normal internal host. |
| Bastion / jump host | `fortis-jetson` (ssh alias w/ `ProxyJump`) | `ws://localhost:8765` **via an ssh tunnel** | See below. |

**Bastion case — the one that needs a tunnel.** If GA only permits ssh to the robot and not arbitrary inbound TCP, forward the bridge over the ssh session:

```bash
ssh -N -L 8765:localhost:8765 fortis-jetson      # leave running
# Foxglove Studio → ws://localhost:8765
```

This is often the *preferred* posture on a locked-down network: it gives you a single authenticated, encrypted, audited channel and closes the unauthenticated bridge problem described next.

### 8.4 Before you expose the bridge on a corporate network

On a dedicated point-to-point cable ([§8.3](#83-the-deployed-path-a-direct-wired-link)) this is largely moot — the only machine that can reach the bridge is the one plugged into it. Read this before the bridge becomes reachable from anything wider than that, including an operator station that is dual-homed onto GA's corporate LAN.

`foxglove_bridge` is launched with `tls: False`, `address: "0.0.0.0"`, and these capabilities: `clientPublish`, `parameters`, `parametersSubscribe`, `services`, `connectionGraph`, `assets`.

**That means anyone who can reach TCP 8765 can command the robot** — publish `/cmd_vel`, fire mission events, call the gripper services, change node parameters. There is no authentication and no transport encryption on that port. The mission FSM gate is a *safety* interlock against out-of-sequence commands; it is **not** an access-control mechanism, and it does not care who is connected.

That is an acceptable posture on a private overlay with per-node ACLs (which is what [§8.5](#85-tailscale-one-concrete-implementation) sets up). It is **not** acceptable on a general corporate LAN. Pick one before deployment:

- **ssh tunnel only** ([§8.3.1](#831-if-the-operator-station-is-not-the-cabled-machine)) — bridge binds locally, authentication is ssh's. Simplest hardening, no code change.
- **Host firewall on the Jetson** — `ufw`/`nftables` restricting 8765 to specific operator source addresses.
- **Overlay ACLs** — Tailscale/Headscale policy limiting 8765 to a tagged operator group.
- **Enable TLS + auth on the bridge** — `foxglove_bridge` supports `tls`, `certfile`, `keyfile`; wiring those (and narrowing `address` off `0.0.0.0`) is a small change to `_foxglove()` in `perception.launch.py`. Not done today.

Whichever GA picks, record it — this is the kind of decision that silently reverts during a demo scramble.

### 8.5 Tailscale (optional, for remote development)

**Not required for deployment.** The deployed robot is cabled ([§8.3](#83-the-deployed-path-a-direct-wired-link)). This section documents the student team's development setup and is offered only as a reference for the case where an engineer wants to reach the Jetson from a desk without moving the robot or re-cabling.

Tailscale gives the Jetson a **stable private address that does not change with the network it is plugged into** — the same `ssh` command and the same Foxglove URL work from the bench, a conference room, or home, with no firewall rules to negotiate per site. GA should feel free to skip it entirely, or substitute their own VPN per [§8.3.1](#831-if-the-operator-station-is-not-the-cabled-machine).

#### Set up the tailnet

1. **Create or choose a tailnet.** General Atomics should use a GA-owned tailnet (SSO-backed against the corporate IdP), not a personal account. The student team's tailnet should be considered temporary and torn down at handoff.
2. **Define tags before adding devices.** Tags let you write access policy about *roles* instead of *machines*, and a tagged device does not expire with a user account. In the tailnet's Access Controls:

   ```jsonc
   {
     "tagOwners": {
       "tag:fortis-robot":    ["group:fortis-admins"],
       "tag:fortis-operator": ["group:fortis-admins"]
     },
     "acls": [
       // Operators reach the robot on ssh and the Foxglove bridge only.
       {
         "action": "accept",
         "src":    ["tag:fortis-operator", "group:fortis-admins"],
         "dst":    ["tag:fortis-robot:22", "tag:fortis-robot:8765"]
       }
     ],
     "ssh": [
       {
         "action": "check",                       // re-auth periodically
         "src":    ["group:fortis-admins"],
         "dst":    ["tag:fortis-robot"],
         "users":  ["fortis", "root"]
       }
     ]
   }
   ```

   Adjust group names to GA's directory. The principle: the robot is a *destination*, operators are *sources*, and the only open ports are 22 and 8765.

#### Join the Jetson

```bash
curl -fsSL https://tailscale.com/install.sh | sh

sudo tailscale up \
  --hostname=fortis-jetson \
  --advertise-tags=tag:fortis-robot \
  --ssh                      # optional: Tailscale-brokered SSH, no key distribution
```

For unattended provisioning (reimaging, spares), generate a **pre-authorized key** in the admin console and use `--authkey=tskey-auth-...`. Make it **non-ephemeral** so the node survives reboots.

> **Do this or the robot will drop off the network mid-demo:** in the admin console, open the `fortis-jetson` node and **disable key expiry**. Tailscale node keys expire on a schedule by default (commonly ~180 days). An expired key on a robot that is physically inside a tokamak is a very bad day. Tagged devices are the standard way to hold this property; confirm it explicitly per node anyway.

Verify:

```bash
tailscale status
tailscale ip -4          # e.g. 100.x.y.z — stable for the life of the node
```

**Enable MagicDNS** in the tailnet settings so `fortis-jetson` resolves by name from every member device. Prefer the name over the 100.x address everywhere — it survives node re-creation.

#### Join operator machines and wire it into the stack

```bash
# on each operator laptop / workstation
curl -fsSL https://tailscale.com/install.sh | sh      # or the macOS/Windows installer
sudo tailscale up --advertise-tags=tag:fortis-operator
```

Then in that machine's `.env`:

```ini
FORTIS_JETSON_HOST=fortis@fortis-jetson
```

That single value powers the remote workflows:

```bash
./stack ssh                                            # interactive shell on the Jetson
./stack ssh "./stack status"                           # one-shot remote command
./stack ssh "./stack switch feat/some-branch && ./stack status"   # flip the robot to a branch, one line
```

`./stack ssh` detects when the local hostname already matches `FORTIS_JETSON_HOST` and no-ops with a friendly message, so **the same `.env` shape works on both the desk and the Jetson**.

Foxglove: **Open connection → Foxglove WebSocket → `ws://fortis-jetson:8765`**.

Manual equivalents, no `stack` involved:

```bash
ssh fortis@fortis-jetson
cd /data/fortis_ws/src/FORTIS && ./stack exec
# inside: source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
```

### 8.6 Why DDS does not cross the network

The container sets **`ROS_LOCALHOST_ONLY=1`**, and that is deliberate. All FORTIS ROS nodes are co-located in one container with `network_mode: host`. Host networking exposes every host interface (VPN tunnels, `docker0`, wifi) to DDS, which **breaks default multicast discovery between the co-located nodes** — the very nodes that must talk to each other. Loopback-only DDS is the correct setting here.

**Consequence:** you cannot run `ros2 topic echo` from your laptop against the robot's topics over *any* remote transport — VPN, overlay, or tunnel. Only the Foxglove WebSocket crosses machines. This is a feature, not a limitation to work around: one TCP port to firewall, one auditable surface, and no DDS discovery traffic on a corporate network. It is also what makes [§8.3](#83-the-deployed-path-a-direct-wired-link) a two-string change rather than a networking project.

If a future need genuinely requires cross-machine DDS, that is a design change, not a config tweak: it means dropping `ROS_LOCALHOST_ONLY`, moving to unicast/discovery-server DDS configuration, and accepting the DDS-over-tunnel MTU and multicast caveats (most corporate VPNs do not carry multicast at all). **Do not do it casually.**

### 8.7 Tailscale-specific network considerations

| Concern | Detail |
|---|---|
| Outbound requirements | Tailscale prefers direct UDP (default 41641) and falls back to **DERP relays over TCP 443** when UDP is blocked. It works behind restrictive egress filtering, at some latency cost. |
| Fully air-gapped reactor areas | Tailscale needs a control-plane reachable path for coordination. In a genuinely isolated area, fall back to a **direct LAN cable to the Jetson**: `ssh fortis@<lan-ip>` and `ws://<lan-ip>:8765`. Nothing in the stack cares. |
| Self-hosting | If GA policy forbids the hosted coordination server, **Headscale** (open-source control server) is a drop-in that speaks the same client protocol. Client setup above is unchanged apart from `--login-server`. |
| Do not commit addresses | Commit `2764593` scrubbed machine-specific IPs from tracked docs. Keep it that way: real hostnames and IPs belong in `.env` (gitignored), never in a tracked file. |

---

## 9. GitHub transfer

Today the repo is a **personal** repository: `https://github.com/nthankas/FORTIS`. Three viable ways to hand it to General Atomics, in decreasing order of fidelity.

### 9.1 Pre-transfer checklist — do this first, in this order

These items are cheap now and expensive or impossible after GA has cloned the repo.

- [ ] **Decide on history rewriting *before* transfer.** Commit `2764593` scrubbed machine-specific IPs from the *working tree*, but **the history still contains them.** Same for anything else ever committed and later removed. If GA's policy requires those out of the object store, do the rewrite (`git filter-repo`) **now** — a rewrite after transfer invalidates every clone GA has made and every SHA in their issue tracker.
- [ ] **Run a secret scan over full history**, not just the tip:
      ```bash
      pip install detect-secrets   # or use gitleaks / trufflehog
      gitleaks detect --source . --log-opts="--all"
      ```
      Known-good by design: `.env` is gitignored (it holds `NGC_API_KEY` and `FORTIS_JETSON_HOST`), and `fortis.env.example` is a template with blank values. Verify no real `.env` was ever committed.
- [ ] **Add a root `LICENSE` file.** Every one of the 12 packages declares `MIT` in `package.xml` and `setup.py`, but **there is no `LICENSE` at the repo root.** Any GA legal review will catch this. Add the MIT text with the correct copyright holder(s), or replace the per-package declarations if GA wants different terms — but resolve the mismatch before transfer, not after.
- [ ] **Confirm authorship and ownership.** Package maintainers are currently `Nikhil Thankasala <nikhilthankasala@gmail.com>` (and `FORTIS` for `fortis_control`). Decide whether these become GA addresses and update `setup.py` + `package.xml` together if so.
- [ ] **Verify no Git LFS and no huge blobs.** The repo is ~6.3 MiB packed, no LFS, no submodules. Largest tracked files are simulation artifacts (`omniwheels.usd` 4 MB, a couple of MC-sweep CSVs). Transfer is trivial — but confirm rather than assume.
- [ ] **Inventory external dependencies GA must be able to reach.** The repo does not vendor them:
      | Dependency | Source | Needed for |
      |---|---|---|
      | `odriverobotics/ros_odrive` @ `8ed510b` | github.com | `fortis_control` — pinned in `tools/vendor_repos.yaml` |
      | `osrf/ros:humble-desktop` | Docker Hub | CPU container |
      | `nvcr.io/nvidia/isaac/ros:<hash>` | NGC (account required) | GPU container |
      | `ros-humble-*` apt packages | ROS 2 apt archive | both containers |
      | YOLOv8n ONNX (optional) | fetched by `ros2 run fortis_perception download_models` | `detector:=yolo` only; the default `blob` backend needs no weights |
      If any are blocked, mirror them internally and update `tools/vendor_repos.yaml` / the Dockerfiles.
- [ ] **Check GA org Actions policy.** `.github/workflows/ci.yml` uses no secrets, but it does use four third-party actions — `actions/checkout@v4`, `actions/setup-python@v5`, `pre-commit/action@v3.0.1`, `ros-tooling/setup-ros@v0.7`. Many enterprise orgs allowlist actions; these must be permitted or the pipeline is dead on arrival.
- [ ] **Snapshot anything outside the repo** that the project depends on: the OnShape model, `FORTIS_FINAL_BOM`, the power BOM, `E:/Capstone/assets/halfReactor.stl` (referenced by the sim but deliberately not in git), and any Foxglove layouts saved only in Studio.

### 9.2 Option A — GitHub repository transfer (recommended)

Preserves commit history, issues, pull requests, releases, wiki, and installs redirects from the old URL so existing clones and links keep working.

1. GA org admin confirms the target org allows incoming transfers and that `nthankas` has permission to create a repo there (or invites them temporarily).
2. `nthankas` → repo **Settings → General → Danger Zone → Transfer ownership** → target org.
3. An org owner **accepts** the transfer.
4. Post-transfer, in the new repo:
   - Re-create **branch protection** on `main` (require the `pre-commit` and `colcon build + test` checks, require PR review). Protection rules do not always survive a transfer intact, and org-level rulesets may layer on top.
   - Add a **`CODEOWNERS`** file naming the GA owners of `src/`, `firmware/`, and `sim/`.
   - Re-add collaborators; GitHub does not carry personal-repo collaborators into an org's permission model.
   - Re-check **Actions secrets/variables** (currently none — but verify) and Actions enablement.
   - Confirm the default branch is still `main`.

**Caveats:** the transferring account loses admin unless re-added. Redirects work but are best-effort — update the URL everywhere anyway (next step).

### 9.3 Option B — mirror push to a fresh GA repo

Use when GA wants a clean break, a different name, or cannot accept a personal-account transfer.

```bash
git clone --mirror https://github.com/nthankas/FORTIS.git fortis-mirror.git
cd fortis-mirror.git
git remote set-url --push origin https://github.com/<GA-ORG>/<repo>.git
git push --mirror
```

Carries **all** branches, tags, and full commit history. Does **not** carry issues, pull requests, releases, or wiki — export those separately (GitHub's REST API or the migration/import tooling) if they matter.

### 9.4 Option C — offline bundle (air-gapped delivery)

A single verifiable file, no network between the two GitHub accounts:

```bash
git bundle create fortis-$(git rev-parse --short HEAD).bundle --all
sha256sum fortis-*.bundle > fortis.bundle.sha256
```

On the GA side:

```bash
git clone fortis-<sha>.bundle FORTIS
cd FORTIS && git remote set-url origin https://github.com/<GA-ORG>/<repo>.git && git push -u origin --all --tags
```

Pair the bundle with a `docker save` of `fortis-dev:latest` if GA cannot pull base images.

### 9.5 After the transfer — update every reference

The repo URL appears in tracked docs and on every machine. Sweep it:

```bash
grep -rn "nthankas/FORTIS" --exclude-dir=.git .
```

Then on **each machine** (workstations *and* the Jetson):

```bash
cd /path/to/FORTIS
git remote set-url origin https://github.com/<GA-ORG>/<repo>.git
git remote -v && git fetch origin && ./stack status
```

Also update: this document's header, the clone commands in [§6.2](#62-clone-and-configure) and [§7.2](#72-repo-and-profile), any CI badges, and any bookmarks. Finally, **archive the old personal repo** (Settings → Archive) so nobody keeps pushing to it — archiving preserves the redirect while making it read-only.

### 9.6 Post-transfer acceptance

- [ ] Fresh clone from the GA URL on a machine that has never seen the project.
- [ ] `./stack up && ./stack exec && ./tools/vendor_import.sh && colcon build && colcon test` → **68 / 0 / 6**.
- [ ] CI green on a trivial no-op PR against the GA repo's `main`.
- [ ] `pre-commit run --all-files` clean.
- [ ] Synthetic perception demo ([§6.6a](#66-prove-it-works--hardware-free-demos)) renders in Foxglove.
- [ ] A GA engineer reaches the Jetson over the GA tailnet: `ssh` **and** `ws://fortis-jetson:8765`.

---

## 10. Operating runbooks

| Task | Document |
|---|---|
| Bring the dev environment up on any machine | `tools/stack/README.md` |
| Container variants, GPU prereqs, Isaac ROS re-pin | `docker/README.md` |
| Calibrate the four ODrive S1s (required before any wheel turns) | `tools/odrive_calibrate.md` |
| Tune ODrive velocity gains | `tools/odrive_vel_tune.md`, `tools/odrive_autotune.py`, `tools/odrive_apply_gains.py` |
| First powered X-drive bench test, wiring → CAN → Foxglove → wheels | `tools/xdrive_bringup.md` |
| Teensy firmware scope, pin map, protocol contract, bench bring-up order | `firmware/teensy/HANDOFF.md` |
| Teensy wire protocol (frames, CRC, message IDs, payload layouts) | `firmware/teensy/PROTOCOL.md` |
| Kinematic calibration helper | `tools/kinematic_calibration.py` |
| Simulation state, canonical scripts, how to run Isaac Sim | `sim/README.md`, `sim/isaac/xdrive/CHANGELOG.md` |
| Why X-drive, orbit, torque, pivot, skid-steer rejection | `sim/analysis/*.md` |
| R0 port entry sim (planned) | `sim/isaac/xdrive/docs/R0_ENTRY_PLAN.md` |
| Per-package contracts: topics, services, actions, params, gating | `src/<pkg>/README.md` |

---

## 11. Documentation index

Start here → this file. Then, by role:

- **Software engineer joining the ROS stack:** root `README.md` → `src/fortis_safety/README.md` (the safety spine) → `src/fortis_drive/README.md` → the package you are touching.
- **Controls / hardware engineer:** `tools/odrive_calibrate.md` → `tools/xdrive_bringup.md` → `src/fortis_control/README.md`.
- **Firmware engineer:** `firmware/teensy/HANDOFF.md` → `firmware/teensy/PROTOCOL.md` → `teensy.ino`.
- **Perception engineer:** `src/fortis_perception/README.md` → `src/fortis_sim_support/README.md` → run the [§6.6](#66-prove-it-works--hardware-free-demos) demos.
- **Mechanical / simulation:** `sim/README.md` → `sim/analysis/` → the OnShape model and BOM.

---

## 12. Known gaps, risks, and open items

Ordered by "will bite you soonest".

### Blocking or safety-relevant

1. **The X-drive H-matrix ω-column signs have never been derived from first principles.** They were inherited from the senior-design module. Round-trip IK/FK tests cannot catch a sign-only error (it cancels through the pseudo-inverse). The *combined* pipeline is bench-verified with boundary corrections in `fortis_drive`, so the robot drives correctly today — but any refactor that touches `xdrive_kinematics.py` is unprotected by tests. **Keep the matrix frozen; correct signs at the hardware boundary.** Re-verify with the ωz check in [§7.5](#75-first-drive--wheels-off-the-ground) after any change.
2. **Teensy bench bring-up is incomplete.** `firmware/teensy/HANDOFF.md` §10 steps 3–10 (scope the level-shifter outputs, wire J1, force a driver fault, J2/J3, servos, EEPROM round-trip, e-stop loop, heartbeat watchdog) are not done. Two specific unknowns: **`PIN_DRV_ENABLE` polarity is unverified** (multimeter it before powering motors) and the **TXS0108E B-side is open-drain** (may need 5 V pull-ups). Do not energise steppers before working through that list in order.
3. **Stepper homing is not implemented.** `handleHomeRequest` still NAKs `ERR_NOT_IMPLEMENTED` for J1/J2/J3. J4 servo homing is done; the gripper still NAKs pending safe-range characterisation. Position recovery on boot reads a **last-known-position file**, and the planned reconciliation against the CL57T's reported position (refuse to act beyond a tolerance) is **not written yet**. Treat arm position after an unexpected power cycle as untrusted until that lands.
4. **Per-joint software limits are not defined.** `clamp_to_limits` in `CMD_SET_JOINT_TARGETS` is parsed and **ignored**. The URDF now carries revolute limits for J1–J4 — those are the canonical values to propagate into the firmware.
5. **The operator UI is a required deliverable and is only partially built.** This is not an optional nicety — Foxglove Studio plus the four committed layouts **is** the operator interface, and there is no fallback path to driving the robot without it.

   *What works today:* `foxglove/fortis_xdrive_teleop.json` (translate/rotate pads, START ORBIT / STOP, ENABLE DRIVE, wheel-velocity plot, mission-state readout, 3D view), `fortis_chassis_cams.json`, `fortis_perception.json`, `fortis_arm.json`. The full drive loop — arm the motors, open the mission gate, drive, orbit — is operable from the teleop layout with no CLI, and was bench-driven that way.

   *What is missing:* (a) a **mission-level UI** — advancing the FSM through `TARGETING` → `ARM_AT_VIEW` → `INSPECT`/`PICK` → `HOLDING` still requires `event_console`, which is explicitly a bring-up REPL and **not** a runtime component; (b) **fault presentation and the operator-ack RESET** have no UI surface, so today clearing a latched `FAULT` means publishing an ack topic by hand; (c) no **layout regression test** — the JSON layouts can silently drift from renamed topics, and nothing catches it.

   The seams the UI needs already exist and are stable: every FSM event is a `std_msgs/Empty` topic at `/fortis/events/<name>`, every guard is a `std_msgs/Bool` at `/fortis/context/<field>`, and state comes back latched on `/fortis/mission_state`. A Foxglove panel set can drive all of it with no new ROS code — the missing work is UI, not plumbing. `/fortis/mission_state_v2` (item 11) exists precisely to give that UI previous-state and transition timestamps; publishing it is a prerequisite for rendering transitions and detecting stalls.

6. **No `LICENSE` file at the repo root** despite all 12 packages declaring MIT. Resolve before transfer ([§9.1](#91-pre-transfer-checklist--do-this-first-in-this-order)).

### Functional gaps

7. **`fortis_description` is not yet integrated as the robot's live description.** The hand-written xacro tree is clean (26 links / 25 joints, no loop closures) and `fortis_comms` has a drift test pinning `LEN_X`/`LEN_Y`/`WHEEL_RADIUS` to it, but the earlier OnShape native export was unusable (95 links / 94 joints, closed loops, orphan links, no collision geometry, no joint limits) and the CAD→URDF track is not finished. Arm link lengths remain **TBD** pending mechanical confirmation.
8. **Isaac ROS is staged but wired to nothing.** `isaac_ros_visual_slam` (cuVSLAM), `isaac_ros_nvblox`, `isaac_ros_cumotion`(+MoveIt adapter), and `isaac_ros_image_proc` are preinstalled in `fortis-dev-gpu` and invoked by **no** package in `src/`. The CPU perception stack is the deployable default. GPU enablement on the Jetson is gated on NGC login ([§7.1](#71-base-os-and-docker)).
9. **MoveIt 2 is not wired.** `fortis_arm` uses analytic 4-DOF IK; there is no `fortis_moveit_config`, and `bringup.launch.py` has a placeholder TODO for the include.
10. **Perception is verified against synthetic data and one public dataset, not against the real reactor.** The RGBD VO number (0.7% ATE on TUM `freiburg1_xyz`) is real and good, but the reactor is a low-texture, specular, IR-hostile environment that nothing in the test set represents. **Expect VO and detection to need retuning on first real data**, and budget time for it.
11. **R0 port-entry simulation not started.** Plan exists (`sim/isaac/xdrive/docs/R0_ENTRY_PLAN.md`); descent through the 22" × 35.5" port is unmodelled.
12. **The tether's data path is unspecified in the repo.** The BOM records the tether as power-only; the Jetson's GbE is listed but never described as tethered. Since the deployed operator link is a physical cable ([§8.3](#83-the-deployed-path-a-direct-wired-link)), confirm with the mechanical team: does the tether carry an Ethernet pair, what is the run length, and does it stay inside Cat5e/Cat6's 100 m limit? If not, plan a fibre media converter or PoE extender before integration.
13. **`foxglove_bridge` runs unauthenticated and binds `0.0.0.0`.** `tls: False`, no auth, with `clientPublish` and `services` enabled — anyone who reaches TCP 8765 can command the robot. Acceptable on a dedicated cable, not on a shared LAN. See [§8.4](#84-before-you-expose-the-bridge-on-a-corporate-network) for the four ways to close it.
14. **`/fortis/mission_state_v2`** (`fortis_msgs/MissionState`, carrying previous state + transition timestamp) is defined but **not published** — the latched `std_msgs/String` topic is what ships.

### Documentation drift found while preparing this handoff

These are safe to fix and worth fixing early, because they mislead a newcomer:

15. **`src/fortis_arm/README.md` is stale.** It documents only the gripper-service stub ("gripper actuation not implemented") and lists IK, the Teensy serial protocol, position-file handling, and gripper actuation as "intentionally not in here." All of those have since landed (`teensy_bridge_node.py`, `arm_motion_node.py`, `arm_ik.py`, a live `MoveToPose` action server), and the gripper is verified on hardware. **Trust the code and the root README over this file.**
16. **`src/fortis_integration_tests/README.md` lists 3 of the 7 test files.** Missing: `test_perception_chain.py`, `test_multicam_fusion.py`, `test_full_mission.py`, and `test_bringup_launch.py` is described but the perception/mission coverage is absent.
17. **`fortis_msgs/README.md` and `fortis_bringup/README.md` both describe the ODrive health bridge as "TBD".** It shipped in `15943c3` as `fortis_safety/odrive_status_bridge_node.py`.
18. **`tools/stack/README.md` omits `stack build`,** which exists in the `stack` script.
19. **`firmware/teensy/HANDOFF.md` §7 predates the Jetson-side bridge.** It says the bridge is "not yet written"; `teensy_bridge_node.py` exists and the gripper has been driven through it live.

---

## 13. Ownership and contacts

| Area | Owner at handoff |
|---|---|
| ROS 2 stack, Jetson integration, simulation, URDF | Nikhil Thankasala — `nikhilthankasala@gmail.com` |
| Teensy firmware, protocol document, desktop mock | Cesar (per `firmware/teensy/HANDOFF.md`) |
| URDF / CAD cleanup fix list | Adrian, Carlos |

Boundaries the student team operated under, worth preserving:

- **`src/**`, `sim/**`, and `src/fortis_description/**` were ROS/sim scope**; `firmware/**` was firmware scope. Bugs crossing the boundary were flagged, not fixed unilaterally.
- **The Teensy wire protocol is a contract.** Frame layout, CRC polynomial (0x1021, init 0xFFFF, computed over `[SEQ, TYPE, PAYLOAD]`), the numeric values of `CMD_*`/`RSP_*`/`EVT_*`/`FAULT_*`/`ERR_*`, payload byte layouts, the 250 ms watchdog, and the ≤100 ms heartbeat cadence may not change silently. Appending new message types at the next free ID is fine; changing a layout means bumping `PROTO_MINOR`/`PROTO_MAJOR` in **both** `teensy.ino` and `PROTOCOL.md`, and updating the host side in the same change.
- **The pin map lives in two files** (`teensy.ino` `#define` block and `PROTOCOL.md` §1). Change both in one commit.
- **`drive_node`'s behaviour is bench-frozen.** Signs, gating, and watchdog timing are verified against the real drivetrain; change them only off the back of a new hardware session.

---

## 14. Handoff acceptance checklist

Sign this off and the handoff is complete.

**Code and access**
- [ ] Repository transferred ([§9](#9-github-transfer)); GA org owns it; old repo archived.
- [ ] `LICENSE` present and correct; maintainer fields updated.
- [ ] Branch protection + `CODEOWNERS` configured on the GA repo.
- [ ] CI green on the GA repo.
- [ ] Secret scan over full history clean.

**Environment**
- [ ] Fresh clone builds and tests on a GA workstation: **68 / 0 / 6**.
- [ ] `pre-commit run --all-files` clean.
- [ ] Synthetic perception demo renders in Foxglove ([§6.6a](#66-prove-it-works--hardware-free-demos)).
- [ ] Map-diff two-run workflow reproduces the added/removed volumes ([§6.6b](#66-prove-it-works--hardware-free-demos)).
- [ ] Mock-Teensy arm run comes up ([§6.6d](#66-prove-it-works--hardware-free-demos)).
- [ ] `vcan` dry run emits `Set_Input_Vel` frames ([§6.6e](#66-prove-it-works--hardware-free-demos)).

**Robot**
- [ ] Wired link up: operator station and Jetson have stable addresses ([§8.3](#83-the-deployed-path-a-direct-wired-link)); `ssh` and `ws://<jetson>:8765` both reachable across the tether.
- [ ] Tether data path confirmed with the mechanical team (Ethernet pair present, run length within spec).
- [ ] Decision recorded on bridge exposure ([§8.4](#84-before-you-expose-the-bridge-on-a-corporate-network)) — dedicated NIC, firewall, tunnel, or TLS.
- [ ] `./stack status` on the Jetson shows the expected ref and a clean tree.
- [ ] `candump can1` shows all four ODrive heartbeats.
- [ ] All four S1s report `is_calibrated` / `is_ready`.
- [ ] ωz rotation-direction check passes with wheels off the ground ([§7.5](#75-first-drive--wheels-off-the-ground)).
- [ ] All four OAK-D Lites enumerate and stream.

**Knowledge**
- [ ] A GA engineer has driven the FSM by hand through `event_console` and can explain the `FAULT` + operator-ack rule.
- [ ] A GA engineer has done one full bench cycle: CAN up → launch → arm → drive → teardown, driven entirely from the Foxglove operator layouts.
- [ ] Operator-UI scope agreed ([§12](#12-known-gaps-risks-and-open-items) item 5): who builds the mission-level panels and the fault/RESET surface.
- [ ] Off-repo artifacts handed over: OnShape model, `FORTIS_FINAL_BOM`, power BOM, reactor STL.
