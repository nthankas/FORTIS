# CLAUDE.md

Operating context for Claude Code sessions in this repo. Read this first.

## Project

FORTIS is a remotely operated inspection robot for the DIII-D tokamak fusion reactor at General Atomics. It enters through a 15.75" x 15.75" access tunnel, descends via tether through the R0 port (22" x 35.5"), and drives on the reactor floor on four omni wheels at 45-degree corners. It carries a 4-DOF arm for camera inspection and thermocouple application.

The robot must straddle a 4.5" step between the inner and outer reactor floor while turning. Skid-steer was tested and rejected for this reason (`sim/isaac/skid_steer_design/ANALYSIS.md`).

## Team and timeline

- ECE 129 senior design at UCSC
- Target ship date: mid-June 2026
- Mechanical lead: Adrian (OnShape model owner)
- Software / sim / state machine: Nikhil
- Other team members exist; assign work by name when the user mentions them

## Hardware (current)

| Subsystem | Selection |
|---|---|
| Chassis | ~15" x 9" x 6" octagonal prism, 3" corner chamfers, ~40 lbs |
| Drive | X-drive (4 omni wheels at 45-deg corners) |
| Wheels | AndyMark 8" Dualie Plastic Omni (am-0463) |
| Drive motors | ODrive M8325s brushless x4 |
| Motor controllers | ODrive S1 x4, CAN bus |
| Arm | 4-DOF parallel-link, ~36" reach |
| Arm motors | NEMA 17 stepper x4 + Cricket Drive MK II 25:1 gearboxes |
| Arm drivers | TMC5160-TA x4 |
| Compute | Jetson Orin Nano Super |
| Cameras | Orbbec Gemini 2 (depth, on arm) + chassis RGB |
| Power | Tether-supplied (no onboard battery) |

The OnShape model and the BOM are authoritative for hardware. Numbers in `analysis/` or in old sim scripts may lag.

Older specs you may find in code or git history that are NO LONGER current:
- REV NEO 2.0 motors -> replaced by ODrive M8325s
- ESP32 firmware -> replaced by Jetson Orin Nano Super doing it all
- 20.4 kg / 15.354" x 9.353" x 7.1" -> replaced by ~40 lb / ~15" x 9" x 6"
- All-NEMA-17 arm with insufficient J2 torque -> reworked into the parallel-link 25:1 design

## Dev workflow

VSCode "Reopen in Container". Container is ROS 2 Humble desktop (see `docker/Dockerfile.dev`). Repo mounts at `/workspace`. ROS 2 is sourced automatically; `install/setup.bash` is sourced at shell start when it exists.

```bash
# Build the ROS 2 workspace
cd /workspace
colcon build --packages-select fortis_safety
source install/setup.bash

# Run the existing pieces
ros2 run fortis_safety mission_state_node
ros2 run fortis_safety event_console

# Tests
python3 -m pytest src/fortis_safety/test/ -v
```

Isaac Sim runs on the Windows host, not in the container.

## Repo layout

```
.devcontainer/      VSCode dev container config
docker/             Dockerfile.dev + docker-compose.yml
src/                ROS 2 colcon workspace
  fortis_safety/    Mission FSM + ROS node + REPL console (only package today)
control/            Pure-Python libraries (no ROS coupling)
  fortis_comms/     Motor abstractions, ODrive S1 wrapper, x-drive kinematics, EKF
sim/                Simulation
  isaac/            Isaac Sim 5.1 (host-only, not container)
analysis/           Drivetrain / arm analysis writeups (markdown)
legacy/             Reference code from earlier design rounds
```

## ROS 2 packages

Planned: `fortis_description`, `fortis_drive`, `fortis_arm`, `fortis_perception`, `fortis_localization`, `fortis_msgs`, `fortis_bringup`, `fortis_safety`.

Existing: `fortis_safety` only.

### Blocking chain

- `fortis_description` (URDF) is blocked on Adrian's OnShape export. Do NOT try to write the URDF from scratch.
- `fortis_drive` is blocked on `fortis_description` (needs joint names and frames).
- `fortis_arm` is blocked on `fortis_description`.
- `fortis_perception`, `fortis_localization`, `fortis_msgs`, `fortis_bringup` are not blocked on hardware but no work has started.

## Communication style

- No em dashes anywhere. Use periods, commas, parentheses, hyphens.
- No emojis in source, comments, or docs.
- No filler ("Great question", "Let me think about this", trailing summaries that restate the diff).
- No fake content or fabricated numbers. If you don't know, say so. If a memory says X exists, verify before recommending it.
- Direct, terse, professional. The user can read a diff and a commit log; don't recap them.
- Comments explain *why*, not *what*. One-paragraph why-comments beat line-by-line narration.

## What NOT to touch

- `sim/isaac/xdrive/assets/diiid_reactor.usd` - collision mesh applied manually in the Isaac Sim GUI. Regenerating the file destroys that.
- The OnShape model, the BOM, and other external artifacts. Out of repo.
- Anything under `sim/isaac/xdrive/deprecated/`. Reference only.
- Anything under `legacy/`. Reference only.

## Procurement constraints

The program forbids:
- Parts from Chinese vendors
- Marketplace sites (Amazon, eBay, AliExpress)

Stick to recognized industrial suppliers (McMaster-Carr, Misumi, AndyMark, ODrive, RockWest Composites, Lin Engineering, etc.). If the user asks for a part recommendation, propose only suppliers in this lane.

## Current priorities (as of 2026-05-01)

1. Sim 2: R0 port entry. Plan in `sim/isaac/xdrive/docs/R0_ENTRY_PLAN.md`. Not started.
2. Wait for OnShape URDF export to unblock `fortis_description` and downstream packages.
3. Iterate on `fortis_safety` if the FSM grows.

## Known issues that are NOT priorities right now

These are documented for context but do not need to be fixed unless asked:
- `control/fortis_comms/odrive_s1.py` has several real-bus / format bugs (see its README). They block real-hardware bring-up but not simulation or FSM work.
- `control/fortis_comms/ekf.py` uses an absolute config path that only resolves under the test fixture.
- Isaac Sim canonical scripts still carry the older 15.354" / 20.4 kg / NEMA-17 arm constants.

## How to verify changes don't break the build

```bash
cd /workspace
colcon build --packages-select fortis_safety
source install/setup.bash
python3 -m pytest src/fortis_safety/test/ -v
```

All 27 mission_state_machine tests should pass.
