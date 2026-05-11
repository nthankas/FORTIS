# FORTIS Cleanup Audit

Branch: `chore/cleanup-audit` (off `main` at `26c4fe2`). Generated 2026-05-11.

Read-only audit. No source files were deleted, moved, or refactored. The
only mechanical changes on this branch are the pre-commit auto-fixes
listed in **Section 9** below (separate commit).

---

## Recommendation buckets (top of file by design)

### SAFE NOW

Things that can be deleted, moved, or trivially fixed with no behavioural
risk. Ordered by LOC reduced / ROI.

| Item | LOC | Action | Rationale |
|---|---:|---|---|
| README drift across 5 files | n/a | rewrite | Multiple READMEs still describe `odrive_s1.py`, `motor_base.py`, `ekf.py`, and the `move_to_pose` action server as live in `fortis_comms` / `fortis_arm`. All four moved to `legacy/` weeks ago. See **Section 5**. |
| `src/fortis_comms/fortis_comms/cfgs/ekf_params.json` + `cfgs/__init__.py` | 7 | delete | Bundled config for `ekf.py`, which is in `legacy/`. `package_data` in `setup.py` ships it; no live consumer in `src/`. Confirmed by grep -- nothing under `src/` reads it. |
| `src/fortis_comms/setup.py` `package_data` for `cfgs/*.json` | 6 | delete block | Same reason as above; removing the JSON also lets this block go. |
| `legacy/coppelia_ik_reference/inverse_kinetmatic_solver.py` | 149 | archive to `docs/historical/` or delete | Imports `coppeliasim_zmqremoteapi_client`. No build, no consumers, no current IK pipeline reads it. Useful only as a "how Carlos did it in 2025" reference; could live in a git tag or `docs/historical/`. |
| `fortis_comms.xdrive_kinematics.xdrive_fk_solver` (function) | 4 | mark as test-only or delete | Only ever called from tests. The drive node uses IK exclusively. Leaving it is fine if planning to use it for odometry verification once `robot_localization` is wired in, but it is currently dead. |
| `fortis_comms.xdrive_kinematics.wheel_rot_to_lin_vel` (function) | 2 | delete | Only referenced from `test_imports.py` smoke import; not called anywhere in production or other tests. |

### SAFE AFTER UPSTREAM SWAP

Interim files that can go once their upstream replacement is wired in.
See **Section 2** for the integration sketches.

| Item | LOC | Replaces with | Blocker |
|---|---:|---|---|
| `legacy/deprecated_motor_stack/odrive_s1.py` | 90 | `odrive_ros2_control` (Factor-Robotics or odriverobotics/ros_odrive) | Need a working `ros2_control` hardware interface in `fortis_drive_hw` plus the per-axis ODrive node IDs configured in the ODrive web GUI. Already in `legacy/` -- final delete blocks on the replacement passing the same drive integration test. |
| `legacy/deprecated_motor_stack/motor_base.py` | 127 | `hardware_interface::SystemInterface` (ros2_control) | Same as above. The abstract `Motor` shape disappears once `ros2_control` owns the per-actuator interface. |
| `legacy/deprecated_ekf/ekf.py` | 122 | `robot_localization` `ekf_node` from cra-ros-pkg | Need wheel-odom publisher + IMU bridge + cuVSLAM pose. None exist yet. |
| `legacy/deprecated_arm_action/move_to_pose_action_server.py` | 141 | MoveIt 2 `MoveGroup` action via `move_action_capability` | Need URDF (blocks on Adrian + Carlos), then `fortis_moveit_config` from the MoveIt Setup Assistant. |
| `src/fortis_arm/fortis_arm/arm_controller_node.py` gripper stub | ~50 | direct gripper command via `JointTrajectoryController` (ros2_control) or `Trigger` -> Teensy serial | Need Teensy `SystemInterface` plus the protocol contract in `firmware/teensy/PROTOCOL.md` to land actuator semantics. State-gating logic stays even after the gripper is real. |
| `src/fortis_comms/fortis_comms/xdrive_kinematics.py` | 32 | `mecanum_drive_controller` from ros2_controllers (or custom mixin) | The X-drive H matrix is one ros2_control plugin away from being deletable. Same caveat as ODrive: needs the hardware interface first. The four IK constants (`WHEEL_RADIUS`, `LEN_X`, `LEN_Y`, `MAX_WHEEL_SPEED`) move into a controller YAML. |

### LEAVE ALONE

Load-bearing despite appearances.

| Item | Why |
|---|---|
| `src/fortis_safety/fortis_safety/mission_state_machine.py` | Pure-Python FSM. ROS-agnostic and well tested (20 tests). Survives every upstream swap because every other node gates off it. |
| `src/fortis_safety/fortis_safety/mission_state_node.py` | Thin ROS wrapper around the FSM. ~155 LOC of which most is QoS + callback factories. Replacing with `nav2_behavior_tree` is a future option, not a cleanup task. |
| `src/fortis_safety/fortis_safety/event_console.py` | Bring-up REPL. Not load-bearing for production, but actively useful and cheap to keep. |
| `src/fortis_drive/fortis_drive/drive_node.py` | The gate is the contract; the IK call is the implementation. Even after `mecanum_drive_controller` lands, *something* has to subscribe to `/cmd_vel`, check mission state, and re-emit. This node is that something. |
| `src/fortis_arm/fortis_arm/arm_controller_node.py` (gate + state cache) | Same reasoning as drive_node. The gate around MoveGroup goal-callbacks survives the MoveIt 2 swap. |
| `legacy/README.md` + the four `deprecated_*/` files | The deprecation pattern (keep the original next to a clear note) is more useful than burying it in git history. Delete only after the upstream replacements have been verified in CI. |
| `analysis/*.md` | Design rationale for irreversible decisions (skid-steer rejection, X-drive choice, orbit geometry). Useful for the team, useful for the GA review. Keep. |
| `firmware/teensy/PROTOCOL.md`, `HANDOFF.md` | Cesar's contract. Touched only when the protocol bumps. Out of scope. |

### Questions for human review

1. `fortis_comms` currently ships only `xdrive_kinematics.py`. Should the package be renamed (e.g. `fortis_kinematics`) to reflect what it actually is, or kept as a stable interface that will grow back when the upstream swap demands a custom mixin? See README at `src/fortis_comms/README.md:1`.
2. `xdrive_kinematics.LEN_X = 4.405 in` and `LEN_Y = 6.462 in` do not match the locked chassis skeleton (13.082 x 8.54 in). Flagged in the comms README itself (`src/fortis_comms/README.md:51-53`). Fixing this changes wheel-velocity output, which is a calibration decision, not a cleanup decision. Confirm direction before fixing.
3. `fortis_msgs/action/MoveToPose.action` has no consumer (the action server moved to `legacy/`). Keep it for the eventual MoveIt 2 wrapper, or delete and re-add when needed?
4. The "Footgun: `CONTEXT_FIELDS` is duplicated" note in `fortis_safety/README.md:31-38` is a real duplication between `mission_state_node.py:38` and `event_console.py:26`. Should this be deduplicated by exporting `CONTEXT_FIELDS` from `mission_state_machine.py` and importing it in both? Trivial, blast radius zero, but pulled out of the audit because it isn't strictly "cleanup" -- it's a small refactor.
5. `src/fortis_bringup/launch/sim.launch.py` and `teleop.launch.py` are stubs that log `TODO: not implemented` and exit. They are referenced by the bringup README as "planned". Are they speculative scaffolding (delete until needed) or genuinely planned for the June 5 demo (leave as TODO placeholders)?

---

## 1. Per-package inventory

LOC counts are non-blank, non-comment Python lines unless noted. Test
LOC reported separately. Counts are wall-clock as of `26c4fe2`.

### fortis_msgs

- **Type:** `ament_cmake`. Pure IDL package.
- **Python LOC:** 0 (no Python source; only `.msg` / `.action` files).
- **Public artifacts:** 4 messages (`ChassisCamClick`, `GraspCandidate`, `MissionState`, `WheelVelocities`), 1 action (`MoveToPose`).
- **Consumers in `src/`:** `WheelVelocities` used by `fortis_drive/drive_node.py:61` and the drive integration test. The other three messages and the action have **no Python or C++ consumer** anywhere in `src/`. They exist for the not-yet-built perception / planner / arm-MoveIt seam.
- **Status:** production. Generation working; consumers will grow as upstream swap proceeds.

### fortis_safety

- **Python LOC (production):** 702 across 5 files (the 5th is `setup.py`).
- **Python LOC (tests):** 245 (`test_mission_state_machine.py`, `test_copyright.py`).
- **Production modules:**
  - `mission_state_machine.py` (337 LOC) -- pure-Python FSM, State / Event enums, `Transition` dataclass, `MissionStateMachine` class, `to_mermaid()` diagram export. Imported by `mission_state_node.py`, `event_console.py`, the integration test, and the bringup launch test.
  - `mission_state_node.py` (155 LOC) -- ROS wrapper. Subscribes per-event + per-context topic; publishes latched `/fortis/mission_state`.
  - `event_console.py` (179 LOC) -- bring-up REPL. Not imported by anything (entry point only).
- **Tests:** `test_mission_state_machine.py` covers 20 cases. No tests on the ROS wrapper directly (covered indirectly by `fortis_integration_tests`).
- **Status:** production. Load-bearing. Touched only when the FSM contract changes.

### fortis_drive

- **Python LOC (production):** 320 across 3 files (drive_node + setup.py + __init__.py).
- **Python LOC (tests):** 427 (`test_drive_node.py`, `conftest.py`, `test_copyright.py`).
- **Production modules:**
  - `drive_node.py` (290 LOC) -- ROS node, X-drive IK + state gating + per-state throttled rejection logging. Imports `fortis_comms.xdrive_kinematics` + `fortis_msgs.WheelVelocities`.
- **Tests:** 5 functional tests. All pass through real DDS round-trips.
- **Status:** production for the gating + topic contract. IK + ODrive driver portions are interim (see Section 2).

### fortis_arm

- **Python LOC (production):** 256 across 3 files.
- **Python LOC (tests):** 373 (`test_state_gating.py`, `test_bringup.py`, `conftest.py`).
- **Production modules:**
  - `arm_controller_node.py` (223 LOC) -- ROS node, state-gate cache + gripper services (`open_gripper`, `close_gripper`, both `std_srvs/Trigger`, both rejected outside `ALLOWED_ARM_STATES` and a stub message otherwise). The `move_to_pose` action server documented in the package README is **NOT** here; it was retired to `legacy/deprecated_arm_action/`.
- **Tests:** 6 functional tests (parametrised over every state in the FSM + the no-state-yet race).
- **Status:** scaffold. Gating + topic contract solid; actuation deferred to the upstream-swap pass.

### fortis_comms

- **Python LOC (production):** 74 across 4 files (`xdrive_kinematics.py` + `setup.py` + 2 empty `__init__.py`).
- **Python LOC (tests):** 85 (`test_xdrive_kinematics.py`, `test_imports.py`, `test_ekf.py` -- the last is an empty no-op module).
- **Production modules:**
  - `xdrive_kinematics.py` (32 LOC) -- X-drive IK / FK, H matrix, `WHEEL_RADIUS` + chassis dimension constants, `MAX_WHEEL_SPEED` saturator.
  - `cfgs/ekf_params.json` -- orphan. Loaded by `ekf.py` which is in `legacy/`. Still shipped via `setup.py` `package_data`.
- **Tests:** 4 functional tests on IK/FK round-trip; `test_imports.py` is a smoke-import.
- **Status:** kinematics in production; **the README describes 4 modules that no longer live here** (`motor_base`, `odrive_s1`, `ekf`, plus `cfgs/ekf_params.json`'s consumer). All moved to `legacy/`. See Section 5.

### fortis_integration_tests

- **Python LOC (production):** 32 (`__init__.py` + setup boilerplate).
- **Python LOC (tests):** 783 across `test_safety_drive_integration.py` (480), `test_bringup_launch.py` (203), `test_safety_arm_integration.py` (72 scaffold-only, all skipped), and `conftest.py`.
- **Status:** test-only package. `test_safety_drive_integration.py` is the production cross-package contract test; `test_bringup_launch.py` exercises the bringup launch file; `test_safety_arm_integration.py` is scaffold-only (4 skipped tests with descriptive bodies, no assertions).

### fortis_bringup

- **Python LOC (production):** 99 across 5 files (3 launch files + `__init__.py` + `setup.py`).
- **Python LOC (tests):** 0 (functional bringup tested from `fortis_integration_tests`).
- **Production artifacts:**
  - `launch/bringup.launch.py` -- composes `mission_state_node` + `drive_node`. **Not a stub** despite the README.
  - `launch/sim.launch.py` -- stub. `LogInfo("TODO: not implemented")`.
  - `launch/teleop.launch.py` -- stub. `LogInfo("TODO: not implemented")`.
  - `config/bringup_params.yaml` -- placeholder, comment-only.
- **Status:** real bringup is live; sim + teleop are stubs.

### fortis_description

- **Python LOC (production):** 0.
- **Type:** `ament_cmake`. Installs empty asset directories.
- **Status:** scaffold. URDF + meshes blocked on Adrian + Carlos OnShape cleanup (95 links / 94 joints).

---

## 2. Interim modules and their upstream replacements

The team is targeting these specific upstream packages, per the design
context: **ODrive S1 ROS 2 driver**, **robot_localization**, **MoveIt 2**,
and the **DepthAI ROS driver** (`depthai_ros_driver`). All four are
Humble-compatible and present in the FORTIS dev image.

### 2.1 `legacy/deprecated_motor_stack/odrive_s1.py` + `motor_base.py` -> `odrive_ros2_control`

- **Current purpose:** hand-rolled CAN wrapper around ODrive S1 0.5.x using `python-can`, plus an abstract `Motor` base class for connect/disconnect/command_velocity. Both moved to `legacy/` already.
- **Upstream:**
  - Official: <https://github.com/odriverobotics/ros_odrive> (apt: `ros-humble-odrive-can`; the `odrive_ros2_control` sub-package is the ros2_control plugin).
  - Community: <https://github.com/Factor-Robotics/odrive_ros2_control> (better-documented for position/velocity/torque + feedforward; the Foxy-to-Humble migration thread is issue #20 on that repo).
- **What integration looks like:**
  1. Add `fortis_drive_hw` package with one `hardware_interface::SystemInterface` per ODrive axis (4 axes daisy-chained on CAN, IDs 1-4).
  2. Controller YAML under `fortis_drive_hw/config/controllers.yaml` declaring `velocity_controllers/JointGroupVelocityController` (or `mecanum_drive_controller` if we want it to also own the IK).
  3. URDF `<ros2_control>` block referencing the four wheel joints from `fortis_description`. **Blocks on URDF**.
  4. `fortis_drive/drive_node.py` becomes a thin `/cmd_vel` -> per-joint-velocity bridge that publishes to the controller's command topic instead of `fortis_msgs/WheelVelocities`. The state-gate stays.
- **Effort:** medium. Most of the lift is one-time `SystemInterface` plumbing.
- **Blocking:** URDF (Adrian + Carlos), then CAN-bus bring-up on real hardware.

### 2.2 `legacy/deprecated_ekf/ekf.py` -> `robot_localization` `ekf_node`

- **Current purpose:** six-state EKF (x, y, theta, vx, vy, omega) with optical-flow and IMU updates. Moved to `legacy/`. Bundled defaults still under `fortis_comms/cfgs/ekf_params.json` (orphaned).
- **Upstream:** <https://github.com/cra-ros-pkg/robot_localization> (`ros-humble-robot-localization`). Reference config: `ros2` branch `params/ekf.yaml`.
- **What integration looks like:**
  1. New package `fortis_localization` with `config/ekf.yaml` declaring odom + IMU + cuVSLAM pose inputs.
  2. `bringup.launch.py` adds `IncludeLaunchDescription` for the ekf_node, plus the wheel-odom publisher (likely emitted by the ros2_control diff-drive / mecanum controller) and the OAK-D-Lite IMU bridge.
  3. Delete the orphan `ekf_params.json` once the new config is canonical.
- **Effort:** small to medium. ekf_node is a black box; lift is in producing the inputs.
- **Blocking:** wheel-odom publisher (needs 2.1 done), cuVSLAM (`isaac_ros_visual_slam`) staged in fortis-dev-gpu but not yet bridged.

### 2.3 `legacy/deprecated_arm_action/move_to_pose_action_server.py` -> MoveIt 2 `MoveGroup`

- **Current purpose:** scaffold action server that accepted `MoveToPose` goals when in `ALLOWED_ARM_STATES`, always returned `succeeded=false`. Moved to `legacy/`.
- **Upstream:** MoveIt 2 (`ros-humble-moveit`). The `MoveGroup` action server publishes the `/move_action` action of type `moveit_msgs/action/MoveGroup`. `pymoveit2` (`ros-humble-pymoveit2`) provides a Python client.
- **What integration looks like:**
  1. `fortis_moveit_config` generated via MoveIt Setup Assistant against the final URDF.
  2. `fortis_arm/arm_controller_node.py` keeps its state-gate; goals are forwarded to `/move_action` via the `moveit_commander` Python client (or directly via the action client). The gate stays; the action target shifts from the scaffold to MoveIt 2.
- **Effort:** large (URDF + SRDF + planner config), but the cleanup half is small (delete the legacy file, adjust one import inside `arm_controller_node`).
- **Blocking:** URDF (Adrian + Carlos) + SRDF authoring + MoveIt Setup Assistant run.

### 2.4 OAK-D cameras (5x) -> `depthai_ros_driver`

- **Current purpose:** none in `src/`. Cameras exist in BOM (4x OAK-D Lite A00483 + 1x OAK-D Pro A00546) but no ROS node consumes them yet.
- **Upstream:** <https://github.com/luxonis/depthai-ros> (`ros-humble-depthai-ros`). Launch entry: `ros2 launch depthai_ros_driver driver.launch.py`. Targeted at OAK-D-lineup cameras out of the box.
- **What integration looks like:**
  1. New package `fortis_perception` with `launch/cameras.launch.py` that includes `depthai_ros_driver` per-device with role-tagged namespaces (`/fortis/camera/chassis_front`, `/fortis/camera/chassis_rear`, `/fortis/camera/chassis_left`, `/fortis/camera/chassis_right`, `/fortis/camera/arm`).
  2. Per the BOM: chassis cameras attach via Coolgear hub on USB Port 3; OAK-D Pro on USB Port 1 with Y-adapter for 5V from the arm rail.
  3. Front/rear OAK-D Lites feed `isaac_ros_visual_slam` stereo input (per CLAUDE.md).
- **Effort:** small (apt install + launch glue). The whole `fortis_perception` package is greenfield.
- **Blocking:** physical OAK-D Pro Y-adapter (per CLAUDE.md, partial); software-only path is unblocked.

### 2.5 `fortis_comms/xdrive_kinematics.py` -> `mecanum_drive_controller` (ros2_controllers) or kept

- **Current purpose:** X-drive IK + FK + saturation. 32 LOC, the math half of the drive bring-up.
- **Upstream candidate:** `mecanum_drive_controller` ships with `ros2_controllers` (`ros-humble-ros2-controllers`). It speaks `/cmd_vel` -> per-wheel-velocity. If the IK math survives wheel-radius and base-frame parameter override, the custom file goes away.
- **What integration looks like:**
  1. Controller YAML declares `mecanum_drive_controller/MecanumDriveController` with `wheel_radius`, `wheel_base`, `track_width` from the chassis dimensions.
  2. `drive_node.py` no longer calls `xdrive_ik_solver`; the controller does it.
- **Effort:** small. The risk is verifying that `mecanum_drive_controller` accepts the X-drive geometry (45-degree corners). If it does not, keep the custom file.
- **Blocking:** 2.1 (need ros2_control hardware interface first).

---

## 3. Dead code findings

### Files not imported anywhere in `src/` (excluding tests)

- `src/fortis_comms/fortis_comms/cfgs/ekf_params.json` -- shipped, not consumed in `src/`.
- `src/fortis_comms/fortis_comms/cfgs/__init__.py` -- empty marker for the orphaned config.

No other production `.py` file under `src/` is fully unreferenced. Every node module is either an entry-point (`mission_state_node`, `drive_node`, `arm_controller_node`, `event_console`) or imported by another production module.

### Functions / classes with no callers (production scope; tests do call them)

- `fortis_comms.xdrive_kinematics.xdrive_fk_solver` -- callers: `test_xdrive_kinematics.py`, `test_imports.py`. No production caller. Keeping it makes sense if odometry verification is planned post-`robot_localization`-swap.
- `fortis_comms.xdrive_kinematics.wheel_rot_to_lin_vel` -- callers: `test_imports.py` (smoke-imported only, not called). Pure dead code.

### Commented-out code blocks > 10 lines

None found. The codebase is clean in that respect.

### Unused imports

Not auto-run (ruff is not in the pre-commit config; per the brief, do
not install or configure it in this pass). Manual scan of the largest
modules (`mission_state_machine.py`, `drive_node.py`, `arm_controller_node.py`,
`mission_state_node.py`, `event_console.py`) finds no obviously unused
imports. The `from __future__ import annotations` lines are deliberate
(PEP 563 forward-ref hygiene).

---

## 4. Legacy + analysis directory assessment

### `legacy/`

| File | Lines | Original purpose | Current relevance | Recommendation |
|---|---:|---|---|---|
| `legacy/README.md` | 17 | Inventory of moved files + the "deprecation pattern" note | Active reference; teammates reading the comms / arm code follow this back here. Aligned with project policy (no AI attribution, "replaced by ros2_control + standard ROS 2 packages" framing). | **Keep.** |
| `legacy/deprecated_motor_stack/motor_base.py` | 127 | Abstract `Motor` base class | Reference for what the contract used to be; useful when wiring up the ros2_control SystemInterface. | **Keep until 2.1 lands**, then delete. |
| `legacy/deprecated_motor_stack/odrive_s1.py` | 90 | `ODrive_S1(Motor)` CAN wrapper | Same as above; also documents the three known bugs called out in the comms README (CAN bus contradiction, struct.pack misuse, blocking recv). | **Keep until 2.1 lands**, then delete. |
| `legacy/deprecated_ekf/ekf.py` | 122 | 6-state EKF | Reference for which states + which sensors the team had in mind. Useful when authoring the `robot_localization` `ekf.yaml`. | **Keep until 2.2 lands**, then delete. |
| `legacy/deprecated_arm_action/move_to_pose_action_server.py` | 141 | `move_to_pose` action server scaffold | Reference for the gating + reply shape; the MoveIt 2 wrapper will want to preserve the same gate semantics. | **Keep until 2.3 lands**, then delete. |
| `legacy/coppelia_ik_reference/inverse_kinetmatic_solver.py` | 149 | Carlos Vazquez's CoppeliaSim IK script for the older 4-DOF arm design | Targets the older arm concept (pre parallel-link rework), different sim, different DH params. No tool reads it. The current arm IK work happens in `sim/isaac/xdrive/`. | **Candidate for deletion** (or archive to `docs/historical/`). Lowest-value entry in `legacy/`. |

### `analysis/`

| File | Lines | Purpose | Recommendation |
|---|---:|---|---|
| `analysis/orbit_analysis.md` | 119 | Tangential-drive geometry on the reactor step. Foundational for the orbit phase. | **Keep.** Design rationale for the locked drive plan. |
| `analysis/pivot_analysis.md` | 102 | Pivot maneuver feasibility on the step. | **Keep.** |
| `analysis/skid_steer_rejection.md` | 125 | Documents the 5%-pass Monte Carlo on skid-steer point turns. Justifies the X-drive choice in the senior-design review. | **Keep.** Load-bearing for the design narrative. |
| `analysis/torque_analysis.md` | 84 | Motor torque budget for the X-drive. | **Keep.** Cross-references the Power Budget v1.0. |
| `analysis/xdrive_analysis.md` | 163 | Full X-drive vs alternatives writeup. | **Keep.** |

None of `analysis/*.md` are stale -- they document irreversible design
decisions and are the reference set for the GA review.

---

## 5. Documentation drift

README claims that no longer match the code.

### Root README (`README.md`)

| Line | README says | Code says | Severity |
|---|---|---|---|
| 147 | "`fortis_comms` | X-drive kinematics in production; `odrive_s1.py` / `motor_base.py` / `ekf.py` are interim helpers slated for replacement by upstream packages" | `odrive_s1.py`, `motor_base.py`, and `ekf.py` are no longer in `fortis_comms`. They moved to `legacy/deprecated_*` weeks ago. The only file in `fortis_comms/fortis_comms/` is `xdrive_kinematics.py`. | High. Top-of-repo claim, factually false. |
| 149 | "`fortis_arm` | scaffold; action + gripper services gated by mission state" | Action server was retired (now in `legacy/deprecated_arm_action/`). Only the gripper services + state cache remain. | Medium. |
| 150 | "`fortis_bringup` | scaffold; stub launch files for `bringup` / `sim` / `teleop`, no node includes yet" | `bringup.launch.py` composes `mission_state_node` + `drive_node`. Only `sim.launch.py` and `teleop.launch.py` are stubs. | Medium. |

### `src/fortis_comms/README.md`

- Lines 9-13: the contents table lists `motor_base`, `odrive_s1`, `xdrive_kinematics`, `ekf`, and `cfgs/ekf_params.json`. Three of those modules are in `legacy/`. **High severity.**
- Lines 19, 21: test table lists `test_imports.py` ("...IK/FK on zero input return zero, `EKF()` constructs from the bundled default config") and `test_ekf.py` ("Init, predict, optical-flow update, IMU update, and full-loop stability"). `EKF()` doesn't exist in the package any more; `test_ekf.py` is an empty no-op module per its own header comment.
- Lines 37-49 (entire "Migration from `control/fortis_comms`" section) and lines 43-50 (entire "Known issues deferred" section) all reference `ekf.py` and `odrive_s1.py` as live, with bug repro line numbers (`odrive_s1.py:17`, `:37`). All pointers stale.

### `src/fortis_arm/README.md`

- Line 9: "`move_to_pose` | action server, `fortis_msgs/action/MoveToPose` | Move the end-effector to a `geometry_msgs/PoseStamped`. Goal callback rejects when current mission state is not in the allowed set. Accepted goals always return `succeeded=False, message='kinematics not implemented'`." -- action server is not in this package any more. **High severity.**
- Lines 70-71: test table describes `test_state_gating.py` as "Action goals are accepted (and return the stub result) in `ALLOWED_ARM_STATES` and rejected at the gate elsewhere; gripper services follow the same gating." -- the action-goal half is no longer tested (and not in the file). Only gripper-service gating remains. The test file's own docstring (`test_state_gating.py:9-11`) correctly notes the action-server retirement; only the README is stale.

### `src/fortis_drive/README.md`

- Lines 27-28: "the actual CAN bring-up lives in `fortis_comms/odrive_s1.py` (interim helper, see that package's README)". `odrive_s1.py` is in `legacy/deprecated_motor_stack/`, not in `fortis_comms`. **Medium severity** (pointer is wrong but the architectural intent is clear).

### `src/fortis_bringup/README.md`

- Line 3: "Currently scaffolding only -- every launch file is a stub that returns an empty `LaunchDescription` and logs a TODO. Real composition is added per package as each one comes online." -- `bringup.launch.py` is no longer a stub. **Medium severity.**
- Line 9 (bringup row), line 26 (Running section header "stubs only today"), line 34 ("Each currently logs `TODO: not implemented` at INFO level") all assert the bringup file is empty. It is not.
- Line 38: "Wire `bringup.launch.py` to the real `mission_state_node`, `drive_node`, and (when present) arm controller." Partially done; only arm-controller wiring remains. Could be restated as "Add the arm controller include."

### `src/fortis_msgs/README.md`

- Line 33: "Used by | `fortis_arm/arm_controller` (`move_to_pose` action server)". No live action server uses this action. The action definition itself is preserved; the consumer note is stale. **Low severity** (the action is correctly described, only the consumer is gone).

### `src/fortis_integration_tests/README.md`

- Lines 15-17: test table lists only `test_safety_drive_integration.py`. Misses `test_bringup_launch.py` (real, 2 tests, both pass) and `test_safety_arm_integration.py` (scaffold-only, 4 skipped). **Low severity** (additional tests aren't mis-described, they just aren't mentioned).

### `src/fortis_safety/README.md`

No drift detected. Topic, QoS, and footgun note all match the code.

### `src/fortis_description/README.md`

No drift -- the package is genuinely empty and the README says so.

### `legacy/README.md`

No drift. Accurately describes the four deprecated files and the
deprecation pattern.

---

## 6. Size + complexity hotspots

### Top 10 largest Python files

| Lines | File |
|---:|---|
| 480 | `src/fortis_integration_tests/test/test_safety_drive_integration.py` |
| 374 | `src/fortis_drive/test/test_drive_node.py` |
| 337 | `src/fortis_safety/fortis_safety/mission_state_machine.py` |
| 290 | `src/fortis_drive/fortis_drive/drive_node.py` |
| 242 | `src/fortis_arm/test/test_state_gating.py` |
| 223 | `src/fortis_arm/fortis_arm/arm_controller_node.py` |
| 220 | `src/fortis_safety/test/test_mission_state_machine.py` |
| 203 | `src/fortis_integration_tests/test/test_bringup_launch.py` |
| 179 | `src/fortis_safety/fortis_safety/event_console.py` |
| 155 | `src/fortis_safety/fortis_safety/mission_state_node.py` |

No production file exceeds 500 LOC. The 480-LOC integration test is
worth a look in the next test-organisation pass -- it covers four
distinct scenarios (a/b/c/d as described in the package README) and
could split along that axis without disturbing fixtures. Not urgent.

`test_drive_node.py` at 374 LOC is dense but coherent (one harness +
five scenarios). Splitting would not improve it.

### Top 10 largest Markdown files

| Lines | File |
|---:|---|
| 403 | `firmware/teensy/PROTOCOL.md` |
| 230 | `CLAUDE.md` (private; gitignored) |
| 220 | `sim/isaac/xdrive/deprecated/results/arm_continuous_sweep_v3/v2/sweep_report_v2.md` |
| 213 | `sim/isaac/deprecated/skid_steer_design/ANALYSIS.md` |
| 206 | `sim/isaac/xdrive/deprecated/results/arm_continuous_sweep_v1/sweep_report.md` |
| 175 | `firmware/teensy/HANDOFF.md` |
| 163 | `analysis/xdrive_analysis.md` |
| 163 | `README.md` |
| 125 | `analysis/skid_steer_rejection.md` |
| 121 | `sim/isaac/xdrive/docs/arm_spec.md` |

The three large `sim/.../deprecated/.../sweep_report*.md` files are sim
artifacts intentionally under the sim package's deprecated-results
discipline (per `sim/README.md`). Out of scope for this audit.

### Cyclomatic complexity

`radon` is not installed and not in the pre-commit config; no automatic
metric available. Manual review finds the largest control-flow surface
in `mission_state_machine.py` (`step` walks the table top-to-bottom with
guard composition; well factored). No `if` ladder deeper than three.

---

## 7. TODO / FIXME / XXX / HACK aggregation

No `FIXME`, `XXX`, or `HACK` markers anywhere under `src/`. All TODOs are
deliberate placeholders in scaffold packages. Listed with package
grouping for ticket conversion.

### fortis_bringup

| File | Line | Text | Suggested ticket |
|---|---:|---|---|
| `src/fortis_bringup/launch/sim.launch.py` | 4, 14 | "TODO: not implemented. Will eventually launch a simulator-backed configuration of the FORTIS stack..." | Decide whether sim.launch.py is in scope for June 5; if not, delete until needed. |
| `src/fortis_bringup/launch/teleop.launch.py` | 4, 14 | "TODO: not implemented. Will eventually launch operator-station nodes..." | Same decision as sim. Foxglove bridge is the most likely first content. |
| `src/fortis_bringup/config/bringup_params.yaml` | 7 | "TODO: populate" | Populate once per-node params are declared (depends on robot_localization config and ros2_control controllers landing). |
| `src/fortis_bringup/README.md` | 9, 10, 11, 17, 36 | TODOs that mirror the README "Planned" tables | Update README to reflect that `bringup.launch.py` is no longer a stub (see Section 5). |

### fortis_description

| File | Line | Text | Suggested ticket |
|---|---:|---|---|
| `src/fortis_description/README.md` | 11, 12, 13, 14, 15, 27 | TODOs marking which subdirectories are still placeholders | Blocked on Adrian + Carlos OnShape cleanup. |
| `src/fortis_description/meshes/PLACEHOLDER.md` | 19 | TODO header | Same. |
| `src/fortis_description/urdf/TODO.md` | 21 | TODO header | Same. |

No TODOs in any production `.py` file under `src/`.

---

## 8. Concrete cleanup recommendations

See **Recommendation buckets** at the top of this document.

---

## 9. Mechanical change log (this branch)

The two mechanical changes the brief authorised were (a) run pre-commit
auto-fixes and (b) optionally remove unused imports via ruff. The brief
specifies skipping ruff because it is not in the pre-commit config; that
decision stands -- this branch does **not** install or configure ruff.

### 9.1 Pre-commit auto-fix pass

`pre-commit run --all-files` (pre-commit 4.6.0) on the tree at audit
start:

```
trim trailing whitespace.................................................Passed
fix end of files.........................................................Passed
check for merge conflicts................................................Passed
check yaml...............................................................Passed
flake8...................................................................Passed
pydocstyle...............................................................Passed
```

All six hooks pass on the current tree with no modifications. **No
auto-fix commit was needed**; the prior `chore: accept pre-commit
auto-fixes on pre-existing files` (`86ffc7d`) on `main` already covered
the only files the hygiene hooks wanted to rewrite.

### 9.2 `colcon test` re-verification

`colcon test` runs from inside `fortis-dev`; cannot be invoked on the
Windows host. Verification deferred to the CI smoke run on this branch
(see "Verification" at the bottom).

---

## Verification

- Local: `pre-commit run --all-files` is the only step runnable from the
  Windows host. Hard failures (if any) listed under Section 9.
- CI: pushing this branch (or opening a PR) will trigger
  `.github/workflows/ci.yml`. The `colcon` job is the authoritative
  signal that nothing structural broke from the auto-fix pass. The brief
  says do not push, so verification is left to the human reviewer to
  trigger when they pick this up.

---

## Anti-recommendations (things explicitly NOT to do tonight)

- Do not wire `robot_localization`, `odrive_ros2_control`,
  `depthai_ros_driver`, or MoveIt 2 yet. Each requires the URDF + the
  hardware interface plumbing this audit doesn't have authority to
  build. See **Section 2** for the integration sketches.
- Do not delete the `legacy/deprecated_*` files. They are still
  referenced as the "before" half of the upstream-swap migration story.
  Delete after the swap is verified, not before.
- Do not deduplicate `CONTEXT_FIELDS` between `mission_state_node.py`
  and `event_console.py` yet. It is a real refactor, not a cleanup, and
  the team made the duplication intentional. See **Section 8 question 4**.
- Do not rename `fortis_comms` even though it now only ships
  `xdrive_kinematics`. See **Section 8 question 1**.
