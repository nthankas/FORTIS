# fortis_safety

Mission-level state machine for FORTIS, plus a REPL-style operator
console for driving it manually during bring-up, plus the drive-health
monitor that latches motor faults into the FSM. The state machine is
the single source of truth for what the rest of the stack is allowed
to do at any moment. `fortis_drive` and `fortis_arm` both subscribe to
its output topic and gate motion off of it.

The decision logic is a pure-Python table-driven FSM with no ROS or
hardware dependencies, exercised by unit tests in isolation. A thin
ROS 2 node (`mission_state_node`) wraps the FSM and bridges it to
topics. An interactive REPL (`event_console`) lets you drive the FSM
by hand from a terminal; this is the only way to advance state until
the planner and UI land.

## What ships here

| Entry point | Purpose |
|---|---|
| `mission_state_node` | The production node. Owns the FSM, subscribes to `/fortis/events/<name>` (one topic per `Event` enum value) and `/fortis/context/<field>` (one topic per guard field), publishes the current state on `/fortis/mission_state` (latched, `TRANSIENT_LOCAL` + `RELIABLE`, depth 1). |
| `odrive_health_monitor_node` | The drive-fault bridge. Subscribes to `/fortis/drive/odrive_health` (`fortis_msgs/OdriveHealth`, a snapshot of all four ODrive S1 axes), publishes latched `/fortis/context/drive_healthy` (True iff a fresh snapshot has arrived AND all four axes report `active_errors == 0`), and emits `/fortis/events/fault` on the True→False edge — which drives the FSM into `FAULT` via the wildcard transition. Startup value is False (fail safe) and does NOT emit a fault (no True→False edge has occurred). |
| `event_console` | Bring-up tool only. A REPL that takes commands like `event start_orbit`, `set target_pose_valid true`, `state` and turns them into the right ROS publishes against the same topics `mission_state_node` subscribes to. Not a runtime component, not launched in production. |
| `mission_state_machine.py` | Pure-Python FSM (no ROS). The State / Event enums and the `TRANSITIONS` table. Imported by both nodes. |

## States

Nine states, defined as `State` in `fortis_safety/mission_state_machine.py`.
The "Drive allowed" and "Arm allowed" columns show the gate
`fortis_drive` and `fortis_arm` apply against `/fortis/mission_state`.

| State | Meaning | Drive allowed | Arm allowed |
|---|---|---|---|
| `IDLE` | Powered on, no mission active. | no | no |
| `ORBIT` | Driving the reactor floor, scanning for targets. | yes | no |
| `TARGETING` | A target was clicked; solving IK for an approach pose. | no | no |
| `ARM_AT_VIEW` | Arm parked at the view pose; operator picks observe vs grasp. | no | yes |
| `INSPECT` | Camera-only inspection (no contact). | no | yes |
| `PICK` | Closing on a grasp target. | no | yes |
| `HOLDING` | Object grasped, awaiting operator decision. | no | yes |
| `RETURN_HOME` | Driving back to home pose; arm stowing. | yes | yes |
| `FAULT` | Fault latched. Only `RESET` (with operator ack) can leave. | no | no |

Stow / deploy is intentionally not modeled here. Stow is a pose flag
the arm controller tracks; the mission can be in any state with the
arm stowed or deployed. Forcing stow into the FSM would create a
combinatorial state explosion.

## Events

Fifteen events, defined as `Event`:

`START_ORBIT`, `CHASSIS_CAM_CLICK`, `STOP`, `ARM_AT_VIEW_POSE`,
`IK_FAILED`, `CANCEL`, `SELECT_OBSERVE`, `SELECT_PICK`, `GRASP_SUCCESS`,
`GRASP_FAIL`, `RELEASE`, `DONE`, `HOME_REACHED`, `FAULT`, `RESET`.

The full event-to-transition mapping is the `TRANSITIONS` list near
the top of `fortis_safety/mission_state_machine.py`. Each `Transition`
row carries `(from_state, event, to_state, guard)`. `from_state = None`
is a wildcard meaning "from any state".

## Decision rules

1. **First-match-wins ordering.** `step()` walks `TRANSITIONS`
   top-to-bottom and takes the first row whose `event` matches, whose
   `from_state` matches (or is `None`), and whose `guard(ctx)` returns
   `True`. When two rows share the same `(from_state, event)` pair,
   the more specific guard must come first. The
   `(PICK, CANCEL)` pair uses this: `pick_pre_contact` first (back
   out to `ARM_AT_VIEW`) and the unguarded fallback second (drop to
   `FAULT` once the gripper has touched the target).
2. **Wildcard `FAULT`.** Any state can transition to `FAULT` on the
   `FAULT` event with no guard. The safety layer uses this to latch
   a fault from anywhere — `odrive_health_monitor_node` emits exactly
   this event when the drive goes unhealthy.
3. **`RESET` requires operator acknowledgement.** Leaving `FAULT` back
   to `IDLE` only happens on `RESET` when `ctx["operator_ack"] == True`.
   The operator publishes that ack via the `operator_ack` context
   topic before sending `RESET`. Recovery is never automatic — a
   drive that comes back healthy does NOT clear the fault.

Guards inspect a runtime `ctx` dict the caller passes to `step()`. The
FSM does not own that dict; the ROS node populates it from
`/fortis/context/<field>` Bool topics. Known fields: `target_pose_valid`,
`ik_ok`, `grasp_candidate_ok`, `gripper_closed`, `gripper_open`,
`arm_at_home`, `chassis_at_home`, `pick_in_contact`, `operator_ack`,
plus `drive_healthy` (published by `odrive_health_monitor_node`, not
by the console).

## Topics

| Topic | Type | Direction | QoS | Notes |
|---|---|---|---|---|
| `/fortis/mission_state` | `std_msgs/String` | published (mission_state_node) | TRANSIENT_LOCAL + RELIABLE, depth 1 | latched current state name; late subscribers receive the latest value on connect |
| `/fortis/events/<event>` | `std_msgs/Empty` | subscribed (mission_state_node) | default | one topic per `Event` enum value, lowercase name (e.g. `/fortis/events/start_orbit`) |
| `/fortis/context/<field>` | `std_msgs/Bool` | subscribed (mission_state_node) | default | one topic per known context field (see list above) |
| `/fortis/drive/odrive_health` | `fortis_msgs/OdriveHealth` | subscribed (health monitor) | default | four-axis S1 snapshot from the upstream bridge; topic overridable via the `health_topic` param |
| `/fortis/context/drive_healthy` | `std_msgs/Bool` | published (health monitor) | TRANSIENT_LOCAL + RELIABLE, depth 1 | latched aggregate: fresh snapshot AND all axes error-free |
| `/fortis/events/fault` | `std_msgs/Empty` | published (health monitor) | default (RELIABLE + VOLATILE) | edge trigger on healthy True→False. Deliberately NOT latched: replaying an old fault to a late subscriber would be worse than the (reliable-delivery) risk of missing one. |

`fortis_arm` and `fortis_drive` subscribe to `/fortis/mission_state`
with the same latched QoS, via `fortis_comms.qos_profiles.latched_qos_profile()`.
Diverging the QoS silently breaks DDS matching; keep them in lockstep.

`mission_state_node` uses `try_step()` rather than `step()` for
incoming events: an event with no matching transition in the current
state logs a warning and is silently dropped, rather than raising and
tearing down the spin loop. Events arrive from external publishers we
do not control, and a stale or premature event should never crash
the FSM.

## Parameters

Only `odrive_health_monitor_node` declares parameters:

| Param | Default | Meaning |
|---|---|---|
| `health_topic` | `/fortis/drive/odrive_health` | Source topic for the `OdriveHealth` snapshots. |
| `watchdog_timeout_s` | `1.0` | A snapshot older than this is treated as if no data exists (drive_healthy = False). The boundary is inclusive: age exactly equal to the timeout is still fresh. |

## Footgun: `CONTEXT_FIELDS` is duplicated

The list of context-guard field names lives in two files:

- `fortis_safety/mission_state_node.py` (`CONTEXT_FIELDS`), used to
  spawn one subscriber per field.
- `fortis_safety/event_console.py` (`CONTEXT_FIELDS`), used to
  validate `set <field> <bool>` commands and to advertise them in the
  help text.

They are intentionally NOT identical: the node's list additionally
contains `drive_healthy`, which the console deliberately omits — that
field is owned by `odrive_health_monitor_node`'s latched publisher,
and a second latched publisher from the console would fight it. For
every other field the two lists must stay in sync; update both when
adding one.

## Building

From `/workspace`:

```bash
colcon build --packages-select fortis_safety
source install/setup.bash
```

## Running

```bash
ros2 run fortis_safety mission_state_node
# and, when drive-fault latching is wanted:
ros2 run fortis_safety odrive_health_monitor_node
```

In a second terminal, watch the state:

```bash
ros2 topic echo /fortis/mission_state
```

In a third terminal, drive the FSM by hand:

```bash
ros2 run fortis_safety event_console
[IDLE] fortis> event start_orbit
[ORBIT] fortis> set target_pose_valid true
[ORBIT] fortis> event chassis_cam_click
[TARGETING] fortis> set ik_ok true
[TARGETING] fortis> event arm_at_view_pose
[ARM_AT_VIEW] fortis>
```

The console subscribes to `/fortis/mission_state` so the prompt shows
the current state. `help` lists all REPL commands.

## Testing

The decision logic is pure Python and runs without ROS:

```bash
cd /workspace
colcon build --packages-select fortis_safety
source install/setup.bash
colcon test --packages-select fortis_safety
colcon test-result --verbose
```

- `test/test_mission_state_machine.py` exercises every transition row,
  the wildcard `FAULT` path, and the `RESET` + operator ack guard.
- `test/test_odrive_health_monitor_node.py` covers the pure aggregation
  core (`compute_aggregate_health`): fail-safe on missing/stale data,
  the inclusive staleness boundary, and any-axis-error → unhealthy.

The ROS wrappers themselves are exercised by the cross-package launch
tests under `fortis_integration_tests`.

Lint (flake8, pep257) runs via pre-commit hooks and the `pre-commit`
job in `.github/workflows/ci.yml`, not via `colcon test`.

## Diagram

`mission_state_machine.py` is runnable as a script and emits a Mermaid
`stateDiagram-v2` of the current transition table:

```bash
python3 -m fortis_safety.mission_state_machine > mission_states.mmd
```

Paste the output into a Mermaid renderer (GitHub markdown, the
Mermaid live editor, etc.) to visualise the machine. Useful for
sanity-checking the table after edits.

## What is intentionally not in here

- Operator UI. The console is bring-up only; the production operator
  interface is not in scope for this package.
- Mission planning / behaviour trees. The FSM is hand-written, not a
  `nav2`-style BehaviorTree. Conversion is a future option, not a
  current goal.
- Persistence. The FSM is volatile. State is not saved across restarts.
- The upstream-status translator. `odrive_health_monitor_node` consumes
  the FORTIS-internal `OdriveHealth` contract; the small bridge that
  converts `odrive_ros2_control`'s status into it is not built yet.
