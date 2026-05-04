# fortis_safety

Mission-level state machine for FORTIS, plus a REPL-style operator console for driving it manually during bring-up. The state machine is the single source of truth for what the rest of the stack is allowed to do at any moment -- `fortis_drive` and `fortis_arm` both subscribe to its output topic and gate motion off of it.

## What ships here

| Entry point | Purpose |
|---|---|
| `mission_state_node` | The production node. Owns the FSM, subscribes to `/fortis/events/<name>` (one topic per `Event` enum value) and `/fortis/context/<field>` (one topic per guard field), publishes the current state on `/fortis/mission_state` (latched, `TRANSIENT_LOCAL` + `RELIABLE`, depth=1). |
| `event_console` | Bring-up tool only. A REPL that takes commands like `event start_orbit`, `set target_pose_valid true`, `state` and turns them into the right ROS publishes against the same topics `mission_state_node` subscribes to. Not a runtime component -- not launched in production, not exercised by any integration test. |
| `mission_state_machine.py` | Pure-Python FSM (no ROS). The State / Event enums and the transition table. Imported by both nodes. |

## Mission states

```
IDLE -> ORBIT -> TARGETING -> ARM_AT_VIEW -> PICK -> HOLDING -> RETURN_HOME -> IDLE
```

`FAULT` is reachable from any state. End-to-end round trip through the happy path is verified.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/fortis/mission_state` | `std_msgs/String` | published | latched (TRANSIENT_LOCAL) |
| `/fortis/events/<name>` | `std_msgs/Empty` | subscribed | one per `Event` enum value |
| `/fortis/context/<field>` | `std_msgs/Bool` | subscribed | one per known guard field |

`fortis_arm` and `fortis_drive` subscribe to `/fortis/mission_state` with the **same** QoS (TRANSIENT_LOCAL + RELIABLE, depth=1). Diverging the QoS silently breaks DDS matching -- keep them in lockstep.

## Footgun: `CONTEXT_FIELDS` is duplicated

The list of valid context-guard field names lives in **two** files:

- `fortis_safety/mission_state_node.py:38` -- `CONTEXT_FIELDS`, used to spawn one subscriber per field.
- `fortis_safety/event_console.py:26` -- `CONTEXT_FIELDS`, used to validate `set <field> <bool>` commands and to advertise them in the help text.

Both must stay in sync, or the console will offer to set fields the node ignores (or refuse fields the node accepts). The duplication is intentional -- the console is a separate process that doesn't import from the node module -- but the gotcha is real. Update both when adding a field.

## Running

```bash
ros2 run fortis_safety mission_state_node
# in a second terminal:
ros2 run fortis_safety event_console
```

The console prompt shows the current state. `help` lists commands.

## Testing

```bash
cd /workspace
source install/setup.bash
colcon test --packages-select fortis_safety
colcon test-result --verbose
```

Tests are pure-Python over the FSM itself (no ROS); the cross-package seam between this state machine and `fortis_drive` is exercised in `fortis_integration_tests`.

## What is intentionally not in here

- Operator UI -- the console is bring-up only; the production operator interface is not in scope for this package.
- Mission planning / behavior trees -- the FSM is hand-written, not a `nav2`-style BehaviorTree. Conversion is a future option, not a current goal.
- Persistence -- the FSM is volatile. State is not saved across restarts.
