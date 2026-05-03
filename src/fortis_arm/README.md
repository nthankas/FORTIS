# fortis_arm

Arm controller seam for FORTIS, gated by mission state. **Kinematics are deferred** -- this package is the architectural contract scaffold (action server + gripper services + state gating), not the motion implementation. The contract is what unblocks the targeting, grasp, and operator-UI work; the IK / trajectory / Teensy serial pipeline lands in a follow-up pass.

## What ships here

| Endpoint | Type | Purpose |
|---|---|---|
| `move_to_pose` | action server, `fortis_msgs/action/MoveToPose` | Move the end-effector to a `geometry_msgs/PoseStamped`. Goal callback rejects when current mission state is not in the allowed set. Accepted goals always return `succeeded=False, message="kinematics not implemented"`. |
| `open_gripper` | service, `std_srvs/srv/Trigger` | Stub; rejects on disallowed state with `success=False` + reason; returns `success=False, message="gripper actuation not implemented"` on allowed state. |
| `close_gripper` | service, `std_srvs/srv/Trigger` | Same shape as `open_gripper`. |
| `/fortis/mission_state` | subscription, `std_msgs/String` | Latched (TRANSIENT_LOCAL + RELIABLE, depth=1). Same topic and QoS as `fortis_drive`. |

### Allowed mission states for arm motion

`ARM_AT_VIEW`, `INSPECT`, `PICK`, `HOLDING`, `RETURN_HOME`. Any other state -- including "no `/fortis/mission_state` has been received yet" -- causes the action goal to be rejected at the gate and the gripper services to return `success=False`.

### Why `std_srvs/Trigger` for the gripper

Open and close are binary commands from the operator with no payload. `Trigger` (empty request, `bool success` + `string message` response) matches that contract exactly. A custom service in `fortis_msgs` would add a build dependency for no behavioural gain. If a future requirement needs (for example) a target jaw position, swap to a custom service then.

## Hardware reference

Captured here so the future Teensy / kinematics pass starts from a known wiring + mechanical setup, not from rediscovery.

### Motion controller

- **Teensy 4.1** acts as the arm motion controller. Connected to the Jetson on **Port 4** over USB serial.
- **TeensyStep** generates step / direction signals for J1, J2, J3 driving **CL57T-V41** closed-loop stepper drivers.
- **SN74HCT245N** octal bus transceivers shift the Teensy's 3.3 V step / dir lines up to the 5 V the CL57T expects.
- **Hitec D845WP** waterproof servo on J4 driven directly off a Teensy PWM pin (no level shift required for the servo signal).
- **ServoCity 3219-0001-0002** rotational servo gripper as the end-effector.

### Joint reference

| Joint | Motor | Reduction / feedback |
|---|---|---|
| J1 | NEMA 17 stepper | Cricket MK II 25:1 cycloidal gearbox; CL57T closed-loop encoder |
| J2 | NEMA 23 stepper | 50:1 planetary gearbox + 1000 PPR incremental encoder; CL57T closed-loop |
| J3 | NEMA 17 stepper | Cricket MK II 25:1 cycloidal gearbox; CL57T closed-loop encoder |
| J4 | Hitec D845WP servo | Internal feedback; commanded via PWM |

### Position recovery

Closed-loop feedback is provided by the CL57T's own encoder per joint -- there are no separate absolute encoders on the arm. On reboot the controller reads the **last-known position from a position file** rather than running a homing routine. Cycloidal gearboxes are near self-locking under the arm's payload, so position drift while powered down is small enough that file-based recovery is acceptable.

When IK lands, the Teensy node will need to reconcile the file-recovered position against the CL57T's reported position at startup and refuse to act if they disagree by more than a configurable tolerance. The scaffold here does not implement that check.

## Why this design

- **State gating is the same shape as `fortis_drive`.** Same topic, same QoS, same throttled rejection log pattern. Reading one node tells you how the other works. Diverging the QoS would silently break DDS matching and is not worth the flexibility.
- **The action contract exists before the kinematics.** Targeting and the operator UI need a stable thing to call so they can be developed in parallel; them getting `succeeded=False, "kinematics not implemented"` back is the worst case, but it is a deterministic worst case, not a hung goal or a missing endpoint.
- **The gripper is two services rather than one with an "open" boolean.** Two endpoints map directly to two operator buttons and to two distinct mission events (`OPEN_GRIPPER`, `CLOSE_GRIPPER`) when those land in `fortis_safety`. Single-service-with-flag would force every caller to know the convention.

## Testing

Run from the workspace root inside the dev container:

```bash
cd /workspace
colcon build --symlink-install --packages-select fortis_msgs fortis_comms fortis_arm
source install/setup.bash
colcon test --packages-select fortis_arm
colcon test-result --verbose
```

Tests:
- `test/test_state_gating.py` -- parametrized over every state in `MissionStateMachine.State`. Action goals are accepted (and return the stub result) in `ALLOWED_ARM_STATES` and rejected at the gate elsewhere; gripper services follow the same gating.
- `test/test_bringup.py` -- if `/fortis/mission_state` has not yet been published when a request arrives, the gate must reject. Mirrors the same race covered in `fortis_drive/test/test_drive_node.test_no_state_received_rejects_cmd_vel`.
- `test/test_flake8.py`, `test/test_pep257.py` -- lint scaffolds, scoped to this package via `Path(__file__).resolve().parent.parent`.

## What is intentionally not in here

- IK or FK for the 4-joint arm.
- Trajectory generation or motion smoothing.
- Teensy serial protocol.
- Position-file read / write.
- CL57T encoder reconciliation against the recovered position.
- Any actual gripper open / close command.

These all land in the next pass over this package once the Teensy firmware contract is locked.
