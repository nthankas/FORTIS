# fortis_arm

ROS 2 stack for the FORTIS 4-DOF arm: kinematics, pose moves, the serial bridge to the Teensy 4.1 firmware, and gripper services. All motion is gated by mission state.

## Nodes

| Executable | What it does |
|---|---|
| `arm_motion` | Solves IK (`arm_ik.py`) for every `/fortis/target_pose` and publishes the latched `/fortis/context/ik_ok` flag. Serves the `move_to_pose` action (`fortis_msgs/action/MoveToPose`): picks the nearest IK solution with a collision-free path (`arm_planner.py`: chassis, floor and self-collision checks, with a raised via pose when the direct move is blocked), commands the move, reports progress, and soft-stops on cancel or timeout. Goals are accepted in `TARGETING`, `ARM_AT_VIEW`, `INSPECT` and `PICK`. |
| `teensy_bridge` | Owns the USB serial link to the firmware (`firmware/teensy/PROTOCOL.md`). Publishes `ArmStatus`, `/joint_states` and diagnostics, translates `/fortis/arm/command` into joint targets, and exposes enable / disable / home / clear_faults services. Before each stepper move it scales the J1..J3 speed limits so all three joints finish together; the firmware runs the move with a trapezoidal profile. |
| `arm_controller` | `open_gripper` / `close_gripper` services (`std_srvs/Trigger`), accepted in `ARM_AT_VIEW`, `INSPECT`, `PICK`, `HOLDING` and `RETURN_HOME`. |

`config/arm_calibration.yaml` holds the step and servo scaling; the values still need a bench calibration pass. `tools/mock_teensy.py` stands in for the firmware in tests.

## Hardware reference

Source of truth is `FORTIS_FINAL_BOM`.

### Motion controller

- **Teensy 4.1** (600 MHz Cortex-M7) acts as the arm motion controller. Connected to the Jetson on **USB Port 4** over USB serial.
- **TeensyStep** generates step / direction signals for J1, J2, J3 driving **StepperOnline CL57T-V41** closed-loop stepper drivers (0-8 A, 18-50 VDC, reads 1000 PPR encoder, hardware closed-loop pos / vel / current). Powered from the 48 V drive bus (min 18 V).
- An **8-channel bi-directional bus transceiver (DIP-20)** shifts the Teensy's 3.3 V step / dir lines up to the 5 V the CL57T optocouplers expect.
- **Hitec D845WP** waterproof servo on J4 driven directly off a Teensy hardware PWM pin (180 oz-in @ 7.4 V; no level shift required for the servo signal).
- **ServoCity 3219-0001-0002** servo-driven parallel gripper kit (75 oz-in @ 6 V, steel gears) as the end-effector, also commanded via Teensy hardware PWM.
- Hardware quadrature inputs on the Teensy are reserved for independent encoder verification (cross-check against the CL57T's own closed loop).

### Joint reference

| Joint | Motor | Reduction / feedback |
|---|---|---|
| J1 (base yaw) | NEMA 17 closed-loop stepper 65 Ncm (17HS24-2104-ME1K, 2.1 A, 1000 PPR magnetic encoder, 3ch differential) | Cricket Drive MK II 25:1 cycloidal gearbox; CL57T closed-loop |
| J2 (shoulder pitch) | NEMA 23 stepper (CN-23HS22-2804-HG50-ME1K, StepperOnline) with 1000 PPR (4000 CPR) magnetic encoder | 50:1 high-precision planetary gearbox + Ruland FHD-MCL-14-F shaft collar; CL57T closed-loop |
| J3 (elbow pitch) | NEMA 17 closed-loop stepper (17HS24-2104-ME1K, same part as J1) | Cricket Drive MK II 25:1 cycloidal gearbox; CL57T closed-loop |
| J4 (wrist) | Hitec D845WP servo (180 oz-in @ 7.4 V, waterproof) | Internal servo feedback; commanded via Teensy hardware PWM |

Arm structural tubing: Clearwater Composites 1" ID x 1 1/8" OD pultruded square carbon fiber. Link lengths are in `fortis_description/urdf/fortis_constants.xacro`.

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
- `test/test_arm_ik.py` -- FK/IK round trip, both elbow branches, unreachable targets, joint limits.
- `test/test_arm_planner.py` -- collision checks, direct and via-pose paths.
- `test/test_arm_motion.py` -- `ik_ok` flag, a full `MoveToPose` against a simulated arm, goal rejection outside allowed states.
- `test/test_teensy_bridge.py`, `test/test_teensy_protocol.py` -- bridge against the firmware mock, frame encode/decode.
- `test/test_state_gating.py`, `test/test_bringup.py` -- gripper service gating per mission state.
