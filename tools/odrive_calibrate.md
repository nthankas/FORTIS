# ODrive S1 + M8325s calibration runbook (Jetson via SSH)

Configure and calibrate four ODrive S1s using `odrivetool` running **directly on the Jetson** via SSH (Tailscale or any other reachable network path). ~5 min per motor. No Windows USB-over-WSL, no Docker container, no GUI.

Required before `fortis_control/bench_one_motor.launch.py` will do anything — the ODrive ros2_control plugin refuses to close the loop on an uncalibrated motor.

**Hardware assumption:** all four S1s have their 2.0 Ω brake resistor and the M8325s integrated NTC thermistor soldered and terminated. Motors free to spin, no wheels mounted.

**Topology assumption:** the Jetson is the same machine that will run the robot. We're using it for calibration because (a) native Linux means `/dev/ttyACM0` and udev rules Just Work, (b) anything we set up here — udev rules, CAN interface, group memberships — is exactly what the deployment will need anyway. Calibration on the Jetson is one less environment to debug later.

## Equipment checklist

- 4 × ODrive S1 (KIT-S1-M8325s-01), brake resistor + thermistor pre-wired
- 4 × M8325s motors, shafts free
- 1 × 48 V bench power supply (with a clear power switch)
- 1 × USB-C-to-(whatever the Jetson has) cable, S1 ↔ Jetson
- 1 × USB-CAN adapter with built-in 120 Ω termination (for the post-calibration bus assembly)
- CAN-bus wire (CAN H + CAN L pair, enough to daisy-chain all 4)
- A sharpie
- The Jetson powered up, on the network, and physically within USB-cable reach of the bench

## Shell legend

Only two shells matter in this version:

| Tag | Where |
|---|---|
| `[laptop]` | bash on your Windows laptop (in WSL2 Ubuntu). Used only to SSH to the Jetson. |
| `[jetson]` | bash on the Jetson (you arrive via `./stack ssh` or `ssh user@<host>`). |
| `[REPL]` | the Python prompt that `odrivetool` drops you into on the Jetson. |

## One-time Jetson setup (do this once ever)

SSH in:

`[laptop]`
```bash
cd ~/FORTIS
./stack ssh                           # uses FORTIS_JETSON_HOST from .env
# OR if .env isn't set up, plain:
ssh <user>@<jetson-tailscale-name>
```

Then on the Jetson, install everything `odrivetool` needs:

`[jetson]`
```bash
sudo apt update
sudo apt install -y python3-pip pipx
pipx install odrive
pipx ensurepath
exec bash                             # reload PATH so 'odrivetool' resolves

# Install ODrive udev rules so non-root users (you) can talk to the USB device.
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add yourself to the dialout group for /dev/ttyACM* access without sudo.
sudo usermod -a -G dialout $USER

# Pick up the new group membership without logging out:
exec sudo -i -u $USER bash -l         # or just log out and back in
```

Verify it worked:

`[jetson]`
```bash
which odrivetool                      # should print ~/.local/bin/odrivetool
groups | grep dialout                 # should include 'dialout'
```

---

# Per-motor calibration loop

Do steps 1–15 below for each S1 in the order **FL → FR → RR → RL** (CAN daisy-chain order: each board's node_id matches its physical position on the harness, with RL at the terminated far end). Only one thing changes between motors: the `node_id` you set in step 11.

| Wheel | Chain position | `node_id` for step 11 | Sharpie label for step 15 |
|---|---|---|---|
| FL (do first) | 1st (next to USB-CAN adapter) | `0` | FL |
| FR | 2nd | `1` | FR |
| RR | 3rd | `2` | RR |
| RL (do last) | 4th (terminated end — close the on-board termination jumper here) | `3` | RL |

## 1. Confirm 48 V supply is OFF.
No LED, no fan. Wait 5 s after switching off for the bus caps to bleed.

## 2. Wire the next S1 to the supply.
Supply + → S1 `VBUS`, supply – → S1 `GND`. Double-check polarity.

## 3. Plug USB-C from the S1 to the **Jetson** (not your laptop).
**No CAN cable yet** — CAN happens after all four are calibrated.

## 4. Turn the 48 V supply ON.
S1 status LED should be solid (green or blue). Red = fault — power down, recheck wiring.

## 5. SSH to the Jetson if you don't already have a shell open there.

`[laptop]`
```bash
./stack ssh
```

## 6. `[jetson]` confirm the S1 is visible.
```bash
ls /dev/ttyACM*
```
Expect `/dev/ttyACM0`. If "No such file or directory", check the cable + supply + LED.

## 7. `[jetson]` launch the REPL.
```bash
odrivetool
```
After a few seconds: `Connected to ODrive S1 <serial> (firmware v0.6.11) via USB as odrv0`, and the prompt changes to `In [1]:`. You are now in `[REPL]`.

## 8. `[REPL]` wipe the controller back to factory defaults.
```python
odrv0.erase_configuration()
```
The S1 will disappear and reconnect. Wait for the reconnection banner before continuing:
```
Connected to ODrive S1 <serial> (firmware v0.6.11) via USB as odrv0
```

**Sanity check the handle is live** before pasting anything else:
```python
print(odrv0.vbus_voltage)             # expect ~48
print(odrv0.serial_number)            # expect a 12-char hex string
```

If either prints nothing, `odrv0` is a stale handle from before the reboot. Exit (`Ctrl+D`), re-run `odrivetool`, and retry step 8. **Don't paste the config block until both prints return real values** — pasting against a stale handle silently does nothing.

## 9. `[REPL]` paste the canonical config block.
Same block for every motor (no per-motor changes here — the `node_id` is set later in step 11):

```python
# Bus
odrv0.config.dc_bus_overvoltage_trip_level = 56
odrv0.config.dc_bus_undervoltage_trip_level = 8
odrv0.config.dc_max_positive_current = 40
odrv0.config.dc_max_negative_current = -10

# Brake resistor (2.0 Ω kit standard, soldered in)
odrv0.config.brake_resistor0.enable = True
odrv0.config.brake_resistor0.resistance = 2.0

# Motor (M8325s)
# motor_type MUST be PMSM_CURRENT_CONTROL on firmware 0.6.x for any 3-phase
# brushless motor. HIGH_CURRENT is the legacy v3-era name; using it on 0.6.x
# causes calibration to fail with a pole-pair mismatch (the firmware runs
# PMSM math against a motor configured as the legacy type).
odrv0.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
odrv0.axis0.config.motor.pole_pairs = 20                # M8325s 100KV datasheet (authoritative): Pole Pairs = 20. Do NOT change.
odrv0.axis0.config.motor.torque_constant = 8.27 / 100   # M8325s datasheet: torque constant = 0.083 Nm/A
odrv0.axis0.config.motor.current_soft_max = 20
odrv0.axis0.config.motor.current_hard_max = 30
# calibration_current = HALF of continuous current rating per ODrive wizard.
# M8325s continuous = 40 A (free air), so calibration_current = 20.
# This was 10 in an earlier rev of this doc; that caused UNBALANCED_PHASES
# during MOTOR_CALIBRATION because the 0.24 V drop across the 24 mΩ phase R
# was too close to the S1's shunt-noise floor — phase resistances looked
# mismatched when they weren't. 20 A doubles the voltage drop (~0.48 V) and
# resistance measurement comes out balanced.
odrv0.axis0.config.motor.calibration_current = 20
odrv0.axis0.config.motor.resistance_calib_max_voltage = 4

# Pre-populate phase R/L from M8325s 100KV datasheet and mark them VALID.
# This tells the firmware to skip its own resistance / inductance measurement
# during MOTOR_CALIBRATION. For very low-resistance motors like the M8325s
# (24 mΩ), the in-firmware measurement can produce ERROR_UNBALANCED_PHASES
# even on a healthy motor because shunt-current noise is comparable to the
# tiny voltage drop. Datasheet values are more accurate than the measurement
# anyway, so bypassing is the recommended path for known motors.
odrv0.axis0.config.motor.phase_resistance = 0.024       # M8325s 100KV datasheet: 24 mΩ phase-neutral
odrv0.axis0.config.motor.phase_inductance = 9.9e-6      # M8325s 100KV datasheet: 9.9 µH phase-neutral
odrv0.axis0.config.motor.phase_resistance_valid = True
odrv0.axis0.config.motor.phase_inductance_valid = True

# Lock-in current for encoder calibration (firmware uses this to hold the
# rotor at a known position during the offset sweep). Per ODrive docs:
# Lock-in current for ENCODER_OFFSET_CALIBRATION. Per the ODrive wizard,
# half of the motor's continuous current rating. M8325s continuous = 40 A,
# so calibration_lockin.current = 20. Without this, encoder calibration can
# silently fail (no rotor lock).
odrv0.axis0.config.calibration_lockin.current = 20

# Velocity / position limits (bench-safe; raise after wheels attached)
odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.controller.config.vel_limit_tolerance = 1.2

# Default control mode = velocity, with direct passthrough. The post-erase
# default is POSITION_CONTROL, which makes `input_vel = ...` silently no-op
# (the controller is looking at input_pos instead). Setting VELOCITY_CONTROL
# here makes the bench spin test in step 13 work as written, AND aligns with
# what fortis_control's joint_velocity_controller will request later.
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode = InputMode.PASSTHROUGH

# Encoder source (M8325s kit uses the S1's onboard magnetic sensor — reads
# the rotor magnet through the back of the S1 PCB). Without this, ENCODER_OFFSET
# _CALIBRATION returns ProcedureResult.NO_RESPONSE because no encoder is wired
# into the control loop.
odrv0.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv0.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0

# Motor thermistor (M8325s integrated NTC, soldered to S1 TEMP/GND)
# gpio_pin = 4 confirmed on FL S1 (board rev as of 2026-05-27 batch).
# If a future S1 board rev returns garbage temperatures at idle, drop back to
# the discovery sweep at step 9b to re-identify the TEMP GPIO mapping.
odrv0.axis0.motor.motor_thermistor.config.gpio_pin = 4
odrv0.axis0.motor.motor_thermistor.config.r_ref = 10000
odrv0.axis0.motor.motor_thermistor.config.beta = 3435
odrv0.axis0.motor.motor_thermistor.config.t_ref = 25
odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower = 80
odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper = 100
odrv0.axis0.motor.motor_thermistor.config.enabled = True
```

### 9a. `[REPL]` save and reboot to arm the brake resistor.

This step exists because the brake chopper doesn't arm until the firmware reboots and re-reads the config. Without this, MOTOR_CALIBRATION fails with `BRAKE_RESISTOR_DISARMED`.

```python
odrv0.save_configuration()
# Wait for the "Oh no odrv0 disappeared" message and reconnection.
# Verify the handle is fresh:
print(odrv0.vbus_voltage)       # should print ~48
odrv0.clear_errors()
dump_errors(odrv0)              # should be silent
```

### 9b. `[REPL]` (fallback only) discover the correct thermistor `gpio_pin`.

For this board batch the correct pin is **`4`**, already baked into the canonical config block. Skip this step unless `odrv0.axis0.motor.motor_thermistor.temperature` reads garbage (e.g. < 0 °C or > 80 °C at idle) — that indicates a silent board-rev change and you'll need to re-sweep:

```python
for pin in [5, 4, 3, 2]:
    odrv0.axis0.motor.motor_thermistor.config.gpio_pin = pin
    odrv0.axis0.motor.motor_thermistor.config.enabled = True
    print(f"pin={pin}, temp={odrv0.axis0.motor.motor_thermistor.temperature}")
    odrv0.axis0.motor.motor_thermistor.config.enabled = False
```

Whichever pin reports a sane room-temperature value (~25 °C) is the right one. Bake it back into the canonical config block and save.

## 10. `[REPL]` run the two calibration sweeps.

The "did it succeed?" check uses `dump_errors` rather than version-specific status attributes — boolean flags in `odrv0.axis0.motor` have moved between firmware revisions.

```python
# Motor resistance + inductance (DC injection, no rotation, ~3 sec)
odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
# Wait ~3 seconds — listen for a brief whirring noise from the motor.
dump_errors(odrv0)                    # expect no output (clean)

# Encoder offset (motor will spin one direction, then the other, ~6 sec)
odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
# Wait until the motor stops physically spinning.
dump_errors(odrv0)                    # expect no output (clean)
```

**If `dump_errors` prints any errors** — stop, don't proceed. See troubleshooting at the bottom; most common is `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE` (phase wires not connected) or shaft not free to spin.

## 11. `[REPL]` set CAN `node_id` + cyclic message rates. **Change `node_id` per the table above.**
```python
odrv0.axis0.config.can.node_id = 3   # FL=0, FR=1, RR=2, RL=3 (chain order) — CHANGE THIS PER MOTOR
odrv0.can.config.protocol = Protocol.SIMPLE
odrv0.can.config.baud_rate = 250000

odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv0.axis0.config.can.encoder_msg_rate_ms = 10
odrv0.axis0.config.can.iq_msg_rate_ms = 50
odrv0.axis0.config.can.torques_msg_rate_ms = 50
odrv0.axis0.config.can.bus_voltage_msg_rate_ms = 1000
odrv0.axis0.config.can.temperature_msg_rate_ms = 1000
odrv0.axis0.config.can.error_msg_rate_ms = 1000
```

## 12. `[REPL]` save to flash and reboot the S1.
```python
odrv0.save_configuration()
# You'll see "Oh no odrv0 disappeared" and a reconnection. Normal. Wait for In [N]:.
```

## 13. `[REPL]` verify the motor spins under USB.
```python
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 1.0          # 1 turn/s
# Watch the shaft — should spin smoothly for a few seconds.
odrv0.axis0.controller.input_vel = 0.0
odrv0.axis0.requested_state = AxisState.IDLE
```

**If the motor did NOT spin smoothly, stop here.** Don't move on with a bad calibration.

## 14. `[REPL]` exit with `Ctrl+D`.
You drop back to the `[jetson]` shell.

## 15. Power down, unplug, label, set aside.
In this order:
1. **Turn the 48 V supply OFF.** Wait 5 s for bus caps to bleed.
2. Unplug USB-C from the S1.
3. **Sharpie the wheel label** (FL / FR / RL / RR) on the S1 case before you forget which one was on the bench.
4. Set this S1 aside.

## Next motor

Bring the next motor (FR, then RL, then RR) onto the bench and **go back to step 1**. Only `node_id` in step 11 and the sharpie label in step 15 change.

---

# All four calibrated — assemble the CAN bus

Once FL / FR / RL / RR are all labelled and set aside:

## 16. Confirm 48 V supply is OFF.

## 17. Daisy-chain CAN: **USB-CAN adapter → FL → FR → RL → RR**.
Wire each S1's `CAN H` to its neighbour's `CAN H`, and `CAN L` to `CAN L`. Polarity matters. Physical position in the chain is unrelated to `node_id`.

## 18. Flip the on-board 120 Ω termination jumper on the **RR** S1 only.
RR is the last S1 in the chain. The USB-CAN adapter already terminates the *other* end internally, so no external resistor is needed.

## 19. Plug the USB-CAN adapter into the **Jetson** (not your laptop).

## 20. `[jetson]` bring up the CAN interface.

The USB-CAN adapter shows up as a SocketCAN interface (usually `can0`). Bring it up at the same baud as the S1s:

```bash
ip link show                                                 # should list 'can0'
sudo ip link set can0 up type can bitrate 250000
candump can0                                                 # press Ctrl+C to stop
```

You should see a steady stream of CAN frames — heartbeats from all four S1s at 10 Hz (100 ms period set in step 11), plus encoder/iq/torques/etc. at their configured rates. The `candump` output is your live confirmation that all four motors are alive and addressable.

If `can0` doesn't appear in `ip link show`, the USB-CAN adapter's kernel driver may not be loaded. Common drivers: `gs_usb` (CANable), `peak_usb` (PCAN). `dmesg | tail -30` after plugging in the adapter usually says which one is matching, or what's missing.

## 21. Turn the 48 V supply ON.
All four S1 status LEDs should come on solid. Run `candump can0` for a few seconds; you should see frames from `node_id` 0, 1, 2, and 3 cycling through.

## 22. Continue to the bench bring-up checklist in `src/fortis_control/README.md`.

The `fortis_control` ros2_control plugin reads from `can0`, addresses the S1s by `node_id`, and consumes their cyclic message stream. Once `candump` confirms all four are heartbeating, the container-side bring-up is the next step. ros2_control can run inside the `fortis-dev` container on the Jetson — Docker on the Jetson does NOT have the WSL/docker-desktop split that bit us on the Windows side, so the container sees `can0` directly via `network_mode: host`.

---

## Reference values (from BOM)

| Param | Value | Source |
|---|---|---|
| Motor | ODrive M8325s | BOM |
| KV | 100 | BOM |
| Pole pairs | 20 | M8325s 100KV datasheet (40-pole outrunner) |
| Phase resistance | 24 mΩ | M8325s 100KV datasheet (phase-neutral) |
| Phase inductance | 9.9 µH | M8325s 100KV datasheet (phase-neutral) |
| Continuous current rating | 40 A free air / 60 A forced air | M8325s 100KV datasheet |
| Peak current rating | 80 A (3-second) | M8325s 100KV datasheet |
| Bus voltage | 48 V | BOM |
| Drive | direct (no gearbox) | BOM |
| Brake resistor | 2.0 Ω | BOM |
| Motor NTC thermistor | 10 kΩ @ 25 °C, β ≈ 3435 | M8325s integrated NTC |
| Bench current limit | 20 A | conservative; raise after thermal characterisation |
| Bench velocity limit | 10 turn/s | safe for unloaded shaft |
| Bench thermistor lower / upper | 80 °C / 100 °C | conservative defaults; M8325s rated to ~120 °C |

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ls /dev/ttyACM*` says "No such file or directory" on Jetson | S1 not powered, USB-C unseated, or motor cable to wrong jack | confirm LED solid, replug USB-C; `dmesg \| tail` after plugging shows whether the kernel enumerated it |
| `odrivetool` says "Device permissions are not set up" | udev rules not installed (one-time Jetson setup skipped) | re-run the udev install in "One-time Jetson setup" |
| `odrivetool` loops `LIBUSB_ERROR_ACCESS` | udev rules in place but you're not in the `plugdev` group (or it never fired) | log out and back in to pick up group membership; or `sudo odrivetool` as a fallback |
| After `erase_configuration()`, all attribute reads return nothing | `odrv0` is a stale handle — the device rebooted, the Python name didn't rebind | `Ctrl+D` to exit, `odrivetool` to reconnect, verify with `print(odrv0.vbus_voltage)` before pasting anything |
| `dump_errors` reports `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE` | phase wires open or shorted, or one phase disconnected | power down, ohm out each of the three motor wires to verify continuity to the windings |
| Encoder calibration in step 10 fails or motor spin is jerky | shaft not free; load or friction on the motor | clear any obstruction, retry |
| Motor in step 13 spins backwards from expected sign | phase-wire order or encoder direction | `odrv0.axis0.config.motor.direction = -1` (or `+1`) to flip; re-save with `save_configuration()`; keep kinematics sign convention canonical so URDF stays consistent |
| `save_configuration()` complains "cannot save while axis is armed" | axis isn't IDLE | `odrv0.axis0.requested_state = AxisState.IDLE` first, then re-save |
| After reboot in step 12, calibration is gone | `save_configuration()` failed silently | run `dump_errors(odrv0)`; common cause is a config value outside allowed range, which `save` quietly rejects |
| `ERROR_MOTOR_OVER_TEMP` immediately on `CLOSED_LOOP_CONTROL` (step 13), even cold | thermistor lead broken or unseated at TEMP / GND | multimeter across the two thermistor leads should read ≈ 10 kΩ at room temp; reseat under the screw |
| `ERROR_DC_BUS_OVER_VOLTAGE_TRIP` during step 13 spin | brake-resistor config disagrees with actual wiring | confirm `brake_resistor0.enable = True` and `brake_resistor0.resistance = 2.0` in step 9; ohm out the resistor itself |
| `ERROR_UNBALANCED_PHASES` during step 10 MOTOR_CALIBRATION | Firmware's auto-measurement of phase resistance can't reliably resolve the M8325s's tiny 24 mΩ phase R against shunt noise — three phase Rs read mismatched even on a healthy motor | **Primary fix:** the canonical config block already pre-populates `phase_resistance = 0.024`, `phase_inductance = 9.9e-6`, and sets both `phase_resistance_valid = True` / `phase_inductance_valid = True`. This causes MOTOR_CALIBRATION to skip the auto-measurement entirely. If you somehow have an older copy of the config block, re-paste from the doc. **Secondary check (if it still fails):** multimeter phase-to-phase R on the motor leads with ODrive powered off — all three pairings should read within ~1 mΩ of each other (~48 mΩ phase-to-phase). Significant mismatch = bad winding or loose crimp on a motor lead. **DO NOT touch `pole_pairs`** — it is 20 per the M8325s 100KV datasheet, full stop. |
| `can0` not appearing in `ip link show` after plugging in USB-CAN adapter | adapter's kernel driver not loaded | `dmesg \| tail -30` after plugging in; install missing driver via apt (`can-utils` and `gs_usb` for CANable, etc.) |
