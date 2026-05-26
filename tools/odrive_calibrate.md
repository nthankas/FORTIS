# ODrive S1 + M8325s calibration runbook

Per-S1, ~5 min each. Required **before** `fortis_control/bench_one_motor.launch.py` will do anything — `odrive_ros2_control_plugin/ODriveHardwareInterface` refuses to close the loop on an uncalibrated motor.

Calibration is a **USB-over-Serial** workflow against one S1 at a time. CAN is set up at the end; don't plug the CAN harness in until step 6.

## What you need on the bench

- One ODrive S1 (KIT-S1-M8325s-01) with 48 V supply, brake resistor wired, **no CAN cable yet**
- One M8325s motor, free to spin a few revolutions (no load on the shaft, no wheel attached)
- USB-C cable, S1 to host
- `odrivetool` already installed (the `odrive` pip dep in `docker/Dockerfile.dev` gives you this; runs on host or inside the dev container with `--device=/dev/ttyACM0` mapped through)
- A sharpie

## Reference values (from BOM)

| Param | Value | Source |
|---|---|---|
| Motor | ODrive M8325s | BOM |
| KV | 100 | BOM |
| Pole pairs | 7 | M8325s datasheet (14-pole motor) |
| Bus voltage | 48 V | BOM |
| Drive | direct (no gearbox) | BOM |
| Recommended motor current limit (bench) | 20 A | conservative for initial spin-up; bump later once thermals are characterised |
| Recommended motor velocity limit (bench) | 10 turn/s | safe for unloaded shaft; ≈ 0.66 m/s at the wheel after a 0.1016 m radius is attached |

## Prerequisite: USB passthrough on Windows + WSL2

Skip this section if you are calibrating from a native Linux machine — just plug in the S1 and `/dev/ttyACM0` shows up.

If you are running `fortis-dev` inside WSL2 on Windows, WSL2 does not see USB devices by default; they must be forwarded from the Windows host with `usbipd-win`. Inside the container itself, no extra config is needed — `docker/docker-compose.yml` already runs `fortis-dev` with `privileged: true`, so once the device is visible in WSL2 the container sees it for free.

### One-time install on Windows

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

### Each session, after plugging in the S1

From a Windows PowerShell prompt (elevated the *first* time per physical S1; a normal prompt is fine after that):

```powershell
.\tools\attach-odrive.ps1
```

The script finds any ODrive on the USB bus, binds it the first time, and forwards it into WSL2. Idempotent — running it twice is a no-op.

If PowerShell refuses to run the script ("running scripts is disabled on this system"), invoke it with `powershell -ExecutionPolicy Bypass -File .\tools\attach-odrive.ps1` instead.

Caveats:

- `usbipd attach` does **not** survive unplug or Windows restart. After either, re-run the script.
- The `bind` step requires admin. The `attach` step does not. So the very first run with a new S1 needs an elevated PowerShell; subsequent runs do not.

### Sanity check from inside WSL or the container

```bash
./tools/check-odrive-usb.sh
```

If it prints `OK:` with a device path, you are ready for step 1 below. If it reports no `/dev/ttyACM*` device, go back and run `attach-odrive.ps1` on the Windows host.

## Steps

### 1. Plug USB only, launch odrivetool

```bash
odrivetool
# Connected to ODrive ... as odrv0
```

If `odrv0` doesn't appear, check `lsusb | grep ODrive` and that no other process is holding the serial port.

### 2. Reset to factory config so the calibration is reproducible

```python
odrv0.erase_configuration()
# odrivetool will disconnect and reconnect; wait for the prompt to return
```

### 3. Set motor + bus params

```python
# Bus
odrv0.config.dc_bus_overvoltage_trip_level = 56
odrv0.config.dc_bus_undervoltage_trip_level = 8
odrv0.config.dc_max_positive_current = 40
odrv0.config.dc_max_negative_current = -10  # brake-resistor regen headroom

# Motor (M8325s)
odrv0.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv0.axis0.config.motor.pole_pairs = 7
odrv0.axis0.config.motor.torque_constant = 8.27 / 100  # = 8.27 / KV
odrv0.axis0.config.motor.current_soft_max = 20
odrv0.axis0.config.motor.current_hard_max = 30
odrv0.axis0.config.motor.calibration_current = 10
odrv0.axis0.config.motor.resistance_calib_max_voltage = 4

# Velocity / position limits (bench-safe; raise after wheels attached)
odrv0.axis0.controller.config.vel_limit = 10  # turn/s
odrv0.axis0.controller.config.vel_limit_tolerance = 1.2
```

### 4. Run the calibration sweeps

```python
# Motor resistance + inductance
odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
# wait until odrv0.axis0.current_state == AxisState.IDLE (a few seconds)
# then check:
odrv0.axis0.motor.is_calibrated   # should be True
dump_errors(odrv0)                # should be clean

# Encoder offset (motor will spin one direction, then the other)
odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
# wait again until current_state == AxisState.IDLE
odrv0.axis0.encoder.is_ready      # should be True
dump_errors(odrv0)
```

If `MOTOR_CALIBRATION` errors out with `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE`, bump `resistance_calib_max_voltage` to 6 and retry. If `ENCODER_OFFSET_CALIBRATION` errors out, the shaft is probably not free to spin — clear any load and retry.

### 5. Assign CAN node_id and enable cyclic messages

This is where you commit the FL/FR/RL/RR identity. **Sharpie the S1 case now** with FL / FR / RL / RR before you forget which one is on the bench.

```python
# Replace 0 with 1/2/3 for FR/RL/RR
odrv0.axis0.config.can.node_id = 0
odrv0.can.config.protocol = Protocol.SIMPLE
odrv0.can.config.baud_rate = 250000

# Cyclic message periods (ms). Required for /joint_states + /odrive_status
# + /controller_status to populate from odrive_ros2_control.
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv0.axis0.config.can.encoder_msg_rate_ms = 10
odrv0.axis0.config.can.iq_msg_rate_ms = 50
odrv0.axis0.config.can.torques_msg_rate_ms = 50
odrv0.axis0.config.can.bus_voltage_msg_rate_ms = 1000
odrv0.axis0.config.can.temperature_msg_rate_ms = 1000
odrv0.axis0.config.can.error_msg_rate_ms = 1000
```

### 6. Persist + reboot, verify motor spins under USB

```python
odrv0.save_configuration()
# Wait for the "Oh no!" message and reconnection. This is normal.

# After reconnect, sanity-check the spin under USB before adding CAN:
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 1.0   # 1 turn/s
# Motor should spin smoothly
odrv0.axis0.controller.input_vel = 0.0
odrv0.axis0.requested_state = AxisState.IDLE
```

If this spins, the motor is calibrated and the encoder offset is good. You are now ready for the ROS bench.

### 7. Unplug USB, plug CAN

The S1's USB port and CAN port can both be live, but for clean separation: shut down `odrivetool` cleanly (`Ctrl+D`), unplug USB. Now plug the CAN harness from the USB-CAN adapter into the S1's `CAN H` / `CAN L` screw terminals. The adapter has built-in 120 Ω termination (per BOM) so no external resistor is needed for a single-S1 bench.

Continue to the bench bring-up checklist in `src/fortis_control/README.md`.

## Repeat for each S1

The procedure is identical for FR / RL / RR — only the `node_id` (and sharpie label) changes. Run steps 1–6 for each S1 separately, USB only, one at a time.

## Once all 4 are calibrated and labelled

Daisy-chain them: USB-CAN adapter → FL → FR → RL → RR → terminator (built into the adapter handles one end; the other end of the chain needs a 120 Ω resistor across CAN H / CAN L, which the **last** S1 on the chain can provide via its on-board termination jumper — see ODrive S1 docs for the jumper position). Each S1's `node_id` is what identifies it on the bus; the physical position in the chain does not matter for addressing.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `dump_errors` reports `ERROR_DRV_FAULT` | gate driver tripped, usually wiring | power-cycle, recheck phase wires |
| Calibration spin is jerky / asymmetric | encoder offset bad | rerun `ENCODER_OFFSET_CALIBRATION` with shaft fully free |
| Motor spins backwards from expected sign | one of: motor phase order, encoder direction, kinematics sign | flip `odrv0.axis0.config.motor.direction` (`+1` / `-1`) — keep the kinematics sign convention canonical |
| `save_configuration()` complains "cannot save while axis is armed" | axis not IDLE | `odrv0.axis0.requested_state = AxisState.IDLE` first |
| After reboot, calibration is gone | `save_configuration()` failed silently | check `dump_errors(odrv0)`; common cause is a config value out of allowed range, which `save` will reject |
