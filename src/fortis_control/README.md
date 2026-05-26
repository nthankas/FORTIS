# fortis_control

`ros2_control` configuration for the FORTIS X-drive chassis. Binds four
ODrive S1s via `odrive_ros2_control` to a `JointGroupVelocityController`
the rest of the stack publishes into.

## What this package owns

- The `<ros2_control>` system block is in `fortis_description` (because
  it has to live in the URDF). This package owns:
  - `config/fortis_drive_controllers.yaml` — full 4-wheel controller
    config.
  - `config/fortis_drive_controllers_bench.yaml` — single-wheel (FL)
    bench variant.
  - `launch/bench_one_motor.launch.py` — bring up controller_manager
    against one S1 on the bench.
  - `launch/drive_hw.launch.py` — bring up controller_manager against
    all four S1s.

## Pre-requisites

1. Each S1 has been calibrated per `tools/odrive_calibrate.md`
   (motor + encoder, node_id assigned, cyclic CAN messages enabled,
   `save_configuration()` done). **The ros2_control plugin will not
   close the loop on an uncalibrated motor.**
2. `ros_odrive` has been vcs-imported into `src/`:
   ```bash
   vcs import src < tools/vendor_repos.yaml
   ```
3. The dev container has `can-utils` (already added to both
   `docker/Dockerfile.dev` and `docker/Dockerfile.dev-gpu`).

## Building

From `/workspace` inside the dev container:

```bash
colcon build --packages-up-to fortis_control
source install/setup.bash
```

`colcon` will build `odrive_ros2_control` first (it is now a vendored
source package).

## Bench bring-up checklist (one motor on the bench)

Each step is independently verifiable. If a step fails, do not move on —
the failure is the symptom; the cause is usually in that step or the
prior one.

### Step 0. The S1 is calibrated

You should have already run through `tools/odrive_calibrate.md` against
this S1. Smoke-test by re-attaching USB and:

```bash
odrivetool
> odrv0.axis0.motor.is_calibrated   # True
> odrv0.axis0.encoder.is_ready      # True
> odrv0.axis0.config.can.node_id    # 0 (or whatever you assigned)
> exit
```

Unplug USB. From here, the S1 talks only over CAN.

### Step 1. Wire CAN

USB-CAN adapter → S1 `CAN H` / `CAN L`. The adapter's built-in 120 Ω
termination handles one end of the bus; for a single-S1 bench the
S1's on-board termination jumper on the other end is OK closed (no
external resistor needed).

### Step 2. Bring up SocketCAN

The ODrive USB-CAN adapter presents as native SocketCAN. Confirm:

```bash
ip link show | grep can
```

If `can0` appears, bring it up:

```bash
sudo ip link set can0 up type can bitrate 250000
sudo ip link set can0 txqueuelen 1000
```

Verify heartbeats are flowing:

```bash
candump can0
# Expect frames at ~10 Hz with ID 0x001 (node_id 0 heartbeat).
# Ctrl-C to stop.
```

If `can0` does **not** appear and instead the adapter shows up as
`/dev/ttyACM*`, it is a slcan-style dongle — use the slcand fallback in
the launch file's docstring.

If `candump` is silent, the S1 is not powered or the CAN wiring is
inverted. Check both before moving on.

### Step 3. Launch the controller_manager

```bash
ros2 launch fortis_control bench_one_motor.launch.py
```

Watch the logs. You should see:

- `controller_manager`: `Loaded fortis_chassis_drive` (the ros2_control system)
- `controller_manager`: `Successful 'configure' of hardware` for `odrive_ros2_control_plugin/ODriveHardwareInterface`
- `joint_state_broadcaster spawner`: `configured … activated`
- `wheel_velocity_controller spawner`: `configured` (but not activated, by design — `--inactive` in the launch)

Common failures:
- `Failed to read 'node_id' parameter` → the xacro did not render the
  `<param name="node_id">` block. Re-check that the launch is passing
  `wheels:=fl`.
- `Failed to open CAN interface 'can0'` → step 2 didn't take. Re-run
  `ip link set can0 up`.
- `Could not get heartbeat from node X` → step 2 silently failed even
  though the interface is up. Run `candump can0` in another terminal
  while the launch is running and see if anything is on the bus.

### Step 4. Activate the controller (this arms the axis)

```bash
ros2 control switch_controllers --activate wheel_velocity_controller
```

`odrive_ros2_control` handles axis state automatically — activating the
controller sends `AXIS_STATE_CLOSED_LOOP_CONTROL` to the S1 over CAN
(see `odrive_hardware_interface.cpp:set_axis_command_mode`). The S1's
status LED should go solid (was blinking in IDLE). The plugin also
auto-selects velocity control mode because that is the interface the
controller claims.

There is no separate `/request_axis_state` service to call. That service
exists on the standalone `odrive_node`; the `odrive_ros2_control` plugin
does not advertise it.

### Step 5. Spin the motor

```bash
ros2 topic pub --once /wheel_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [2.0]}"
```

The motor should spin at 2 rad/s (~0.32 turn/s). At the wheel radius of
0.1016 m, that's ~0.20 m/s at the contact patch — slow enough to be
obviously safe on the bench.

Watch the joint state stream to verify the encoder feedback:

```bash
ros2 topic echo /joint_states
# position[0] should increment monotonically while spinning;
# velocity[0] should hover near 2.0.
```

### Step 6. Stop and disarm

```bash
# Command zero velocity (motor stops but stays armed)
ros2 topic pub --once /wheel_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# Deactivate the controller — this sends AXIS_STATE_IDLE to the S1
ros2 control switch_controllers --deactivate wheel_velocity_controller
```

The S1 status LED should go back to blinking.

### Step 8 (optional). Drive it from the actual stack

With the bench launch still running, in another terminal:

```bash
ros2 launch fortis_bringup bringup.launch.py
ros2 run fortis_safety event_console
fortis> event start_orbit

# In another terminal:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

`fortis_drive.drive_node` will run the X-drive IK and publish a
`Float64MultiArray` of `[fl, fr, rl, rr]` velocities. Only index 0 (FL)
will actually do anything because the bench-variant controller has only
one joint loaded; the other three indices are silently dropped by the
controller. This is the intended behaviour for the single-motor bench.

## Bench bring-up: full chassis (when all four S1s are on the bench)

Same as above with:

- `tools/odrive_calibrate.md` repeated for each S1, node_ids 0/1/2/3
- All four S1s daisy-chained, last one's termination jumper closed
- `ros2 launch fortis_control drive_hw.launch.py` instead of
  `bench_one_motor.launch.py`
- Step 6 array is `[fl, fr, rl, rr]` — e.g. `[2.0, 2.0, 2.0, 2.0]` to
  drive forward at constant speed on all four wheels.

## How this composes with the rest of the stack

```
                        /cmd_vel
                            |
                            v
            fortis_drive.drive_node
            - IK via fortis_comms
            - gated by /fortis/mission_state
                            |
                            v
       /wheel_velocity_controller/commands  (Float64MultiArray [fl,fr,rl,rr])
                            |
                            v
              controller_manager  (this launch)
              + JointGroupVelocityController
                            |
                            v
         odrive_ros2_control_plugin/ODriveHardwareInterface
                            |
                            v
                       SocketCAN can0
                            |
                            v
                       4x ODrive S1
```

`fortis_drive` was the integration point already in place. This package
gives the topic on the bottom-left of that diagram an actual consumer.

## Known limitations

- **No error surfacing yet.** Per the upstream `odrive_ros2_control`
  README: "If an ODrive disarms for some reason (e.g. undervoltage),
  the application that connects to ros2_control will currently not be
  notified." Treat `/odrive_status` as the source of truth for axis
  health during bench work; the mission FSM cannot react to S1 faults
  until this is plumbed.
- **Bench variant is hardcoded to FL.** If your bench has FR instead,
  hand-edit `fortis_drive_controllers_bench.yaml` and the launch's
  `wheels:=fl` arg in the same commit. Don't introduce a launch arg
  for this — the inevitable misuse (controller loaded for FL, S1 on
  bench is FR) would silently command the wrong motor.
- **Renaming BL/BR → RL/RR is deferred.** The kinematics module
  (`fortis_comms.xdrive_kinematics`) and the `WheelVelocities` message
  still use BL/BR; the URDF + ros2_control config use RL/RR. The
  rename happens at the boundary in `drive_node._wheel_command_to_controller_array`.
  Tracked as a follow-up; see the `fortis_drive` module docstring.
