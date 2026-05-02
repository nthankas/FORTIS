# fortis_drive

ROS 2 wrapper around the FORTIS X-drive inverse kinematics, gated by mission state.

The node turns operator velocity commands into per-wheel angular velocities, but
only when the mission state machine (`fortis_safety/mission_state_node`) is in a
state where chassis motion is allowed. In every other state, the node publishes
explicit zeros on a separate topic so a downstream stop is never inferred from
the *absence* of a message.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | subscribed | desired chassis velocity (`linear.x` = forward, `linear.y` = strafe, `angular.z` = yaw rate) |
| `/fortis/mission_state` | `std_msgs/String` | subscribed | latched (TRANSIENT_LOCAL + RELIABLE) so we get the latest state on connect |
| `/fortis/drive/wheel_velocities` | `fortis_msgs/WheelVelocities` | published | per-wheel rad/s, one per accepted `/cmd_vel` |
| `/fortis/drive/zero_velocities` | `fortis_msgs/WheelVelocities` | published | per-wheel rad/s set to zero, one per rejected `/cmd_vel` |

The drive accepts `/cmd_vel` only while the mission state is `ORBIT` or
`RETURN_HOME`. Anything else (and the bring-up window before any state has been
received) causes the command to be rejected, zeros to be published, and a
warning to be logged at most once per second per state name.

## Parameters

None. The set of allowed states, the throttle interval, and the topic names are
module-level constants in `drive_node.py` to avoid the failure mode of "node
silently does the wrong thing because someone overrode a parameter at launch".
If parametrisation becomes necessary, declare them through `declare_parameter`
with explicit defaults that match the constants today.

## Building

From `/workspace`:

```bash
colcon build --packages-select fortis_msgs fortis_drive
source install/setup.bash
```

`fortis_msgs` must build first (it generates the `WheelVelocities` Python
class). `colcon` orders this automatically because `fortis_drive`'s
`package.xml` declares `<depend>fortis_msgs</depend>`.

## Running

```bash
ros2 run fortis_drive drive_node
```

In a second terminal, watch the gating in action:

```bash
ros2 topic echo /fortis/drive/wheel_velocities
ros2 topic echo /fortis/drive/zero_velocities
```

In a third terminal, drive the gate:

```bash
# put the mission state into ORBIT (allows motion)
ros2 run fortis_safety event_console
fortis> event start_orbit

# in a fourth terminal, send a Twist
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# you should see WheelVelocities messages
# now in event_console: `event stop`  (back to IDLE)
# Twists now produce zero_velocities instead, with one warning per second
```

## Testing

```bash
cd /workspace
colcon build --packages-select fortis_msgs fortis_drive
source install/setup.bash
python3 -m pytest src/fortis_drive/test/test_drive_node.py -v
```

The test file stands up a real `DriveNode` plus a helper publisher / subscriber
and asserts on what comes out of the wire (not on internal state). See the
module docstring in `test_drive_node.py` for the rationale.

## fortis_comms import shim

The X-drive kinematics live at `<repo>/control/fortis_comms/xdrive_kinematics.py`.
That directory is a pre-ROS Python library with no `setup.py`, so colcon does
not put it on `PYTHONPATH`. `drive_node.py` therefore walks up from `__file__`
at startup looking for `control/fortis_comms/xdrive_kinematics.py` and prepends
`<repo>/control` to `sys.path` before importing it. This avoids duplicating the
kinematics into this package, but it is a *shim* -- when `fortis_comms` gains a
proper `setup.py` and gets installed, the shim function in `drive_node.py` can
be deleted.
