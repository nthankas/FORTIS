# fortis_msgs

Custom ROS 2 message types for the FORTIS tokamak inspection robot.

This is an `ament_cmake` package because IDL message generation requires the
C++ build pipeline; an `ament_python` package can't host `.msg` files.

## Building

From `/workspace`:

```bash
colcon build --packages-select fortis_msgs
source install/setup.bash
```

## Messages

| Message | Used on (topic) | Purpose |
|---|---|---|
| `ChassisCamClick` | `/fortis/targeting/chassis_click` | Operator click on a chassis camera image. Carries the pixel coordinate, the producing camera ID, and the click timestamp. The targeting pipeline consumes this to project the click into 3-D and propose an arm view-pose. |
| `GraspCandidate` | `/fortis/planner/grasp_candidates` | A candidate grasp from the grasp planner. Carries the end-effector pose in the robot frame, planner confidence in `[0.0, 1.0]`, an approach unit vector, and a timestamp. |
| `MissionState` | `/fortis/mission_state_v2` | Richer mission-state announcement than the latched `std_msgs/String` topic published today by `fortis_safety/mission_state_node`. Carries previous state and the transition timestamp so consumers can render transitions and detect stalls. The plain `String` topic is preserved for back-compat. |
| `WheelVelocities` | `/fortis/drive/wheel_velocities`, `/fortis/drive/zero_velocities` | Per-wheel angular velocity command for the X-drive (FL, FR, BL, BR in rad/s at the wheel shaft). Published by `fortis_drive/drive_node`. |

Topic names listed above are the intended use sites at the time this package
was created. They are not enforced by the package itself.

## Actions

| Action | Used by | Purpose |
|---|---|---|
| `MoveToPose` | `fortis_arm/arm_controller` (`move_to_pose` action server) | Move the arm end-effector to a target `geometry_msgs/PoseStamped`. Goal carries the target pose; result is `bool succeeded` + `string message`; feedback is `float32 progress` in `[0.0, 1.0]`. The current `fortis_arm` implementation always returns `succeeded=false, message="kinematics not implemented"` -- the contract is the deliverable, the motion is deferred. |

## Field-naming conventions

- Wheel order is always `fl, fr, bl, br` (front/back, left/right).
- Pose / point fields use `geometry_msgs/Pose` and `geometry_msgs/Point` so
  rviz, tf, and MoveIt can consume them directly.
- Every message carries a `builtin_interfaces/Time stamp`. We don't use
  `std_msgs/Header` because none of these messages need a `frame_id` -- the
  frame is implicit in the topic, or already encoded inside the field
  (`camera_id` for the click, robot-frame convention for the pose).

## Adding a new message

1. Add `msg/MyMessage.msg`.
2. Append `"msg/MyMessage.msg"` to the `msg_files` list in `CMakeLists.txt`.
3. If the message uses a new dependency package (e.g. `sensor_msgs`), add it
   to both the `find_package` block and the `DEPENDENCIES` list of
   `rosidl_generate_interfaces`, and add `<depend>sensor_msgs</depend>` to
   `package.xml`.
4. Rebuild: `colcon build --packages-select fortis_msgs && source install/setup.bash`.
5. Verify: `ros2 interface show fortis_msgs/msg/MyMessage`.
