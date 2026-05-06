# legacy/

Reference code from earlier design iterations. Not imported by anything in the current build, kept so we can look back at how a decision was made.

## Contents

- `coppelia_ik_reference/inverse_kinetmatic_solver.py` - Carlos Vazquez's CoppeliaSim IK solver for a 4-DOF arm. Filename typo ("kinetmatic") preserved from original. Reference only - the current arm planning happens in `sim/isaac/xdrive/lib/arm_ik_v3.py` and `sim/isaac/xdrive/canonical/xdrive_reactor_arm_v3.py`. Targets a different sim, different DH params, and a different arm design (the older 4-DOF concept before the parallel-link rework).

- `deprecated_motor_stack/motor_base.py` - Abstract `Motor` base class (connect/disconnect/command_velocity, status enum, context manager) that lived at `src/fortis_comms/fortis_comms/motor_base.py`. Replaced by ros2_control + standard ROS 2 packages.

- `deprecated_motor_stack/odrive_s1.py` - Concrete `ODrive_S1` subclass of `Motor` that hand-rolled an ODrive S1 CAN wrapper using python-can; lived at `src/fortis_comms/fortis_comms/odrive_s1.py`. Replaced by ros2_control + standard ROS 2 packages.

- `deprecated_ekf/ekf.py` - Six-state EKF (x, y, theta, vx, vy, omega) for X-drive odometry with optical-flow and IMU updates; lived at `src/fortis_comms/fortis_comms/ekf.py`. Replaced by ros2_control + standard ROS 2 packages.

- `deprecated_arm_action/move_to_pose_action_server.py` - The `move_to_pose` action server scaffold (`fortis_msgs/action/MoveToPose`) that was embedded in `src/fortis_arm/fortis_arm/arm_controller_node.py`. Goal callback gated on mission state and accepted goals always returned `succeeded=False, message="kinematics not implemented"`. Replaced by ros2_control + standard ROS 2 packages.

Don't add new code here. If something is being deprecated, a `deprecated/` subdir alongside the active code (the way `sim/isaac/xdrive/deprecated/` is laid out) keeps it next to its replacement and easier to find.
