# fortis_bringup

Top-level launch composition for FORTIS. Currently scaffolding only — every launch file is a stub that returns an empty `LaunchDescription` and logs a TODO. Real composition is added per package as each one comes online.

## Planned launch files

| File | Eventual purpose |
|---|---|
| `launch/bringup.launch.py` | TODO: full robot bringup. Mission state machine, drive, arm controller, perception, localization, diagnostics. |
| `launch/sim.launch.py` | TODO: simulator-backed bringup with the same ROS interfaces as the hardware path. |
| `launch/teleop.launch.py` | TODO: operator-station only (Foxglove bridge, click-to-3D adapter, `/cmd_vel` input). No robot-side drivers. |

## Planned config

| File | Eventual purpose |
|---|---|
| `config/bringup_params.yaml` | TODO: per-node parameter overrides loaded by `bringup.launch.py`. Currently a placeholder header. |

## Building

```bash
colcon build --packages-select fortis_bringup
source install/setup.bash
```

## Running (stubs only today)

```bash
ros2 launch fortis_bringup bringup.launch.py
ros2 launch fortis_bringup sim.launch.py
ros2 launch fortis_bringup teleop.launch.py
```

Each currently logs `TODO: not implemented` at INFO level and exits the launch event loop with no actions to wait on.

## TODO

- Wire `bringup.launch.py` to the real `mission_state_node`, `drive_node`, and (when present) arm controller.
- Add MoveIt 2 launch include once `fortis_moveit_config` exists.
- Add `depthai_ros_driver` includes for the 5 OAK cameras once `fortis_perception` exists. Per the BOM that is 4x OAK-D Lite (Active Focus) A00483 (left + right toroidal VIO, rear outward depth, front angled-up depth) plus 1x OAK-D Pro (Active Stereo IR, Active Focus, Standard FOV) A00546 on the arm L4 midpoint.
- Add `robot_localization` ekf_node once `fortis_localization` exists.
- Add namespace + `use_sim_time` arguments to `sim.launch.py`.
- Add Foxglove bridge to `teleop.launch.py`.
- Populate `config/bringup_params.yaml` with per-node parameter blocks.
