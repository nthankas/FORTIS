# fortis_bringup

Top-level launch composition for FORTIS. Partial today: `bringup.launch.py` composes the mission state machine and the drive node; `sim.launch.py` and `teleop.launch.py` remain stubs. More node includes are added per package as each one comes online.

## Launch files

| File | Status | Purpose |
|---|---|---|
| `launch/bringup.launch.py` | live | Composes `mission_state_node` (`fortis_safety`) and `drive_node` (`fortis_drive`). Arm controller, perception, localization, and diagnostics are added as those packages come online. |
| `launch/sim.launch.py` | stub | TODO: simulator-backed bringup with the same ROS interfaces as the hardware path. Currently logs `TODO: not implemented`. |
| `launch/teleop.launch.py` | stub | TODO: operator-station only (Foxglove bridge, click-to-3D adapter, `/cmd_vel` input). Currently logs `TODO: not implemented`. |

## Planned config

| File | Eventual purpose |
|---|---|
| `config/bringup_params.yaml` | TODO: per-node parameter overrides loaded by `bringup.launch.py`. Currently a placeholder header. |

## Building

```bash
colcon build --packages-select fortis_bringup
source install/setup.bash
```

## Running

```bash
ros2 launch fortis_bringup bringup.launch.py    # live: mission_state_node + drive_node
ros2 launch fortis_bringup sim.launch.py        # stub: logs TODO and exits
ros2 launch fortis_bringup teleop.launch.py     # stub: logs TODO and exits
```

The two stub launch files log `TODO: not implemented` at INFO level and exit the launch event loop with no actions to wait on.

## TODO

- Add the arm controller include (`fortis_arm/arm_controller_node`) to `bringup.launch.py`.
- Add MoveIt 2 launch include once `fortis_moveit_config` exists.
- Add `depthai_ros_driver` includes for the 5 OAK cameras once `fortis_perception` exists. Per the BOM that is 4x OAK-D Lite (Active Focus) A00483 (left + right toroidal VIO, rear outward depth, front angled-up depth) plus 1x OAK-D Pro (Active Stereo IR, Active Focus, Standard FOV) A00546 on the arm L4 midpoint.
- Add `robot_localization` ekf_node once `fortis_localization` exists.
- Add namespace + `use_sim_time` arguments to `sim.launch.py`.
- Add Foxglove bridge to `teleop.launch.py`.
- Populate `config/bringup_params.yaml` with per-node parameter blocks.
