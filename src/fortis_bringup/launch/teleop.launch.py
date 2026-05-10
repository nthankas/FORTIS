"""
Teleop bringup launch file (stub).

TODO: not implemented. Will eventually launch operator-station nodes
(Foxglove bridge, click-to-3D adapter, /cmd_vel input) without bringing
up the robot-side hardware drivers.
"""
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.LogInfo(
            msg='fortis_bringup/teleop.launch.py: TODO: not implemented'
        ),
    ])
