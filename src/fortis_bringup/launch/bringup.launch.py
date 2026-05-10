"""
Top-level FORTIS bringup launch file (stub).

TODO: not implemented. Will eventually compose mission_state_node,
drive_node, arm controller, perception, localization, and diagnostics
into one launch entry point for the full robot.
"""
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.LogInfo(
            msg='fortis_bringup/bringup.launch.py: TODO: not implemented'
        ),
    ])
