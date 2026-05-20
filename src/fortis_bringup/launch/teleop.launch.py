"""
Developer / debug teleop bringup.

Launches teleop_twist_keyboard and remaps its output to /cmd_vel so the
drive node receives keyboard-driven Twist messages. This is for
developer iteration only: the production operator UI is the Foxglove
+ click-to-3D path, not a keyboard REPL. Use this when bringing up the
drive stack on the bench or in sim, before the UI is up.

Limitations
-----------
teleop_twist_keyboard reads stdin directly from the terminal. The
launch system attaches the node's stdin to the launch process, which
works only when this file is launched in the foreground from an
interactive terminal. Backgrounded launches (``&``), nohup, daemonised
runs, and IDE "run" buttons that pipe stdin produce a node that
publishes nothing because it cannot read keys. If keyboard input is
not landing on /cmd_vel, check that ros2 launch is the foreground
process attached to a tty.

Pairs with
----------
Run fortis_safety/mission_state_node and fortis_drive/drive_node in
other terminals first (or use bringup.launch.py); this launch file
does not bring up anything except the teleop node. The drive node
gates /cmd_vel by mission state; START_ORBIT before sending keys, or
nothing will move.
"""

import launch
import launch_ros.actions


CMD_VEL_TOPIC = "/cmd_vel"


def generate_launch_description():
    teleop_node = launch_ros.actions.Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        # teleop_twist_keyboard publishes to /cmd_vel by default in
        # ROS 2 Humble; the remap is a no-op today but keeps the
        # contract explicit so a future package rename or default
        # change does not silently break the wiring.
        remappings=[("/cmd_vel", CMD_VEL_TOPIC)],
        # output="screen" + emulate_tty so the keyboard help banner
        # the package prints on startup is visible. prefix=xterm
        # would also work but is heavier and requires X.
        output="screen",
        emulate_tty=True,
    )
    return launch.LaunchDescription([
        teleop_node,
    ])
