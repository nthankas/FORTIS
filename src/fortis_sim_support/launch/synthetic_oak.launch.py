"""
Synthetic OAK bring-up: one oak_replayer per requested chassis mount.

Publishes the depthai v3 topic contract under /oak_chassis_<mount>/ for
every mount named in `cameras`, plus /fortis/sim/ground_truth, so
downstream perception launch files consume either the real camera rig
or these replayers unchanged. Every instance broadcasts its own static
camera mount TF; only the FIRST owns the dynamic odom->base_link TF and
the ground-truth topic (publish_base_tf) -- all instances sample the
same base trajectory, one rigid robot. Defaults mirror the real capture
config (640x400 @ 15 fps) on the default 1 m orbit, front mount only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _replayers(context):
    """Build one oak_replayer node per mount named in `cameras`."""
    mounts = [
        m.strip()
        for m in LaunchConfiguration("cameras").perform(context).split(",")
        if m.strip()
    ]
    if not mounts:
        raise RuntimeError("cameras must name at least one mount")
    publish_tf = (
        LaunchConfiguration("publish_tf").perform(context).strip().lower()
        in ("true", "1"))
    return [
        Node(
            package="fortis_sim_support",
            executable="oak_replayer_node",
            name=f"oak_replayer_{mount}",
            output="screen",
            parameters=[{
                "mount": mount,
                "camera_name": f"oak_chassis_{mount}",
                "trajectory": LaunchConfiguration("trajectory").perform(context),
                "scene": LaunchConfiguration("scene").perform(context),
                "publish_tf": publish_tf,
                # Exactly one instance owns odom->base_link + ground truth.
                "publish_base_tf": index == 0,
                "fps": float(LaunchConfiguration("fps").perform(context)),
            }],
        )
        for index, mount in enumerate(mounts)
    ]


def generate_launch_description():
    """Declare the replayer arguments and defer node assembly to launch time."""
    args = [
        DeclareLaunchArgument(
            "cameras",
            default_value="front",
            description=(
                "Comma-separated chassis mounts to replay "
                "(front|rear|left|right); each becomes one oak_replayer "
                "publishing under /oak_chassis_<mount>/."
            ),
        ),
        DeclareLaunchArgument(
            "trajectory",
            default_value="orbit",
            description="Base trajectory shared by all replayers: orbit | line | hold.",
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="baseline",
            description="Scene variant: baseline | modified (one box added, one removed).",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description=(
                "Broadcast TF: every replayer sends its own static camera "
                "mount TF; the first additionally owns odom->base_link."
            ),
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="15.0",
            description="Render/publish rate; default matches the real capture config.",
        ),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_replayers)])
