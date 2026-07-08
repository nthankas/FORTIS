"""
OAK camera replayer (stub).

Planned behaviour: replay recorded (or synthetic_scene-rendered) RGBD
frames, CameraInfo, and IMU samples under a configurable camera
namespace, byte-compatible with the topics the real depthai v3 driver
publishes, plus ground truth on /fortis/sim/ground_truth
(nav_msgs/Odometry). Until then this is a named placeholder node so the
package installs, launches, and imports cleanly.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under; the launch file and tests
#: key on it.
NODE_NAME = "oak_replayer"


def main(args=None):
    """Entry point registered as the `oak_replayer_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("oak_replayer stub running -- no topics published yet.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
