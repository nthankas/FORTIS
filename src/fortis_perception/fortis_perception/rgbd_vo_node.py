"""
RGBD visual odometry node (stub).

Planned behaviour: run the rgbd_vo core (frame-to-frame feature
tracking + depth-informed pose solve) on synchronised RGB and depth
frames and publish /fortis/vo (nav_msgs/Odometry). Until then this is a
named placeholder so launch plumbing and imports stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "rgbd_vo"


def main(args=None):
    """Entry point registered as the `rgbd_vo_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("rgbd_vo stub running -- no topics published yet.")
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
