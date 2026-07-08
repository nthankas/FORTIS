"""
Depth-to-point-cloud node (stub).

Planned behaviour: subscribe to one camera's registered depth image and
CameraInfo, back-project pixels through the pinhole model, and publish
/fortis/perception/<cam>/points (sensor_msgs/PointCloud2) in the
camera's optical frame. One instance runs per camera. Until then this
is a named placeholder so launch plumbing and imports stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "depth_to_cloud"


def main(args=None):
    """Entry point registered as the `depth_to_cloud_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("depth_to_cloud stub running -- no topics published yet.")
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
