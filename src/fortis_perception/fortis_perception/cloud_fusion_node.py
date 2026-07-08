"""
Multi-camera cloud fusion node (stub).

Planned behaviour: subscribe to every per-camera
/fortis/perception/<cam>/points cloud, TF-transform each into
base_link, and publish the merged /fortis/perception/points_fused
(sensor_msgs/PointCloud2). Until then this is a named placeholder so
launch plumbing and imports stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "cloud_fusion"


def main(args=None):
    """Entry point registered as the `cloud_fusion_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("cloud_fusion stub running -- no topics published yet.")
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
