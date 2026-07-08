"""
Voxel mapping node (stub).

Planned behaviour: accumulate /fortis/perception/points_fused into a
persistent voxel map in the odom frame (grid math lives in
voxel_grid.py) and publish /fortis/perception/map/cloud
(sensor_msgs/PointCloud2). Until then this is a named placeholder so
launch plumbing and imports stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "voxel_map"


def main(args=None):
    """Entry point registered as the `voxel_map_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("voxel_map stub running -- no topics published yet.")
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
