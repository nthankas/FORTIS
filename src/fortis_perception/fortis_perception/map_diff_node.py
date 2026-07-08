"""
Cross-run map diff node (stub).

Planned behaviour: compare the live voxel map against a saved reference
run and publish /fortis/perception/map_diff/markers
(visualization_msgs/MarkerArray) plus
/fortis/perception/map_diff/summary (fortis_msgs/MapDiffSummary).
Until then this is a named placeholder so launch plumbing and imports
stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "map_diff"


def main(args=None):
    """Entry point registered as the `map_diff_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("map_diff stub running -- no topics published yet.")
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
