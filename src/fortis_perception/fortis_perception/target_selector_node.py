"""
Target selector node (stub).

Planned behaviour: rank live detections into a single active target and
publish /fortis/target_pose (geometry_msgs/PoseStamped) plus the
/fortis/context/{target_pose_valid, grasp_candidate_ok, ik_ok}
std_msgs/Bool flags shared with the detection node. Until then this is
a named placeholder so launch plumbing and imports stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "target_selector"


def main(args=None):
    """Entry point registered as the `target_selector_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("target_selector stub running -- no topics published yet.")
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
