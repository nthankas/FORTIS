"""
System health node (stub).

Planned behaviour: watch the perception stack's topic liveness and
latency and publish aggregated /diagnostics
(diagnostic_msgs/DiagnosticArray), alongside the teensy_bridge's
hardware diagnostics. Until then this is a named placeholder so launch
plumbing and imports stay green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "system_health"


def main(args=None):
    """Entry point registered as the `system_health_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("system_health stub running -- no topics published yet.")
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
