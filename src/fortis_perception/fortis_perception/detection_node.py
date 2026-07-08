"""
Object detection node (stub).

Planned behaviour: run the detectors backends on camera frames and
publish /fortis/perception/detections (vision_msgs/Detection2DArray),
/fortis/perception/detections3d (Detection3DArray),
/fortis/perception/detection_markers (MarkerArray), per-camera
/fortis/perception/annotations/<cam> (foxglove_msgs/ImageAnnotations),
and /fortis/perception/grasp_candidate (fortis_msgs/GraspCandidate).
Model weights are fetched by the download_models console script. Until
then this is a named placeholder so launch plumbing and imports stay
green.
"""

import rclpy
from rclpy.node import Node

#: Node name the placeholder registers under.
NODE_NAME = "detection"


def main(args=None):
    """Entry point registered as the `detection_node` console script."""
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info("detection stub running -- no topics published yet.")
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
