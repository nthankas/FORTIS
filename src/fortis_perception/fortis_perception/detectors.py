"""
Object detection backends (stub).

Will wrap the detection model(s) behind a common interface (load, infer
on RGB frames, return 2D boxes + classes) so detection_node can swap
backends without changing its ROS plumbing. Kept free of ROS imports so
it is unit-testable in isolation.
"""
