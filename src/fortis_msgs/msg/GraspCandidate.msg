# A grasp pose returned by the grasp planner.
#
# pose         Target end-effector pose, expressed in the robot frame.
# confidence   Planner confidence in this candidate, in [0.0, 1.0]. Higher is
#              better. Consumers should treat 0.0 as "do not attempt".
# approach     Unit vector for the approach direction in the robot frame. The
#              arm should align its tool axis to this vector before closing
#              the last few centimetres to the pose.
# stamp        Time the candidate was generated.

geometry_msgs/Pose pose
float32 confidence
geometry_msgs/Vector3 approach
builtin_interfaces/Time stamp
