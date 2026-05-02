# Operator click on a chassis camera image, used to designate a target.
#
# pixel       Image-space click coordinates. x, y are pixel offsets from the
#             top-left of the source image; z is unused (set to 0.0).
# camera_id   Identifier for the OAK-D that produced the image, e.g.
#             "oak_chassis_front". Used by the targeting pipeline to look up
#             the camera's intrinsics and frame transform.
# stamp       Time the click was registered at the operator station.

geometry_msgs/Point pixel
string camera_id
builtin_interfaces/Time stamp
