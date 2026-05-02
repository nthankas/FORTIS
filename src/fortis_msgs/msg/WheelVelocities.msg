# Per-wheel angular velocity command for the X-drive.
#
# Wheel naming follows the chassis convention: looking down on the robot with
# +X (forward) pointing radially toward the reactor centre post. Wheels sit at
# the four 45-deg chamfered corners.
#
#   FL = front-left   FR = front-right
#   BL = back-left    BR = back-right
#
# Units are rad/s at the wheel shaft (i.e. after any gearbox), positive in the
# rolling-forward direction for that wheel as wired.
#
# stamp   Time the command was computed.

float32 fl
float32 fr
float32 bl
float32 br
builtin_interfaces/Time stamp
