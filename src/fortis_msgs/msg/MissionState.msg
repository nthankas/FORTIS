# Richer mission-state announcement. Complements the plain std_msgs/String
# state topic published by fortis_safety/mission_state_node by also carrying
# the previous state and a transition timestamp, which downstream consumers
# (UI, loggers, watchdogs) need to render transitions and detect stalls.
#
# state            Current state. One of:
#                    IDLE, ORBIT, TARGETING, ARM_AT_VIEW, INSPECT,
#                    PICK, HOLDING, RETURN_HOME, FAULT
# previous_state   Last state before this transition. Empty string on the
#                  first announcement after node startup (no prior state).
# stamp            Time the transition into `state` occurred.

string state
string previous_state
builtin_interfaces/Time stamp
