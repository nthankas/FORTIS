# Health snapshot for one ODrive S1 axis. Composed into OdriveHealth.
#
# Fields are chosen to give the FORTIS safety FSM and operator console
# exactly what they need to reason about axis faults, without binding the
# safety layer to the upstream odrive_ros2_control message schema. A
# small bridge node translates upstream /odrive_status into this contract.
#
#   node_id              CAN node_id 0..3 (FL=0, FR=1, RR=2, RL=3 per the
#                        FORTIS chain order).
#   armed                True iff axis_state == CLOSED_LOOP_CONTROL. False
#                        in IDLE, MOTOR_CALIBRATION, any error state, etc.
#   active_errors        Bitfield from the ODrive firmware. 0 = no faults.
#                        Any non-zero value is a hard failure of this axis.
#   vbus_voltage         Bus voltage in volts at the time of the snapshot.
#   motor_temperature    Motor NTC reading in degrees Celsius (the M8325s
#                        integrated thermistor; -1.0 if disconnected or
#                        gpio_pin not yet discovered).
#   fet_temperature      S1 power-stage thermistor reading in degrees
#                        Celsius.

uint8 node_id
bool armed
uint32 active_errors
float32 vbus_voltage
float32 motor_temperature
float32 fet_temperature
