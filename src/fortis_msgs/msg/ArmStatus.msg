# Decoded Teensy 4.1 arm-controller status (RSP_STATUS, firmware/teensy/PROTOCOL.md 3.10),
# republished by the teensy bridge with a connectivity flag.
#
# connected      True while the serial link is up and STATUS frames are fresh.
#                When False every other field holds its last-known value.
# joint_steps    J1..J3 stepper positions in raw steps (int32, signed).
# j4_us          Last commanded J4 servo pulse width, microseconds.
# gripper_us     Last commanded gripper servo pulse width, microseconds.
# fault_flags    Fault bitmask, PROTOCOL.md section 4 (bit0 heartbeat timeout,
#                bits1-3 driver alarms J1-J3, bit4 EEPROM CRC, bit5 over-temp,
#                bit6 uncalibrated, bit7 e-stop, bit8 frame overflow).
# state_bits     bit0 enabled, bit1 moving, bit2..4 homed J1..J3.
# uptime_ms      Firmware milliseconds since boot.
# mcu_temp_c     MCU internal temperature, degrees C (decoded from 0.1 C units).
# stamp          Host receive time of the STATUS frame.

builtin_interfaces/Time stamp
bool connected
int32[3] joint_steps
uint16 j4_us
uint16 gripper_us
uint16 fault_flags
uint16 state_bits
uint32 uptime_ms
float32 mcu_temp_c
