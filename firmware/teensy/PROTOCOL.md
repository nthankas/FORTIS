# Teensy 4.1 Motion Controller Protocol

This document specifies the binary serial protocol between a host (Jetson or PC)
and the Teensy 4.1 firmware that drives three NEMA-23 closed-loop steppers
(J1..J3, via CL57T-V41 drivers behind an SN74HCT245N level shifter), one
high-power waterproof servo (J4, Hitec D845WP), and one hobby-grade gripper
servo. The protocol is self-contained: it does not depend on any URDF, ROS,
or higher-level controller stack.

- Link: USB CDC serial, **1 000 000 baud** (`Serial.begin(1000000)`)
- Byte order: **little-endian** for all multi-byte fields
- Logic levels: Teensy is 3.3 V; the SN74HCT245N (HCT family) accepts 3.3 V
  TTL high on its A-side inputs and drives 5 V CMOS-clean on its B-side
  outputs into the CL57T-V41 STEP/DIR/ENA optocouplers
- Frame timing: a complete frame must be received within 50 ms once the start
  byte is seen, otherwise the parser resyncs

---

## 1. Pin assignments (informative)

These are the pins chosen by `firmware/teensy/main.ino`. They are repeated
here so a host implementer or hardware integrator can wire the board without
opening the `.ino` file. Any change to `main.ino` MUST be reflected here.

| Function                       | Teensy pin | Direction (Teensy view) | Notes                                     |
|--------------------------------|-----------:|-------------------------|-------------------------------------------|
| J1 STEP                        |  2         | OUT (3V3 -> A1 of '245) | A-side, level-shifted to 5V on B1         |
| J1 DIR                         |  3         | OUT (3V3 -> A2 of '245) |                                           |
| J2 STEP                        |  4         | OUT (3V3 -> A3 of '245) |                                           |
| J2 DIR                         |  5         | OUT (3V3 -> A4 of '245) |                                           |
| J3 STEP                        |  6         | OUT (3V3 -> A5 of '245) |                                           |
| J3 DIR                         |  7         | OUT (3V3 -> A6 of '245) |                                           |
| ENABLE (shared, active-low)    |  9         | OUT (3V3 -> A7 of '245) | Drives ENA- on all three CL57T-V41        |
| J4 servo PWM                   | 28         | OUT (FlexPWM3.1.B)      | 50 Hz, 500..2500 us pulse                 |
| Gripper servo PWM              | 29         | OUT (FlexPWM3.1.A)      | 50 Hz, 1000..2000 us pulse                |
| SN74HCT245N /OE (active-low)   | 14         | OUT                     | Pulled high (disabled) at boot            |
| SN74HCT245N DIR                | 15         | OUT                     | Tied high in firmware (A->B, fixed)       |
| J1 ALM (CL57T-V41 ALM+)        | 22         | IN (via second '245)    | See "Driver alarm wiring" below           |
| J2 ALM                         | 23         | IN (via second '245)    |                                           |
| J3 ALM                         | 20         | IN (via second '245)    |                                           |
| External E-STOP                | 21         | IN, INPUT_PULLUP        | Active-low: low = E-STOP asserted         |
| Status LED                     | 13         | OUT                     | Built-in LED, blinks on heartbeat         |

### Driver alarm wiring

The CL57T-V41 ALM output is an open-collector signal referenced to its
internal optocoupler. Wire ALM+ to a 5 V pull-up on the B-side of a second
SN74HCT245N (configured B->A, 5 V -> 3.3 V; the HCT '245 tolerates this
because the A-side is just an input on the Teensy). The firmware reads
the alarm with internal pull-up and treats **logic low for >5 ms** as an
active alarm (CL57T-V41 ALM is active-low when a fault is latched).

The same '245 used for STEP/DIR is **not** suitable for the return path —
use a second '245 with DIR tied low, or a dedicated bidirectional shifter.
This protocol document is agnostic to which level shifter is used as long as
the Teensy sees clean 3.3 V logic on the ALM pins.

---

## 2. Frame format

All frames, in both directions, share this structure:

```
+--------+--------+--------+--------+----------------+----------+----------+--------+
| START  | LEN    | SEQ    | TYPE   | PAYLOAD (LEN)  | CRC_LO   | CRC_HI   | END    |
| 0xA5   | 1 byte | 1 byte | 1 byte | LEN bytes      | 1 byte   | 1 byte   | 0x5A   |
+--------+--------+--------+--------+----------------+----------+----------+--------+
```

| Field    | Size  | Description                                                              |
|----------|------:|--------------------------------------------------------------------------|
| START    | 1     | Always `0xA5`. Used for resync.                                          |
| LEN      | 1     | Number of payload bytes (0..240). Excludes header and CRC.               |
| SEQ      | 1     | 8-bit sequence number, wraps at 0xFF. Host increments per command.       |
| TYPE     | 1     | Message type ID (see section 3).                                         |
| PAYLOAD  | LEN   | Type-specific payload, little-endian.                                    |
| CRC_LO   | 1     | Low byte of CRC-16-CCITT computed over `[SEQ, TYPE, PAYLOAD]`.           |
| CRC_HI   | 1     | High byte of CRC-16-CCITT.                                               |
| END      | 1     | Always `0x5A`. Optional integrity check; firmware uses it for resync.    |

**Maximum frame size:** `8 + 240 = 248 bytes`.
**No escaping is used** — the combination of length-prefix, CRC-16-CCITT
across (SEQ, TYPE, PAYLOAD), and a unique END byte is sufficient. If a frame
fails CRC, the firmware emits `RSP_NAK` with `ERR_CRC` and resyncs by
discarding bytes until the next `0xA5`.

### CRC-16-CCITT details

- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- No reflection of input or output
- No final XOR
- Computed over `[SEQ, TYPE, PAYLOAD[0..LEN-1]]` in that order

Reference implementation:

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}
```

---

## 3. Message types

Type IDs are split into ranges by direction:

| Range       | Direction       | Purpose            |
|-------------|-----------------|--------------------|
| `0x01..0x3F` | host -> teensy | Commands (`CMD_*`) |
| `0x80..0xBF` | teensy -> host | Responses (`RSP_*`)|
| `0xC0..0xDF` | teensy -> host | Async events (`EVT_*`) |

| Name                       | ID    | Direction      | Payload size |
|----------------------------|-------|----------------|--------------|
| `CMD_HEARTBEAT`            | 0x01  | host -> teensy | 0            |
| `CMD_GET_STATUS`           | 0x02  | host -> teensy | 0            |
| `CMD_ENABLE`               | 0x03  | host -> teensy | 0            |
| `CMD_DISABLE`              | 0x04  | host -> teensy | 0            |
| `CMD_SET_JOINT_TARGETS`    | 0x10  | host -> teensy | 18           |
| `CMD_SET_JOINT_VELOCITIES` | 0x11  | host -> teensy | 12           |
| `CMD_HOME_REQUEST`         | 0x12  | host -> teensy | 1            |
| `CMD_CLEAR_FAULTS`         | 0x13  | host -> teensy | 2            |
| `RSP_ACK`                  | 0x80  | teensy -> host | 2            |
| `RSP_NAK`                  | 0x81  | teensy -> host | 3            |
| `RSP_STATUS`               | 0x82  | teensy -> host | 28           |
| `EVT_FAULT`                | 0xC0  | teensy -> host | 4            |
| `EVT_BOOT`                 | 0xC1  | teensy -> host | 8            |

Every `CMD_*` is acknowledged with either `RSP_ACK` (echoes the command's
SEQ and TYPE) or `RSP_NAK` (echoes SEQ, TYPE, and an error code). The
exception is `CMD_GET_STATUS`, which is answered with `RSP_STATUS` (no
separate ACK).

### 3.1 `CMD_HEARTBEAT` (0x01)

Empty payload. Must be sent by the host at least every **100 ms**. If the
firmware does not receive any frame addressed to it (any valid CMD) for
more than **250 ms**, it sets `FAULT_HEARTBEAT_TIMEOUT`, drives ENABLE
inactive, and stops all motion. Cleared by `CMD_CLEAR_FAULTS`.

### 3.2 `CMD_GET_STATUS` (0x02)

Empty payload. Firmware replies with a single `RSP_STATUS` frame.

### 3.3 `CMD_ENABLE` (0x03) / `CMD_DISABLE` (0x04)

Empty payload. Asserts or releases the shared ENABLE line driving all three
CL57T-V41 drivers. While disabled, `CMD_SET_JOINT_TARGETS` and
`CMD_SET_JOINT_VELOCITIES` are NAK'd with `ERR_DISABLED`.

### 3.4 `CMD_SET_JOINT_TARGETS` (0x10)

Absolute target positions. J1..J3 are in encoder/driver-step counts (signed,
zero == "homed"). J4 and gripper are servo pulse widths in microseconds.

| Offset | Size | Field          | Description                          |
|-------:|-----:|----------------|--------------------------------------|
| 0      | 4    | `j1_steps`     | int32, target position in steps       |
| 4      | 4    | `j2_steps`     | int32, target position in steps       |
| 8      | 4    | `j3_steps`     | int32, target position in steps       |
| 12     | 2    | `j4_us`        | uint16, 500..2500                     |
| 14     | 2    | `gripper_us`   | uint16, 1000..2000                    |
| 16     | 2    | `flags`        | bit0=clamp_to_limits, others reserved |

### 3.5 `CMD_SET_JOINT_VELOCITIES` (0x11)

Velocity limits used until the next `CMD_SET_JOINT_TARGETS`. Each value is
the *signed* maximum velocity for the upcoming move; sign defines direction
when used in pure-velocity mode.

| Offset | Size | Field         | Description                    |
|-------:|-----:|---------------|--------------------------------|
| 0      | 4    | `j1_sps`      | int32, steps/sec               |
| 4      | 4    | `j2_sps`      | int32, steps/sec               |
| 8      | 4    | `j3_sps`     | int32, steps/sec               |

### 3.6 `CMD_HOME_REQUEST` (0x12)

| Offset | Size | Field    | Description                               |
|-------:|-----:|----------|-------------------------------------------|
| 0      | 1    | `mask`   | bit0=J1, bit1=J2, bit2=J3, 0x07 = all     |

Homing implementation is firmware-side and is NOT specified by this
protocol; the firmware NAKs with `ERR_NOT_IMPLEMENTED` until homing is
implemented.

### 3.7 `CMD_CLEAR_FAULTS` (0x13)

| Offset | Size | Field    | Description                                                    |
|-------:|-----:|----------|----------------------------------------------------------------|
| 0      | 2    | `mask`   | uint16; bits set here are cleared from the latched fault word  |

Sticky faults that physically persist (e.g. an asserted E-STOP, a still-active
ALM line) will re-latch immediately on the next 1 kHz fault scan.

### 3.8 `RSP_ACK` (0x80)

| Offset | Size | Field         | Description                  |
|-------:|-----:|---------------|------------------------------|
| 0      | 1    | `acked_seq`   | SEQ from the command being acknowledged |
| 1      | 1    | `acked_type`  | TYPE from the command        |

### 3.9 `RSP_NAK` (0x81)

| Offset | Size | Field         | Description                       |
|-------:|-----:|---------------|-----------------------------------|
| 0      | 1    | `nak_seq`     | SEQ from the rejected command     |
| 1      | 1    | `nak_type`    | TYPE from the rejected command    |
| 2      | 1    | `err_code`    | See section 5                     |

### 3.10 `RSP_STATUS` (0x82)

| Offset | Size | Field         | Description                                  |
|-------:|-----:|---------------|----------------------------------------------|
| 0      | 4    | `j1_steps`    | int32, current commanded step position       |
| 4      | 4    | `j2_steps`    | int32                                        |
| 8      | 4    | `j3_steps`    | int32                                        |
| 12     | 2    | `j4_us`       | uint16, last commanded servo pulse           |
| 14     | 2    | `gripper_us`  | uint16                                       |
| 16     | 2    | `fault_flags` | uint16, see section 4                        |
| 18     | 2    | `state`       | uint16, bit0=enabled, bit1=moving, bit2=homed_j1, bit3=homed_j2, bit4=homed_j3 |
| 20     | 4    | `uptime_ms`   | uint32, ms since boot                        |
| 24     | 2    | `mcu_temp_c10`| int16, MCU temperature in 0.1 C units        |
| 26     | 2    | `reserved`    | zero                                         |

### 3.11 `EVT_FAULT` (0xC0)

Sent asynchronously when *any* fault bit transitions from 0 to 1.

| Offset | Size | Field          | Description                      |
|-------:|-----:|----------------|----------------------------------|
| 0      | 2    | `new_bits`     | uint16, bits that just rose      |
| 2      | 2    | `current_flags`| uint16, current latched word     |

### 3.12 `EVT_BOOT` (0xC1)

Sent once after `setup()` completes.

| Offset | Size | Field           | Description                            |
|-------:|-----:|-----------------|----------------------------------------|
| 0      | 1    | `proto_major`   | currently 1                            |
| 1      | 1    | `proto_minor`   | currently 0                            |
| 2      | 1    | `eeprom_slot`   | 0 or 1, slot used for recovery, 0xFF if both invalid |
| 3      | 1    | `recovered`     | 0 = uncalibrated, 1 = positions recovered |
| 4      | 4    | `git_short`     | uint32, optional firmware build id (0 if unused) |

---

## 4. Fault flags (uint16 bitmask)

| Bit | Name                       | Meaning                                                      |
|----:|----------------------------|--------------------------------------------------------------|
|  0  | `FAULT_HEARTBEAT_TIMEOUT`  | No host CMD received in >250 ms                              |
|  1  | `FAULT_DRIVER_ALARM_J1`    | CL57T-V41 ALM on J1 active >5 ms                             |
|  2  | `FAULT_DRIVER_ALARM_J2`    | CL57T-V41 ALM on J2                                          |
|  3  | `FAULT_DRIVER_ALARM_J3`    | CL57T-V41 ALM on J3                                          |
|  4  | `FAULT_EEPROM_CRC`         | Both EEPROM slots failed CRC at boot                         |
|  5  | `FAULT_OVER_TEMP`          | MCU internal temp > 85 C                                     |
|  6  | `FAULT_UNCALIBRATED`       | At least one stepper joint has not been homed                |
|  7  | `FAULT_E_STOP`             | External E-STOP input asserted                               |
|  8  | `FAULT_FRAME_OVERFLOW`     | Serial RX buffer overflow (informational)                    |
|  9  | `FAULT_BAD_PARAMETER`      | Out-of-range value seen in a recent command                  |
| 10..15 | reserved                | must be zero                                                 |

### Latching behavior

All faults are **latched**: once set, a bit remains set until the host clears
it with `CMD_CLEAR_FAULTS`. A latched fault that corresponds to a still-active
physical condition (E-STOP held down, ALM still asserted, MCU still hot) will
immediately re-latch on the next 1 kHz fault scan, and the firmware will emit
another `EVT_FAULT`. This means the host sees a fresh `EVT_FAULT` only on
rising edges, but a `CMD_GET_STATUS` always reflects the current latched
word.

While **any** fault is latched, motion is inhibited and `CMD_ENABLE` is
NAK'd with `ERR_FAULT_ACTIVE`.

---

## 5. Error codes (`RSP_NAK.err_code`)

| Code | Name                  | Meaning                                          |
|-----:|-----------------------|--------------------------------------------------|
| 0x00 | `ERR_NONE`            | reserved (not used in NAK)                       |
| 0x01 | `ERR_CRC`             | CRC mismatch on inbound frame                    |
| 0x02 | `ERR_LENGTH`          | Payload length wrong for this TYPE               |
| 0x03 | `ERR_UNKNOWN_TYPE`    | TYPE not recognized                              |
| 0x04 | `ERR_BAD_PARAMETER`   | Field out of allowed range                       |
| 0x05 | `ERR_DISABLED`        | Motion command received while disabled           |
| 0x06 | `ERR_FAULT_ACTIVE`    | Cannot enable / move while any fault is latched  |
| 0x07 | `ERR_NOT_HOMED`       | Motion command for an unhomed joint              |
| 0x08 | `ERR_NOT_IMPLEMENTED` | Feature stub, e.g. homing                        |
| 0x09 | `ERR_BUSY`            | Previous motion still executing                  |

---

## 6. EEPROM checkpointing scheme

The Teensy 4.1's emulated EEPROM (4284 bytes) is used to remember the last
commanded joint positions across power cycles. Two fixed-size slots are kept
and written ping-pong to survive a power loss mid-write.

### 6.1 Slot layout

```c
struct EepromSlot {
    uint32_t magic;        // 'FRTS' = 0x53545246 little-endian
    uint16_t version;      // currently 1
    uint16_t reserved;     // zero
    int32_t  j1_steps;
    int32_t  j2_steps;
    int32_t  j3_steps;
    uint16_t j4_us;
    uint16_t gripper_us;
    uint32_t write_count;  // monotonic, wraps at 2^32
    uint16_t crc;          // CRC-16-CCITT over all preceding bytes
    uint16_t reserved2;    // zero, pads to 32 bytes
};                         // sizeof == 32
```

Two slots are stored at fixed EEPROM offsets:

| Slot | EEPROM offset | Size |
|-----:|--------------:|-----:|
| 0    | 0             | 32   |
| 1    | 32            | 32   |

The remaining EEPROM space is reserved for future use (calibration tables,
serial number, etc.).

### 6.2 Write algorithm

```
N = 50            // every Nth target update
M = 500 ms        // or every M ms, whichever comes first
```

On each `CMD_SET_JOINT_TARGETS` (and once per `M` ms in `loop()` if there
have been any updates since the last write):

1. Read both slots and pick `valid_slot` = the one with valid CRC and higher
   `write_count` (modular comparison; if neither is valid, treat as 0).
2. Write to the **other** slot with `write_count = valid_slot.write_count + 1`.
3. Recompute CRC and commit.

### 6.3 Read / recovery on boot

1. Read slot 0, validate magic, version, CRC.
2. Read slot 1, validate magic, version, CRC.
3. If both valid: pick the one with larger `write_count` (modular).
4. If only one valid: pick it.
5. If neither valid: set `FAULT_UNCALIBRATED`, set `FAULT_EEPROM_CRC`, all
   joint positions default to 0, motion is inhibited until `CMD_HOME_REQUEST`
   succeeds for each axis.

The firmware reports which slot was used in `EVT_BOOT.eeprom_slot` (0, 1, or
0xFF for "neither valid").

---

## 7. Watchdog and heartbeat timing

| Parameter                       | Value     |
|---------------------------------|-----------|
| Host heartbeat period (target)  | <= 100 ms |
| Firmware heartbeat timeout      | 250 ms    |
| Fault scan period               | 1 ms      |
| EEPROM checkpoint period (max)  | 500 ms    |
| EEPROM checkpoint per-N updates | 50        |
| ALM debounce                    | 5 ms      |
| Status reply latency (typical)  | < 2 ms    |

When the heartbeat times out:

1. `FAULT_HEARTBEAT_TIMEOUT` is set.
2. ENABLE line is driven inactive (motors freewheel; CL57T-V41 holds last
   position open-loop until the alarm timeout).
3. Any in-flight TeensyStep motion is stopped via `StepControl::stopAsync()`.
4. An `EVT_FAULT` is emitted.

Recovery: the host clears the fault with `CMD_CLEAR_FAULTS`, resumes
heartbeats, then issues `CMD_ENABLE`.

---

## 8. Versioning

The protocol major/minor is reported in `EVT_BOOT`. Hosts MUST verify the
major version matches what they were built against; minor version increases
add fields/messages backwards-compatibly. Major bumps are breaking.

Current version: **1.0**.
