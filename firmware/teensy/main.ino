// SPDX-License-Identifier: MIT
//
// FORTIS Teensy 4.1 motion-controller firmware skeleton.
//
// Drives:
//   - J1, J2, J3: NEMA-23 closed-loop steppers via CL57T-V41 drivers
//                 (3.3V Teensy outputs are level-shifted to 5V by SN74HCT245N)
//   - J4:        Hitec D845WP servo on hardware PWM
//   - Gripper:   generic hobby servo on hardware PWM
//
// Host link: USB serial @ 1 000 000 baud. Framing/protocol per PROTOCOL.md.
//
// This is a SKELETON. Search for "TODO" for the bits left as stubs.

#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <TeensyStep.h>

// ---------------------------------------------------------------------------
// Build-time flags
// ---------------------------------------------------------------------------

#define MOCK_MODE           0   // 1 = no GPIO writes, all motor I/O -> Serial
#define FIRMWARE_BUILD_ID   0u  // optional: set from build system (git short)
#define PROTO_MAJOR         1
#define PROTO_MINOR         0

// ---------------------------------------------------------------------------
// Pin map (must match PROTOCOL.md section 1)
// ---------------------------------------------------------------------------

// Stepper STEP/DIR (3V3 outputs, level-shifted to 5V via SN74HCT245N #1)
#define PIN_J1_STEP         2
#define PIN_J1_DIR          3
#define PIN_J2_STEP         4
#define PIN_J2_DIR          5
#define PIN_J3_STEP         6
#define PIN_J3_DIR          7

// Shared driver enable (active-low at the CL57T-V41 ENA- input)
#define PIN_DRV_ENABLE      9

// Servo PWM (FlexPWM-capable pins)
#define PIN_J4_SERVO        28
#define PIN_GRIPPER_SERVO   29

// SN74HCT245N #1 control (forward path, A->B, 3V3->5V)
#define PIN_HCT245_OE_N     14   // active-low; high = outputs Hi-Z
#define PIN_HCT245_DIR      15   // tied high in firmware = A->B

// Driver alarm inputs (5V open-collector; arrives via SN74HCT245N #2 B->A)
#define PIN_J1_ALM          22
#define PIN_J2_ALM          23
#define PIN_J3_ALM          20

// External E-STOP loop, active-low, internal pull-up
#define PIN_ESTOP           21

// Status LED
#define PIN_LED             LED_BUILTIN

// ---------------------------------------------------------------------------
// Stepper / servo configuration
// ---------------------------------------------------------------------------

struct StepperCfg {
    int32_t  full_steps_per_rev;
    int32_t  microstepping;        // CL57T-V41 DIP setting, e.g. 8
    int32_t  max_speed_sps;        // steps per second
    int32_t  max_accel_sps2;       // steps per second^2
};

static const StepperCfg kJ1Cfg = { 200, 8, 8000, 20000 };
static const StepperCfg kJ2Cfg = { 200, 8, 8000, 20000 };
static const StepperCfg kJ3Cfg = { 200, 8, 8000, 20000 };

static const uint16_t kJ4MinUs      = 500;
static const uint16_t kJ4MaxUs      = 2500;
static const uint16_t kJ4DefaultUs  = 1500;

static const uint16_t kGripperMinUs     = 1000;
static const uint16_t kGripperMaxUs     = 2000;
static const uint16_t kGripperDefaultUs = 1500;

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

static const uint8_t  kFrameStart       = 0xA5;
static const uint8_t  kFrameEnd         = 0x5A;
static const uint8_t  kMaxPayload       = 240;
static const uint32_t kHeartbeatTimeoutMs = 250;
static const uint32_t kFaultScanPeriodMs  = 1;
static const uint32_t kEepromCheckpointMaxMs = 500;
static const uint32_t kEepromCheckpointEveryN = 50;
static const uint32_t kAlmDebounceMs    = 5;
static const int16_t  kOverTempC10      = 850;     // 85.0 C

// Message types
enum : uint8_t {
    CMD_HEARTBEAT             = 0x01,
    CMD_GET_STATUS            = 0x02,
    CMD_ENABLE                = 0x03,
    CMD_DISABLE               = 0x04,
    CMD_SET_JOINT_TARGETS     = 0x10,
    CMD_SET_JOINT_VELOCITIES  = 0x11,
    CMD_HOME_REQUEST          = 0x12,
    CMD_CLEAR_FAULTS          = 0x13,

    RSP_ACK                   = 0x80,
    RSP_NAK                   = 0x81,
    RSP_STATUS                = 0x82,

    EVT_FAULT                 = 0xC0,
    EVT_BOOT                  = 0xC1,
};

// Error codes
enum : uint8_t {
    ERR_NONE             = 0x00,
    ERR_CRC              = 0x01,
    ERR_LENGTH           = 0x02,
    ERR_UNKNOWN_TYPE     = 0x03,
    ERR_BAD_PARAMETER    = 0x04,
    ERR_DISABLED         = 0x05,
    ERR_FAULT_ACTIVE     = 0x06,
    ERR_NOT_HOMED        = 0x07,
    ERR_NOT_IMPLEMENTED  = 0x08,
    ERR_BUSY             = 0x09,
};

// Fault flags
enum : uint16_t {
    FAULT_HEARTBEAT_TIMEOUT  = 1u << 0,
    FAULT_DRIVER_ALARM_J1    = 1u << 1,
    FAULT_DRIVER_ALARM_J2    = 1u << 2,
    FAULT_DRIVER_ALARM_J3    = 1u << 3,
    FAULT_EEPROM_CRC         = 1u << 4,
    FAULT_OVER_TEMP          = 1u << 5,
    FAULT_UNCALIBRATED       = 1u << 6,
    FAULT_E_STOP             = 1u << 7,
    FAULT_FRAME_OVERFLOW     = 1u << 8,
    FAULT_BAD_PARAMETER      = 1u << 9,
};

// State bits
enum : uint16_t {
    STATE_ENABLED   = 1u << 0,
    STATE_MOVING    = 1u << 1,
    STATE_HOMED_J1  = 1u << 2,
    STATE_HOMED_J2  = 1u << 3,
    STATE_HOMED_J3  = 1u << 4,
};

// ---------------------------------------------------------------------------
// EEPROM slot layout (must match PROTOCOL.md section 6)
// ---------------------------------------------------------------------------

static const uint32_t kEepromMagic = 0x53545246u;  // 'FRTS' little-endian
static const uint16_t kEepromVersion = 1;
static const int      kEepromSlot0Off = 0;
static const int      kEepromSlot1Off = 32;

#pragma pack(push, 1)
struct EepromSlot {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    int32_t  j1_steps;
    int32_t  j2_steps;
    int32_t  j3_steps;
    uint16_t j4_us;
    uint16_t gripper_us;
    uint32_t write_count;
    uint16_t crc;
    uint16_t reserved2;
};
#pragma pack(pop)
static_assert(sizeof(EepromSlot) == 32, "EepromSlot must be 32 bytes");

// ---------------------------------------------------------------------------
// TeensyStep + Servo objects
// ---------------------------------------------------------------------------

static Stepper g_j1(PIN_J1_STEP, PIN_J1_DIR);
static Stepper g_j2(PIN_J2_STEP, PIN_J2_DIR);
static Stepper g_j3(PIN_J3_STEP, PIN_J3_DIR);
static StepControl g_motion;

static Servo g_j4_servo;
static Servo g_gripper_servo;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------

struct State {
    uint16_t fault_flags;
    uint16_t state_flags;
    uint32_t last_heartbeat_ms;
    uint32_t last_fault_scan_ms;
    uint32_t last_eeprom_write_ms;
    uint32_t target_updates_since_eeprom;
    uint8_t  rx_seq_last;       // most-recent SEQ seen, informational
    bool     rx_seq_valid;

    int32_t  j1_target_steps;
    int32_t  j2_target_steps;
    int32_t  j3_target_steps;
    uint16_t j4_us;
    uint16_t gripper_us;

    // ALM debounce
    uint32_t j1_alm_low_since;  // 0 = high, otherwise ms timestamp
    uint32_t j2_alm_low_since;
    uint32_t j3_alm_low_since;
};
static State g_state;

// ---------------------------------------------------------------------------
// Forward declarations (the Arduino IDE only auto-prototypes non-static fns)
// ---------------------------------------------------------------------------

static uint16_t crc16_ccitt(const uint8_t *data, size_t len);
static void     sendFrame(uint8_t type, const uint8_t *payload, uint8_t len);
static void     sendAck(uint8_t acked_seq, uint8_t acked_type);
static void     sendNak(uint8_t nak_seq, uint8_t nak_type, uint8_t err);
static void     parseFrame();
static void     handleFrame(uint8_t seq, uint8_t type,
                            const uint8_t *payload, uint8_t len);
static void     handleSetJointTargets(uint8_t seq, const uint8_t *p, uint8_t len);
static void     handleSetJointVelocities(uint8_t seq, const uint8_t *p, uint8_t len);
static void     handleHomeRequest(uint8_t seq, const uint8_t *p, uint8_t len);
static void     handleClearFaults(uint8_t seq, const uint8_t *p, uint8_t len);
static void     handleGetStatus(uint8_t seq);
static uint8_t  recoverFromEeprom();
static void     checkpointEeprom();
static void     applyDriverEnable(bool enable);
static void     stopAllMotion();
static void     writeServos(uint16_t j4_us, uint16_t gripper_us);
static bool     motionAllowed(uint8_t &err_out);
static int16_t  readMcuTempC10();
static void     scanFaults();
static void     serviceMotion();
static void     maybeCheckpointEeprom();

// ---------------------------------------------------------------------------
// CRC-16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no final XOR)
// ---------------------------------------------------------------------------

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Frame I/O
// ---------------------------------------------------------------------------

static void sendFrame(uint8_t type, const uint8_t *payload, uint8_t len) {
    static uint8_t s_seq = 0;
    uint8_t seq = s_seq;
    s_seq = (uint8_t)(s_seq + 1);

    // CRC over [seq, type, payload]
    uint8_t crc_buf[2 + kMaxPayload];
    crc_buf[0] = seq;
    crc_buf[1] = type;
    if (len) memcpy(crc_buf + 2, payload, len);
    uint16_t crc = crc16_ccitt(crc_buf, (size_t)len + 2);

    Serial.write(kFrameStart);
    Serial.write(len);
    Serial.write(seq);
    Serial.write(type);
    if (len) Serial.write(payload, len);
    Serial.write((uint8_t)(crc & 0xFF));
    Serial.write((uint8_t)((crc >> 8) & 0xFF));
    Serial.write(kFrameEnd);
}

static void sendAck(uint8_t acked_seq, uint8_t acked_type) {
    uint8_t p[2] = { acked_seq, acked_type };
    sendFrame(RSP_ACK, p, sizeof(p));
}

static void sendNak(uint8_t nak_seq, uint8_t nak_type, uint8_t err) {
    uint8_t p[3] = { nak_seq, nak_type, err };
    sendFrame(RSP_NAK, p, sizeof(p));
}

// Tiny streaming parser. Call as often as possible from loop().
// On a complete, CRC-validated frame, dispatches to handleFrame().
static void parseFrame() {
    enum Phase : uint8_t { WAIT_START, READ_LEN, READ_SEQ, READ_TYPE,
                           READ_PAYLOAD, READ_CRC_LO, READ_CRC_HI, READ_END };
    static Phase    s_phase = WAIT_START;
    static uint8_t  s_len = 0;
    static uint8_t  s_seq = 0;
    static uint8_t  s_type = 0;
    static uint8_t  s_payload[kMaxPayload];
    static uint8_t  s_pidx = 0;
    static uint8_t  s_crc_lo = 0;
    static uint16_t s_crc = 0;
    static uint32_t s_started_ms = 0;

    // 50 ms inter-byte timeout once we've seen START
    if (s_phase != WAIT_START && (millis() - s_started_ms) > 50) {
        s_phase = WAIT_START;
    }

    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        switch (s_phase) {
            case WAIT_START:
                if (b == kFrameStart) {
                    s_phase = READ_LEN;
                    s_started_ms = millis();
                }
                break;
            case READ_LEN:
                if (b > kMaxPayload) {
                    g_state.fault_flags |= FAULT_FRAME_OVERFLOW;
                    s_phase = WAIT_START;
                } else {
                    s_len = b;
                    s_phase = READ_SEQ;
                }
                break;
            case READ_SEQ:
                s_seq = b;
                s_phase = READ_TYPE;
                break;
            case READ_TYPE:
                s_type = b;
                s_pidx = 0;
                s_phase = (s_len == 0) ? READ_CRC_LO : READ_PAYLOAD;
                break;
            case READ_PAYLOAD:
                s_payload[s_pidx++] = b;
                if (s_pidx >= s_len) s_phase = READ_CRC_LO;
                break;
            case READ_CRC_LO:
                s_crc_lo = b;
                s_phase = READ_CRC_HI;
                break;
            case READ_CRC_HI:
                s_crc = (uint16_t)s_crc_lo | ((uint16_t)b << 8);
                s_phase = READ_END;
                break;
            case READ_END: {
                if (b != kFrameEnd) {
                    sendNak(s_seq, s_type, ERR_CRC);
                    s_phase = WAIT_START;
                    break;
                }
                // Validate CRC over [seq, type, payload]
                uint8_t crc_buf[2 + kMaxPayload];
                crc_buf[0] = s_seq;
                crc_buf[1] = s_type;
                memcpy(crc_buf + 2, s_payload, s_len);
                uint16_t c = crc16_ccitt(crc_buf, (size_t)s_len + 2);
                if (c != s_crc) {
                    sendNak(s_seq, s_type, ERR_CRC);
                    s_phase = WAIT_START;
                    break;
                }

                g_state.rx_seq_last = s_seq;
                g_state.rx_seq_valid = true;
                handleFrame(s_seq, s_type, s_payload, s_len);
                s_phase = WAIT_START;
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// EEPROM ping-pong
// ---------------------------------------------------------------------------

static uint16_t slot_crc(const EepromSlot &s) {
    // CRC over the slot bytes EXCLUDING the trailing crc + reserved2 fields.
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&s);
    return crc16_ccitt(p, sizeof(EepromSlot) - 4);
}

static bool readSlot(int offset, EepromSlot &out) {
    uint8_t *p = reinterpret_cast<uint8_t *>(&out);
    for (size_t i = 0; i < sizeof(EepromSlot); ++i) p[i] = EEPROM.read(offset + i);
    if (out.magic != kEepromMagic) return false;
    if (out.version != kEepromVersion) return false;
    return slot_crc(out) == out.crc;
}

static void writeSlot(int offset, EepromSlot &s) {
    s.magic = kEepromMagic;
    s.version = kEepromVersion;
    s.reserved = 0;
    s.reserved2 = 0;
    s.crc = slot_crc(s);
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&s);
    for (size_t i = 0; i < sizeof(EepromSlot); ++i) EEPROM.update(offset + i, p[i]);
}

// Returns: 0 = used slot 0, 1 = used slot 1, 0xFF = neither valid.
static uint8_t recoverFromEeprom() {
    EepromSlot s0, s1;
    bool v0 = readSlot(kEepromSlot0Off, s0);
    bool v1 = readSlot(kEepromSlot1Off, s1);

    if (!v0 && !v1) {
        g_state.fault_flags |= FAULT_EEPROM_CRC | FAULT_UNCALIBRATED;
        g_state.j1_target_steps = 0;
        g_state.j2_target_steps = 0;
        g_state.j3_target_steps = 0;
        g_state.j4_us = kJ4DefaultUs;
        g_state.gripper_us = kGripperDefaultUs;
        return 0xFF;
    }

    EepromSlot pick;
    uint8_t which;
    if (v0 && v1) {
        // Modular comparison so we keep working after wrap.
        uint32_t diff = s0.write_count - s1.write_count;
        if ((int32_t)diff > 0) { pick = s0; which = 0; }
        else                   { pick = s1; which = 1; }
    } else if (v0) {
        pick = s0; which = 0;
    } else {
        pick = s1; which = 1;
    }

    g_state.j1_target_steps = pick.j1_steps;
    g_state.j2_target_steps = pick.j2_steps;
    g_state.j3_target_steps = pick.j3_steps;
    g_state.j4_us            = pick.j4_us;
    g_state.gripper_us       = pick.gripper_us;
    // Recovered positions count as homed (we trust them).
    g_state.state_flags |= STATE_HOMED_J1 | STATE_HOMED_J2 | STATE_HOMED_J3;
    return which;
}

static void checkpointEeprom() {
    // Pick the slot we did NOT just use as "current" (highest write_count) and
    // overwrite it with current state + write_count + 1.
    EepromSlot s0, s1;
    bool v0 = readSlot(kEepromSlot0Off, s0);
    bool v1 = readSlot(kEepromSlot1Off, s1);

    uint32_t next_count;
    int dst_offset;
    if (v0 && v1) {
        if ((int32_t)(s0.write_count - s1.write_count) > 0) {
            next_count = s0.write_count + 1;
            dst_offset = kEepromSlot1Off;
        } else {
            next_count = s1.write_count + 1;
            dst_offset = kEepromSlot0Off;
        }
    } else if (v0) {
        next_count = s0.write_count + 1;
        dst_offset = kEepromSlot1Off;
    } else if (v1) {
        next_count = s1.write_count + 1;
        dst_offset = kEepromSlot0Off;
    } else {
        next_count = 1;
        dst_offset = kEepromSlot0Off;
    }

    EepromSlot dst{};
    dst.j1_steps = g_state.j1_target_steps;
    dst.j2_steps = g_state.j2_target_steps;
    dst.j3_steps = g_state.j3_target_steps;
    dst.j4_us    = g_state.j4_us;
    dst.gripper_us = g_state.gripper_us;
    dst.write_count = next_count;
    writeSlot(dst_offset, dst);

    g_state.last_eeprom_write_ms = millis();
    g_state.target_updates_since_eeprom = 0;
}

// ---------------------------------------------------------------------------
// Motion / servo helpers
// ---------------------------------------------------------------------------

static void applyDriverEnable(bool enable) {
#if MOCK_MODE
    Serial.print(F("[MOCK] driver_enable=")); Serial.println(enable ? 1 : 0);
#else
    // CL57T-V41 ENA- is active-low. The HCT245 inverts logic? No — the '245
    // is a buffer, not an inverter. So drive PIN_DRV_ENABLE *low* to assert
    // ENA-. The driver datasheet inverts this depending on common-anode vs
    // common-cathode wiring of the optocoupler; verify on bench.
    digitalWriteFast(PIN_DRV_ENABLE, enable ? LOW : HIGH);
#endif
    if (enable) g_state.state_flags |= STATE_ENABLED;
    else        g_state.state_flags &= ~STATE_ENABLED;
}

static void stopAllMotion() {
#if MOCK_MODE
    Serial.println(F("[MOCK] stop_all"));
#else
    if (g_motion.isRunning()) g_motion.stopAsync();
#endif
    g_state.state_flags &= ~STATE_MOVING;
}

static void writeServos(uint16_t j4_us, uint16_t gripper_us) {
#if MOCK_MODE
    Serial.print(F("[MOCK] j4_us=")); Serial.print(j4_us);
    Serial.print(F(" gripper_us=")); Serial.println(gripper_us);
#else
    g_j4_servo.writeMicroseconds(j4_us);
    g_gripper_servo.writeMicroseconds(gripper_us);
#endif
    g_state.j4_us = j4_us;
    g_state.gripper_us = gripper_us;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static bool motionAllowed(uint8_t &err_out) {
    if (g_state.fault_flags) { err_out = ERR_FAULT_ACTIVE; return false; }
    if (!(g_state.state_flags & STATE_ENABLED)) { err_out = ERR_DISABLED; return false; }
    if (!(g_state.state_flags & STATE_HOMED_J1) ||
        !(g_state.state_flags & STATE_HOMED_J2) ||
        !(g_state.state_flags & STATE_HOMED_J3)) {
        err_out = ERR_NOT_HOMED;
        return false;
    }
    return true;
}

static void handleSetJointTargets(uint8_t seq, const uint8_t *p, uint8_t len) {
    if (len != 18) { sendNak(seq, CMD_SET_JOINT_TARGETS, ERR_LENGTH); return; }
    uint8_t err = ERR_NONE;
    if (!motionAllowed(err)) { sendNak(seq, CMD_SET_JOINT_TARGETS, err); return; }

    int32_t j1, j2, j3;
    uint16_t j4, gr, flags;
    memcpy(&j1, p + 0,  4);
    memcpy(&j2, p + 4,  4);
    memcpy(&j3, p + 8,  4);
    memcpy(&j4, p + 12, 2);
    memcpy(&gr, p + 14, 2);
    memcpy(&flags, p + 16, 2);
    (void)flags;  // TODO: honor clamp_to_limits bit once limits are wired

    if (j4 < kJ4MinUs || j4 > kJ4MaxUs ||
        gr < kGripperMinUs || gr > kGripperMaxUs) {
        g_state.fault_flags |= FAULT_BAD_PARAMETER;
        sendNak(seq, CMD_SET_JOINT_TARGETS, ERR_BAD_PARAMETER);
        return;
    }

    g_state.j1_target_steps = j1;
    g_state.j2_target_steps = j2;
    g_state.j3_target_steps = j3;

#if MOCK_MODE
    Serial.print(F("[MOCK] targets j1=")); Serial.print(j1);
    Serial.print(F(" j2=")); Serial.print(j2);
    Serial.print(F(" j3=")); Serial.println(j3);
#else
    g_j1.setTargetAbs(j1);
    g_j2.setTargetAbs(j2);
    g_j3.setTargetAbs(j3);
    if (g_motion.isRunning()) g_motion.stopAsync();
    g_motion.moveAsync(g_j1, g_j2, g_j3);
#endif
    g_state.state_flags |= STATE_MOVING;
    writeServos(j4, gr);

    g_state.target_updates_since_eeprom++;
    if (g_state.target_updates_since_eeprom >= kEepromCheckpointEveryN) {
        checkpointEeprom();
    }

    sendAck(seq, CMD_SET_JOINT_TARGETS);
}

static void handleSetJointVelocities(uint8_t seq, const uint8_t *p, uint8_t len) {
    if (len != 12) { sendNak(seq, CMD_SET_JOINT_VELOCITIES, ERR_LENGTH); return; }
    int32_t v1, v2, v3;
    memcpy(&v1, p + 0, 4);
    memcpy(&v2, p + 4, 4);
    memcpy(&v3, p + 8, 4);

#if MOCK_MODE
    Serial.print(F("[MOCK] vmax j1=")); Serial.print(v1);
    Serial.print(F(" j2=")); Serial.print(v2);
    Serial.print(F(" j3=")); Serial.println(v3);
#else
    // TeensyStep takes unsigned max speed; sign of velocity is implied by the
    // sign of (target - current). For now we use abs() and treat the sign as
    // informational. TODO: implement true velocity-mode with continuous moves.
    g_j1.setMaxSpeed(v1 < 0 ? -v1 : v1);
    g_j2.setMaxSpeed(v2 < 0 ? -v2 : v2);
    g_j3.setMaxSpeed(v3 < 0 ? -v3 : v3);
#endif

    sendAck(seq, CMD_SET_JOINT_VELOCITIES);
}

static void handleHomeRequest(uint8_t seq, const uint8_t *p, uint8_t len) {
    if (len != 1) { sendNak(seq, CMD_HOME_REQUEST, ERR_LENGTH); return; }
    // TODO: implement homing. Likely a per-joint move-until-ALM sequence with
    // a slow back-off, then setPosition(0). Until that exists, NAK with
    // ERR_NOT_IMPLEMENTED so the host fails fast.
    (void)p;
    sendNak(seq, CMD_HOME_REQUEST, ERR_NOT_IMPLEMENTED);
}

static void handleClearFaults(uint8_t seq, const uint8_t *p, uint8_t len) {
    if (len != 2) { sendNak(seq, CMD_CLEAR_FAULTS, ERR_LENGTH); return; }
    uint16_t mask;
    memcpy(&mask, p, 2);
    g_state.fault_flags &= (uint16_t)~mask;
    sendAck(seq, CMD_CLEAR_FAULTS);
}

static void handleGetStatus(uint8_t /*seq*/) {
    uint8_t buf[28];
    memset(buf, 0, sizeof(buf));
    int32_t j1 = g_state.j1_target_steps;
    int32_t j2 = g_state.j2_target_steps;
    int32_t j3 = g_state.j3_target_steps;
    memcpy(buf + 0,  &j1, 4);
    memcpy(buf + 4,  &j2, 4);
    memcpy(buf + 8,  &j3, 4);
    memcpy(buf + 12, &g_state.j4_us, 2);
    memcpy(buf + 14, &g_state.gripper_us, 2);
    memcpy(buf + 16, &g_state.fault_flags, 2);
    memcpy(buf + 18, &g_state.state_flags, 2);
    uint32_t up = millis();
    memcpy(buf + 20, &up, 4);
    int16_t temp_c10 = readMcuTempC10();
    memcpy(buf + 24, &temp_c10, 2);
    // buf[26..27] reserved zero
    sendFrame(RSP_STATUS, buf, sizeof(buf));
}

static void handleFrame(uint8_t seq, uint8_t type,
                        const uint8_t *payload, uint8_t len) {
    g_state.last_heartbeat_ms = millis();   // any valid CMD feeds the watchdog

    switch (type) {
        case CMD_HEARTBEAT:
            if (len != 0) { sendNak(seq, type, ERR_LENGTH); return; }
            sendAck(seq, type);
            break;
        case CMD_GET_STATUS:
            if (len != 0) { sendNak(seq, type, ERR_LENGTH); return; }
            handleGetStatus(seq);
            break;
        case CMD_ENABLE:
            if (len != 0) { sendNak(seq, type, ERR_LENGTH); return; }
            if (g_state.fault_flags) { sendNak(seq, type, ERR_FAULT_ACTIVE); return; }
            applyDriverEnable(true);
            sendAck(seq, type);
            break;
        case CMD_DISABLE:
            if (len != 0) { sendNak(seq, type, ERR_LENGTH); return; }
            applyDriverEnable(false);
            stopAllMotion();
            sendAck(seq, type);
            break;
        case CMD_SET_JOINT_TARGETS:
            handleSetJointTargets(seq, payload, len);
            break;
        case CMD_SET_JOINT_VELOCITIES:
            handleSetJointVelocities(seq, payload, len);
            break;
        case CMD_HOME_REQUEST:
            handleHomeRequest(seq, payload, len);
            break;
        case CMD_CLEAR_FAULTS:
            handleClearFaults(seq, payload, len);
            break;
        default:
            sendNak(seq, type, ERR_UNKNOWN_TYPE);
            break;
    }
}

// ---------------------------------------------------------------------------
// Periodic services
// ---------------------------------------------------------------------------

// Teensy 4.1 has an internal temperature sensor available via tempmonGetTemp()
// in newer cores. Returned in 0.1 C units. TODO: replace stub with real call
// once the target Arduino core is pinned.
static int16_t readMcuTempC10() {
    // TODO: return (int16_t)(tempmonGetTemp() * 10.0f);
    return 0;
}

static void scanFaults() {
    uint32_t now = millis();
    uint16_t prev = g_state.fault_flags;

    // Heartbeat watchdog
    if ((now - g_state.last_heartbeat_ms) > kHeartbeatTimeoutMs) {
        if (!(g_state.fault_flags & FAULT_HEARTBEAT_TIMEOUT)) {
            g_state.fault_flags |= FAULT_HEARTBEAT_TIMEOUT;
            applyDriverEnable(false);
            stopAllMotion();
        }
    }

    // E-STOP (active-low input)
#if !MOCK_MODE
    if (digitalReadFast(PIN_ESTOP) == LOW) g_state.fault_flags |= FAULT_E_STOP;
#endif

    // ALM debounce — bit is set if pin held LOW for >= kAlmDebounceMs
#if !MOCK_MODE
    auto debounce = [&](uint8_t pin, uint32_t &low_since, uint16_t bit) {
        bool low = (digitalReadFast(pin) == LOW);
        if (low) {
            if (low_since == 0) low_since = now;
            else if ((now - low_since) >= kAlmDebounceMs) g_state.fault_flags |= bit;
        } else {
            low_since = 0;
        }
    };
    debounce(PIN_J1_ALM, g_state.j1_alm_low_since, FAULT_DRIVER_ALARM_J1);
    debounce(PIN_J2_ALM, g_state.j2_alm_low_since, FAULT_DRIVER_ALARM_J2);
    debounce(PIN_J3_ALM, g_state.j3_alm_low_since, FAULT_DRIVER_ALARM_J3);
#endif

    // Over-temperature
    int16_t t = readMcuTempC10();
    if (t >= kOverTempC10) g_state.fault_flags |= FAULT_OVER_TEMP;

    // EVT_FAULT on rising edges
    uint16_t rising = (uint16_t)(g_state.fault_flags & ~prev);
    if (rising) {
        applyDriverEnable(false);
        stopAllMotion();
        uint8_t p[4];
        memcpy(p + 0, &rising, 2);
        memcpy(p + 2, &g_state.fault_flags, 2);
        sendFrame(EVT_FAULT, p, sizeof(p));
    }
}

static void serviceMotion() {
#if !MOCK_MODE
    if (!g_motion.isRunning() && (g_state.state_flags & STATE_MOVING)) {
        g_state.state_flags &= ~STATE_MOVING;
    }
#endif
}

static void maybeCheckpointEeprom() {
    if (g_state.target_updates_since_eeprom == 0) return;
    if ((millis() - g_state.last_eeprom_write_ms) >= kEepromCheckpointMaxMs) {
        checkpointEeprom();
    }
}

// ---------------------------------------------------------------------------
// setup() / loop()
// ---------------------------------------------------------------------------

void setup() {
    memset(&g_state, 0, sizeof(g_state));
    g_state.last_heartbeat_ms = millis();   // grace period at boot

    Serial.begin(1000000);
    // No while(!Serial) — USB host might not be present on a headless boot.

#if !MOCK_MODE
    pinMode(PIN_J1_STEP, OUTPUT); pinMode(PIN_J1_DIR, OUTPUT);
    pinMode(PIN_J2_STEP, OUTPUT); pinMode(PIN_J2_DIR, OUTPUT);
    pinMode(PIN_J3_STEP, OUTPUT); pinMode(PIN_J3_DIR, OUTPUT);
    pinMode(PIN_DRV_ENABLE, OUTPUT);
    digitalWriteFast(PIN_DRV_ENABLE, HIGH);  // disabled at boot

    // Level shifter: DIR=high (A->B), OE held high (Hi-Z) until pins are set
    pinMode(PIN_HCT245_OE_N, OUTPUT);
    pinMode(PIN_HCT245_DIR, OUTPUT);
    digitalWriteFast(PIN_HCT245_OE_N, HIGH);
    digitalWriteFast(PIN_HCT245_DIR, HIGH);

    pinMode(PIN_J1_ALM, INPUT_PULLUP);
    pinMode(PIN_J2_ALM, INPUT_PULLUP);
    pinMode(PIN_J3_ALM, INPUT_PULLUP);
    pinMode(PIN_ESTOP,  INPUT_PULLUP);
    pinMode(PIN_LED,    OUTPUT);

    // Configure TeensyStep
    g_j1.setMaxSpeed(kJ1Cfg.max_speed_sps).setAcceleration(kJ1Cfg.max_accel_sps2);
    g_j2.setMaxSpeed(kJ2Cfg.max_speed_sps).setAcceleration(kJ2Cfg.max_accel_sps2);
    g_j3.setMaxSpeed(kJ3Cfg.max_speed_sps).setAcceleration(kJ3Cfg.max_accel_sps2);

    // Servos
    g_j4_servo.attach(PIN_J4_SERVO,      kJ4MinUs,      kJ4MaxUs);
    g_gripper_servo.attach(PIN_GRIPPER_SERVO, kGripperMinUs, kGripperMaxUs);
#endif

    // Recover positions from EEPROM
    uint8_t slot = recoverFromEeprom();
    bool recovered = (slot != 0xFF);

#if !MOCK_MODE
    // Now that step/dir lines are at known states, enable the level shifter.
    digitalWriteFast(PIN_HCT245_OE_N, LOW);

    // Push servo defaults out immediately so they don't twitch on first move.
    g_j4_servo.writeMicroseconds(g_state.j4_us);
    g_gripper_servo.writeMicroseconds(g_state.gripper_us);

    // Set TeensyStep "current position" to recovered counts.
    g_j1.setPosition(g_state.j1_target_steps);
    g_j2.setPosition(g_state.j2_target_steps);
    g_j3.setPosition(g_state.j3_target_steps);
#endif

    // Announce boot
    uint8_t boot[8];
    boot[0] = PROTO_MAJOR;
    boot[1] = PROTO_MINOR;
    boot[2] = slot;
    boot[3] = recovered ? 1 : 0;
    uint32_t bid = FIRMWARE_BUILD_ID;
    memcpy(boot + 4, &bid, 4);
    sendFrame(EVT_BOOT, boot, sizeof(boot));

    g_state.last_heartbeat_ms = millis();
    g_state.last_fault_scan_ms = millis();
    g_state.last_eeprom_write_ms = millis();
}

void loop() {
    parseFrame();

    uint32_t now = millis();
    if ((now - g_state.last_fault_scan_ms) >= kFaultScanPeriodMs) {
        g_state.last_fault_scan_ms = now;
        scanFaults();
    }

    serviceMotion();
    maybeCheckpointEeprom();

    // Heartbeat-LED: 1 Hz steady when healthy, 8 Hz blink when faulted.
    static uint32_t s_last_led = 0;
    uint32_t period = g_state.fault_flags ? 125 : 500;
    if ((now - s_last_led) >= period) {
        s_last_led = now;
#if !MOCK_MODE
        digitalWriteFast(PIN_LED, !digitalReadFast(PIN_LED));
#endif
    }
}
