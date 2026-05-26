// SPDX-License-Identifier: MIT
//
// FORTIS Teensy 4.1 — J4 servo home-command demo sketch.
//
// Purpose: visually demonstrate the CMD_HOME_REQUEST behavior for the J4
// servo (mask bit 3) WITHOUT needing the host-side protocol bridge that
// Nikhil + Claude are writing. Mirrors the exact behavior the main firmware
// implements in handleHomeRequest() for bit 3:
//
//     mask & 0x08  ->  writeServos(kJ4HomeUs, current_gripper_us)
//
// Cycle:
//   1. Drive J4 to an OFFSET position (1000 us) so a "home" move is obvious
//   2. Wait 3 seconds
//   3. Fire the "home" routine -> snap to kJ4HomeUs (1500 us)
//   4. Wait 3 seconds
//   5. Drive J4 to the OTHER offset (1900 us)
//   6. Wait 3 seconds
//   7. Fire "home" again -> snap to 1500 us
//   8. Wait 3 seconds, then loop
//
// What "pass" looks like at the servo:
//   - J4 visibly snaps to mid-position on each "home" event
//   - Serial Monitor prints '[home_j4] home -> 1500 us'
//
// Wiring (must match main firmware):
//   Signal (orange/yellow) ──→ Teensy pin 28
//   Servo +6V (red)        ──→ EXTERNAL 6V supply (+), 2A limit
//   Servo GND (brown/black)──→ EXTERNAL 6V supply (-)
//                              │
//                              └──→ Teensy GND  (any GND pin)
//
// Safety:
//   - DO NOT power the D845WP from Teensy USB. Use external 6V/2A.
//   - Common ground between Teensy and supply is mandatory.
//   - kJ4MinUs/kJ4MaxUs must stay within this physical servo's safe range
//     as calibrated in teensy.ino (currently 800-2000 us). If you ever
//     change the constants here, mirror the change in teensy.ino too.

#include <Arduino.h>
#include <Servo.h>

// Mirror the main firmware's J4 calibration. Keep in sync with teensy.ino.
static const int      kJ4Pin       = 28;
static const uint16_t kJ4MinUs     = 800;    // calibrated safe floor (stalls below ~700)
static const uint16_t kJ4MaxUs     = 2000;
static const uint16_t kJ4HomeUs    = 1000;   // FORTIS J4 standard home (stowed/parked pose)
static const uint16_t kOffsetLowUs = 800;    // safe-floor offset; visible delta from home
static const uint16_t kOffsetHiUs  = 1900;

static const uint32_t kStepDelayMs = 3000;

static Servo g_j4_servo;

// --- The actual "home command" routine -------------------------------------
//
// This is the exact substance of handleHomeRequest()'s mask-bit-3 branch in
// teensy.ino, lifted out of the protocol-handling code. In the real firmware
// it runs because the host sent CMD_HOME_REQUEST with mask=0x08. Here we
// just call it directly on a timer.

static void simulate_cmd_home_j4() {
    g_j4_servo.writeMicroseconds(kJ4HomeUs);
    Serial.print(F("[home_j4] home -> "));
    Serial.print(kJ4HomeUs);
    Serial.println(F(" us"));
}

static void drive_offset(uint16_t us) {
    g_j4_servo.writeMicroseconds(us);
    Serial.print(F("[home_j4] offset -> "));
    Serial.print(us);
    Serial.println(F(" us"));
}

void setup() {
    Serial.begin(1000000);
    delay(500);
    Serial.println(F("[home_j4] boot. Pin 28, cycles offset -> home -> offset -> home."));

    g_j4_servo.attach(kJ4Pin, kJ4MinUs, kJ4MaxUs);
    g_j4_servo.writeMicroseconds(kJ4HomeUs);   // start at neutral
    Serial.print(F("[home_j4] startup at home: "));
    Serial.println(kJ4HomeUs);
    delay(1000);
}

void loop() {
    drive_offset(kOffsetLowUs);
    delay(kStepDelayMs);

    simulate_cmd_home_j4();
    delay(kStepDelayMs);

    drive_offset(kOffsetHiUs);
    delay(kStepDelayMs);

    simulate_cmd_home_j4();
    delay(kStepDelayMs);
}
