// SPDX-License-Identifier: MIT
//
// FORTIS Teensy 4.1 — single-joint stepper bring-up ("nudge") test sketch.
//
// Purpose: prove the full motion chain for ONE joint end to end —
//   Teensy 3.3V STEP/DIR/ENA  →  TXS0108E level shifter  →  CL57T-V41 driver
//   →  NEMA-23 closed-loop stepper.
// This is the FIRST time the steppers move. The arm is assembled, so this
// sketch is deliberately conservative: nothing moves on boot, every move is
// small, slow, and triggered by a single keypress you send over USB serial.
//
// IT DOES NOT USE THE FORTIS PROTOCOL. It bypasses homing / software limits
// on purpose so we can verify wiring before any of that exists. Keep moves
// tiny until you know which way the joint travels and how far.
//
// Defaults target J1. To test J2 or J3, change PIN_STEP / PIN_DIR / PIN_ALM
// below to that joint's pins (see the firmware pin map) and re-flash.
//
//   J1 STEP/DIR/ALM = 2 / 3 / 22
//   J2 STEP/DIR/ALM = 4 / 5 / 23
//   J3 STEP/DIR/ALM = 6 / 7 / 20
//
// ---------------------------------------------------------------------------
// SAFETY — read before powering the motor supply:
//   1. Flash THIS sketch first, with the motor supply OFF. Confirm the serial
//      menu prints. The driver boots DISABLED and the shifter boots Hi-Z, so
//      no pulses reach the driver until you press 'e'.
//   2. Before connecting the driver, verify ~5V STEP/DIR/ENA edges at the
//      TXS0108E B-side (press 'f' a few times and scope/meter the outputs).
//   3. Verify CL57T-V41 ENA polarity with a multimeter. If 'e' does NOT
//      actually enable the driver (motor not holding torque), the opto wiring
//      is the opposite polarity — see kEnableActiveLow below.
//   4. Keep a hand on the motor-supply switch for the first moves. Start with
//      the smallest nudge ('1'). Increase only once direction + scale are known.
// ---------------------------------------------------------------------------
//
// Serial menu (1000000 baud, send a single character):
//   e : ENABLE driver  (asserts ENA, then enables the level shifter)
//   d : DISABLE driver (Hi-Z shifter, de-asserts ENA)
//   f : nudge FORWARD  by the current step count
//   b : nudge BACKWARD by the current step count
//   1 : set nudge size = 20 steps   (tiny)
//   2 : set nudge size = 100 steps
//   3 : set nudge size = 400 steps
//   z : zero the position counter (does not move the motor)
//   ? : reprint this menu + current status

#include <Arduino.h>
#include "teensystep4.h"

using TS4::Stepper;

// ---- Pin map (J1 by default — must match firmware/teensy/teensy.ino) -------
static const int PIN_STEP      = 2;    // J1 STEP
static const int PIN_DIR       = 3;    // J1 DIR
static const int PIN_ALM       = 22;   // J1 ALM (driver alarm input)
static const int PIN_DRV_ENA   = 9;    // shared driver ENABLE
static const int PIN_LS_OE     = 14;   // TXS0108E OE (active-HIGH: HIGH=enabled)

// ---- Behavior config -------------------------------------------------------
// CL57T-V41 ENA- is active-low in the firmware's assumption. If the bench
// multimeter check shows the opposite, flip this to false and re-flash.
static const bool     kEnableActiveLow = true;

// Motion math for this joint (StepperOnline 23HS22-2804-HG50-ME1K):
//   200 full steps/rev * 8 microstep      = 1600 steps per MOTOR rev
//   * 50:1 gearbox                         = 80,000 steps per JOINT rev
//   => 0.0045 deg of joint travel per step
//   => ~356 steps just to take up the 1.6 deg gearbox backlash
// So nudges below a few hundred steps won't visibly move the joint at all.
// The presets below ('1'/'2'/'3') are sized in JOINT degrees and are all
// small; '3' is ~72 deg of joint travel, so only use it after you've
// confirmed the arm has clearance to swing that far.
static const uint32_t kNudgeSpeed = 1600;  // steps/s at the motor (~7.2 deg/s at the joint) — slow + observable

// ALM is read with an internal pull-up. Open-collector drivers usually pull
// LOW when faulted, so LOW = alarm. We only REPORT it here; verify on bench.
static const int      kAlmActiveState = LOW;

static Stepper g_joint(PIN_STEP, PIN_DIR);

static int32_t g_nudge   = 800;     // current nudge size (steps); ~3.6 deg at the joint
static bool    g_enabled = false;
static bool    g_running = false;   // true while in continuous free-run mode ('r')
static int     g_lastAlm = -1;

static void setDriverEnable(bool enable) {
    // Drive ENA to the asserted level for "enabled".
    if (kEnableActiveLow) digitalWriteFast(PIN_DRV_ENA, enable ? LOW : HIGH);
    else                  digitalWriteFast(PIN_DRV_ENA, enable ? HIGH : LOW);
    g_enabled = enable;
}

static void printMenu() {
    Serial.println(F("\n[stepper_nudge] J1 bring-up. Send one char:"));
    Serial.println(F("  e=enable  d=disable  f=fwd  b=back"));
    Serial.println(F("  r=run-forever(fwd)  s=stop"));
    Serial.println(F("  1=800steps  2=4000steps  3=16000steps  z=zero  ?=help"));
    Serial.print(F("  status: "));
    Serial.print(g_enabled ? F("ENABLED") : F("disabled"));
    if (g_running) Serial.print(F(" RUNNING"));
    Serial.print(F("  nudge=")); Serial.print(g_nudge);
    Serial.print(F(" steps  pos=")); Serial.print(g_joint.getPosition());
    Serial.print(F("  ALM=")); Serial.println(digitalRead(PIN_ALM));
}

// Stop any continuous run cleanly.
static void doStop() {
    if (g_running) {
        g_joint.stopAsync();
        g_running = false;
        Serial.print(F("[stepper_nudge] STOPPED. pos=")); Serial.println(g_joint.getPosition());
    }
}

// Continuous free-run forward — pulses forever until 's' (or an ALM fault).
// WARNING: once the driver+motor are connected this drives the joint
// nonstop until it hits a hard stop. Use only for the no-load scope test,
// or with clearance + a hand on the stop key.
static void doRun() {
    if (!g_enabled) {
        Serial.println(F("[stepper_nudge] driver DISABLED — press 'e' first."));
        return;
    }
    if (digitalRead(PIN_ALM) == kAlmActiveState) {
        Serial.println(F("[stepper_nudge] ALM active — driver faulted. Not running."));
        return;
    }
    g_running = true;
    g_joint.rotateAsync(kNudgeSpeed);   // non-blocking: pulses continuously
    Serial.println(F("[stepper_nudge] RUNNING forward forever. Press 's' to stop."));
}

static void doNudge(int32_t delta) {
    if (!g_enabled) {
        Serial.println(F("[stepper_nudge] driver DISABLED — press 'e' first."));
        return;
    }
    if (g_running) {
        Serial.println(F("[stepper_nudge] in RUN mode — press 's' to stop before nudging."));
        return;
    }
    if (digitalRead(PIN_ALM) == kAlmActiveState) {
        Serial.println(F("[stepper_nudge] ALM active — driver faulted. Not moving."));
        return;
    }
    Serial.print(F("[stepper_nudge] moving ")); Serial.print(delta);
    Serial.print(F(" steps @ ")); Serial.print(kNudgeSpeed); Serial.println(F(" sps..."));
    g_joint.moveRel(delta, kNudgeSpeed);   // blocking; small + slow by design
    Serial.print(F("[stepper_nudge] done. pos=")); Serial.println(g_joint.getPosition());
}

void setup() {
    Serial.begin(1000000);
    delay(500);

    // Boot everything to a SAFE state before the shifter is enabled.
    pinMode(PIN_STEP, OUTPUT);    digitalWriteFast(PIN_STEP, LOW);
    pinMode(PIN_DIR,  OUTPUT);    digitalWriteFast(PIN_DIR,  LOW);
    pinMode(PIN_DRV_ENA, OUTPUT);
    setDriverEnable(false);                       // driver disabled at boot
    pinMode(PIN_ALM, INPUT_PULLUP);

    pinMode(PIN_LS_OE, OUTPUT);
    digitalWriteFast(PIN_LS_OE, LOW);             // shifter Hi-Z until pins settle

    TS4::begin();
    g_joint.setMaxSpeed(kNudgeSpeed).setAcceleration(1000);
    g_joint.setPosition(0);

    // Pins are now at known states — enable the level shifter outputs.
    digitalWriteFast(PIN_LS_OE, HIGH);

    Serial.println(F("[stepper_nudge] boot. Driver DISABLED, shifter live."));
    printMenu();
}

void loop() {
    // Report ALM transitions so a driver fault is obvious even when idle.
    int alm = digitalRead(PIN_ALM);
    if (alm != g_lastAlm) {
        g_lastAlm = alm;
        Serial.print(F("[stepper_nudge] ALM line = ")); Serial.print(alm);
        Serial.println(alm == kAlmActiveState ? F("  (ALARM)") : F("  (ok)"));
    }

    // Safety: if a fault appears while free-running, stop immediately.
    if (g_running && alm == kAlmActiveState) {
        Serial.println(F("[stepper_nudge] ALM during run — auto-stopping."));
        doStop();
    }

    if (!Serial.available()) return;
    char c = (char)Serial.read();
    switch (c) {
        case 'e': setDriverEnable(true);
                  Serial.println(F("[stepper_nudge] driver ENABLED (motor should hold torque).")); break;
        case 'd': doStop(); setDriverEnable(false);
                  Serial.println(F("[stepper_nudge] driver disabled.")); break;
        case 'f': doNudge(+g_nudge); break;
        case 'b': doNudge(-g_nudge); break;
        case 'r': doRun();  break;
        case 's': doStop(); break;
        case '1': g_nudge = 800;   Serial.println(F("[stepper_nudge] nudge = 800 steps (~3.6 deg joint)"));  break;
        case '2': g_nudge = 4000;  Serial.println(F("[stepper_nudge] nudge = 4000 steps (~18 deg joint)")); break;
        case '3': g_nudge = 16000; Serial.println(F("[stepper_nudge] nudge = 16000 steps (~72 deg joint)")); break;
        case 'z': g_joint.setPosition(0); Serial.println(F("[stepper_nudge] position zeroed.")); break;
        case '?': printMenu(); break;
        case '\n': case '\r': break;        // ignore line endings
        default:  break;
    }
}
