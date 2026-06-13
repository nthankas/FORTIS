// SPDX-License-Identifier: MIT
//
// FORTIS Teensy 4.1 — 3-joint arm bring-up ("nudge") test sketch.
//
// Extends tests/stepper_j1_nudge to all three steppers (J1/J2/J3) so you can
// jog each joint individually OR move them together as a coordinated group
// (each joint moved async, then waited on together). Bypasses the FORTIS
// protocol / homing / software limits on purpose — hardware bring-up only.
//
// Pin map matches firmware/teensy/teensy.ino:
//   J1 STEP/DIR/ALM = 6 / 7 / 22   (STEP/DIR swapped with J3 for this bench wiring)
//   J2 STEP/DIR/ALM = 4 / 5 / 23
//   J3 STEP/DIR/ALM = 2 / 3 / 20   (STEP/DIR swapped with J1 for this bench wiring)
//   shared ENABLE   = 9          (one ENA line feeds all three drivers)
//   level-shifter OE = 14
//
// Joints that aren't physically wired just pulse into nothing — harmless.
//
// ---------------------------------------------------------------------------
// SAFETY: the arm is assembled. There are NO software limits and NO homing,
// so the firmware has no idea where the joints are. Every default here is
// small and slow. The continuous 'r' (run-forever) mode will drive a joint
// nonstop into a hard stop if you let it — keep a hand on 's' and the power.
// Start with one joint selected; only use ALL ('0') once you trust each joint.
// ---------------------------------------------------------------------------
//
// Serial menu (1000000 baud, single char):
//   e : ENABLE all drivers      d : DISABLE all drivers
//   1 : select J1   2 : select J2   3 : select J3   0 : select ALL (group)
//   f : nudge selected FORWARD   b : nudge selected BACK
//   r : run selected FOREVER (fwd)   s : STOP all motion (controlled ramp)
//   SPACEBAR : KILL — disable drivers instantly (motors go limp)
//   + : bigger nudge   - : smaller nudge
//   < : slower   > : faster   (motor top speed, changeable live / mid-run)
//   z : zero all position counters   ? : help + status
//   J4 wrist servo (D845WP, pin 28):
//   [ : -50 us   ] : +50 us   h : home (1000 us)   n : neutral (1500 us)
//   gripper servo (pin 29):
//   , : -50 us   . : +50 us   o : open (2000 us)   c : close (1400 us)

#include <Arduino.h>
#include <Servo.h>
#include "teensystep4.h"

using TS4::Stepper;

// ---- Pin map (must match firmware/teensy/teensy.ino) -----------------------
static const int PIN_ENABLE = 9;    // shared, active-low at the driver
static const int PIN_LS_OE  = 14;   // TXS0108E OE, active-HIGH
static const int PIN_ALM[3] = { 22, 23, 20 };   // J1, J2, J3
static const int PIN_J4     = 28;   // J4 wrist servo (D845WP), FlexPWM @ 50 Hz

// J4 servo, bench-calibrated safe range (stalls below ~700 / above ~2000 us).
static const uint16_t kJ4MinUs     = 800;
static const uint16_t kJ4MaxUs     = 2000;
static const uint16_t kJ4NeutralUs = 1500;   // safe mid pose at boot
static const uint16_t kJ4HomeUs    = 2000;   // FORTIS J4 "home"/stowed pose (was 1000)
static const uint16_t kJ4StepUs    = 50;     // per-press fine adjust

static const int PIN_GRIP = 29;   // gripper servo, FlexPWM @ 50 Hz

// Gripper servo, conservative range (boots OPEN/released so it can't clamp).
static const uint16_t kGripMinUs   = 1200;
static const uint16_t kGripMaxUs   = 2000;
static const uint16_t kGripOpenUs  = 2000;   // released
static const uint16_t kGripCloseUs = 1400;   // gripping
static const uint16_t kGripStepUs  = 50;     // per-press fine adjust

// ---- Behavior config -------------------------------------------------------
static const bool     kEnableActiveLow = true;   // flip if bench check shows opposite
static const uint32_t kSpeed           = 6400;   // DEFAULT steps/s at the motor (live-adjustable with < / >)
static const uint32_t kAccel           = 20000;  // steps/s^2 (ramp to top speed in ~0.3 s)
static const uint32_t kSpeedMin        = 800;    // floor for < key
static const uint32_t kSpeedMax        = 32000;  // ceiling for > key (stay well under driver/motor limits)
static const uint32_t kSpeedStep       = 1600;   // change per < / > press
static const int      kAlmActiveState  = LOW;    // LOW = alarm (report only; verify on bench)

// 1600 steps/motor-rev * 50:1 gearbox = 80,000 steps/joint-rev => 0.0045 deg/step.
// Nudge presets (steps) and roughly what they are at the joint:
static const int32_t  kNudgeSmall = 800;    // ~3.6 deg
static const int32_t  kNudgeMed   = 4000;   // ~18 deg
static const int32_t  kNudgeBig   = 16000;  // ~72 deg

static Stepper  g_j1(6, 7);
static Stepper  g_j2(4, 5);
static Stepper  g_j3(2, 3);
static Stepper* g_steppers[3] = { &g_j1, &g_j2, &g_j3 };
static const char* g_names[3] = { "J1", "J2", "J3" };

static Servo   g_j4;
static uint16_t g_j4_us = kJ4NeutralUs;
static Servo   g_grip;
static uint16_t g_grip_us = kGripOpenUs;

static int      g_sel     = 0;        // 0/1/2 = J1/J2/J3, -1 = ALL
static int32_t  g_nudge   = kNudgeSmall;
static uint32_t g_speed   = kSpeed;   // current top speed (steps/s), live-adjustable
static bool     g_enabled = false;
static bool     g_running = false;
static int      g_lastAlm[3] = { -1, -1, -1 };

static bool selIsAll() { return g_sel < 0; }

static bool anyMoving() { return g_j1.isMoving || g_j2.isMoving || g_j3.isMoving; }

// Change the live top speed by one step (dir = +1 faster, -1 slower), clamp,
// and if we're currently free-running, re-issue so it takes effect instantly.
static void changeSpeed(int dir) {
    int32_t s = (int32_t)g_speed + dir * (int32_t)kSpeedStep;
    if (s < (int32_t)kSpeedMin) s = kSpeedMin;
    if (s > (int32_t)kSpeedMax) s = kSpeedMax;
    g_speed = (uint32_t)s;
    for (int i = 0; i < 3; i++) g_steppers[i]->setMaxSpeed(g_speed);
    Serial.print(F("[arm] speed = ")); Serial.print(g_speed);
    Serial.print(F(" sps (~")); Serial.print(g_speed * 360 / 80000);
    Serial.println(F(" deg/s at joint)"));
    if (g_running) {   // apply live to an in-progress run
        if (selIsAll()) for (int i = 0; i < 3; i++) g_steppers[i]->rotateAsync(g_speed);
        else            g_steppers[g_sel]->rotateAsync(g_speed);
    }
}

// Move the J4 wrist servo to an absolute microsecond target, clamped to the
// calibrated safe range. The servo responds immediately (no enable needed).
static void setJ4(int us) {
    if (us < kJ4MinUs) us = kJ4MinUs;
    if (us > kJ4MaxUs) us = kJ4MaxUs;
    g_j4_us = (uint16_t)us;
    g_j4.writeMicroseconds(g_j4_us);
    Serial.print(F("[arm] J4 = ")); Serial.print(g_j4_us); Serial.println(F(" us"));
}

// Move the gripper servo to an absolute microsecond target, clamped to range.
static void setGrip(int us) {
    if (us < kGripMinUs) us = kGripMinUs;
    if (us > kGripMaxUs) us = kGripMaxUs;
    g_grip_us = (uint16_t)us;
    g_grip.writeMicroseconds(g_grip_us);
    Serial.print(F("[arm] GRIP = ")); Serial.print(g_grip_us); Serial.println(F(" us"));
}

static const __FlashStringHelper* selName() {
    return selIsAll() ? F("ALL") : reinterpret_cast<const __FlashStringHelper*>(g_names[g_sel]);
}

static void setEnable(bool en) {
    if (kEnableActiveLow) digitalWriteFast(PIN_ENABLE, en ? LOW : HIGH);
    else                  digitalWriteFast(PIN_ENABLE, en ? HIGH : LOW);
    g_enabled = en;
}

static void stopAll() {
    for (int i = 0; i < 3; i++) g_steppers[i]->stopAsync();
    if (g_running) {
        g_running = false;
        Serial.println(F("[arm] STOPPED."));
    }
}

static void printStatus() {
    Serial.print(F("[arm] sel=")); Serial.print(selName());
    Serial.print(g_enabled ? F("  ENABLED") : F("  disabled"));
    if (g_running) Serial.print(F("  RUNNING"));
    Serial.print(F("  nudge=")); Serial.print(g_nudge);
    Serial.print(F(" steps  speed=")); Serial.print(g_speed); Serial.print(F("sps"));
    Serial.print(F("  pos[J1,J2,J3]="));
    Serial.print(g_j1.getPosition()); Serial.print(',');
    Serial.print(g_j2.getPosition()); Serial.print(',');
    Serial.print(g_j3.getPosition());
    Serial.print(F("  J4=")); Serial.print(g_j4_us); Serial.print(F("us"));
    Serial.print(F("  GRIP=")); Serial.print(g_grip_us); Serial.println(F("us"));
}

static void printMenu() {
    Serial.println(F("\n[arm] 3-joint bring-up. Send one char:"));
    Serial.println(F("  e=enable d=disable | 1/2/3=select J1/J2/J3  0=select ALL"));
    Serial.println(F("  f=fwd b=back | r=run-forever s=STOP | +/-=nudge size | z=zero ?=help"));
    Serial.println(F("  < = slower   > = faster  (motor speed, live)"));
    Serial.println(F("  SPACEBAR = *** KILL *** (disable drivers instantly)"));
    Serial.println(F("  J4 wrist servo:  [ = -50us   ] = +50us   h = home(1000)   n = neutral(1500)"));
    Serial.println(F("  gripper servo:   , = -50us   . = +50us   o = open(2000)   c = close(1400)"));
    printStatus();
}

// Is any selected joint's ALM asserting a fault?
static bool selAlarmActive() {
    if (selIsAll()) {
        for (int i = 0; i < 3; i++)
            if (digitalRead(PIN_ALM[i]) == kAlmActiveState) return true;
        return false;
    }
    return digitalRead(PIN_ALM[g_sel]) == kAlmActiveState;
}

static bool preMoveChecks() {
    if (!g_enabled) { Serial.println(F("[arm] DISABLED — press 'e' first.")); return false; }
    if (g_running)  { Serial.println(F("[arm] RUNNING — press 's' first.")); return false; }
    if (selAlarmActive()) { Serial.println(F("[arm] ALM active on a selected joint — not moving.")); return false; }
    return true;
}

static void doNudge(int32_t delta) {
    if (!preMoveChecks()) return;
    if (selIsAll()) {
        // Coordinated move: kick off all three async, then wait for them to
        // finish. Equal delta + speed => they start/stop together. (Done
        // manually instead of StepperGroup to avoid std::vector in this sketch.)
        Serial.print(F("[arm] group move ")); Serial.print(delta); Serial.println(F(" steps..."));
        for (int i = 0; i < 3; i++)
            g_steppers[i]->moveRelAsync(delta, g_speed);
        while (anyMoving()) delay(1);   // blocking until all three finish
    } else {
        Serial.print(F("[arm] ")); Serial.print(g_names[g_sel]);
        Serial.print(F(" move ")); Serial.print(delta); Serial.println(F(" steps..."));
        g_steppers[g_sel]->moveRel(delta, g_speed);   // blocking
    }
    printStatus();
}

static void doRun() {
    if (!preMoveChecks()) return;
    g_running = true;
    if (selIsAll()) for (int i = 0; i < 3; i++) g_steppers[i]->rotateAsync(g_speed);
    else            g_steppers[g_sel]->rotateAsync(g_speed);
    Serial.print(F("[arm] RUNNING ")); Serial.print(selName());
    Serial.println(F(" forward forever. Press 's' to stop."));
}

void setup() {
    Serial.begin(1000000);
    delay(500);

    for (int i = 0; i < 3; i++) pinMode(PIN_ALM[i], INPUT_PULLUP);
    pinMode(PIN_ENABLE, OUTPUT);
    setEnable(false);                       // disabled at boot

    pinMode(PIN_LS_OE, OUTPUT);
    digitalWriteFast(PIN_LS_OE, LOW);       // shifter Hi-Z until pins settle

    TS4::begin();
    for (int i = 0; i < 3; i++) {
        g_steppers[i]->setMaxSpeed(g_speed).setAcceleration(kAccel);
        g_steppers[i]->setPosition(0);
    }

    digitalWriteFast(PIN_LS_OE, HIGH);      // pins settled — enable shifter

    // Servos: attach and move to safe poses (wrist neutral, gripper open).
    g_j4.attach(PIN_J4, kJ4MinUs, kJ4MaxUs);
    setJ4(kJ4NeutralUs);
    setGrip(kGripOpenUs);

    Serial.println(F("[arm] boot. All drivers DISABLED, shifter live."));
    printMenu();
}

void loop() {
    // Report ALM transitions per joint; auto-stop a run if a fault appears.
    for (int i = 0; i < 3; i++) {
        int alm = digitalRead(PIN_ALM[i]);
        if (alm != g_lastAlm[i]) {
            g_lastAlm[i] = alm;
            Serial.print(F("[arm] ")); Serial.print(g_names[i]);
            Serial.print(F(" ALM = ")); Serial.print(alm);
            Serial.println(alm == kAlmActiveState ? F("  (ALARM)") : F("  (ok)"));
        }
    }
    if (g_running && selAlarmActive()) {
        Serial.println(F("[arm] ALM during run — auto-stopping."));
        stopAll();
    }

    if (!Serial.available()) return;
    char c = (char)Serial.read();
    switch (c) {
        case 'e': setEnable(true);  Serial.println(F("[arm] ENABLED (joints should hold torque).")); break;
        case 'd': stopAll(); setEnable(false); Serial.println(F("[arm] disabled.")); break;
        case '1': g_sel = 0;  Serial.println(F("[arm] selected J1"));  break;
        case '2': g_sel = 1;  Serial.println(F("[arm] selected J2"));  break;
        case '3': g_sel = 2;  Serial.println(F("[arm] selected J3"));  break;
        case '0': g_sel = -1; Serial.println(F("[arm] selected ALL (group)")); break;
        case 'f': doNudge(+g_nudge); break;
        case 'b': doNudge(-g_nudge); break;
        case 'r': doRun();  break;
        case 's': stopAll(); break;
        case ' ': // SPACEBAR = panic kill: cut drive instantly (motors go limp)
            stopAll(); setEnable(false);
            Serial.println(F("[arm] *** KILL *** drivers disabled."));
            break;
        case '+': g_nudge = (g_nudge == kNudgeSmall) ? kNudgeMed : kNudgeBig;
                  Serial.print(F("[arm] nudge = ")); Serial.print(g_nudge); Serial.println(F(" steps")); break;
        case '-': g_nudge = (g_nudge == kNudgeBig) ? kNudgeMed : kNudgeSmall;
                  Serial.print(F("[arm] nudge = ")); Serial.print(g_nudge); Serial.println(F(" steps")); break;
        case 'z': for (int i = 0; i < 3; i++) g_steppers[i]->setPosition(0);
                  Serial.println(F("[arm] all positions zeroed.")); break;
        case '<': changeSpeed(-1); break;
        case '>': changeSpeed(+1); break;
        case '[': setJ4(g_j4_us - kJ4StepUs); break;
        case ']': setJ4(g_j4_us + kJ4StepUs); break;
        case 'h': setJ4(kJ4HomeUs);    break;
        case 'n': setJ4(kJ4NeutralUs); break;
        case ',': setGrip(g_grip_us - kGripStepUs); break;
        case '.': setGrip(g_grip_us + kGripStepUs); break;
        case 'o': setGrip(kGripOpenUs);  break;
        case 'c': setGrip(kGripCloseUs); break;
        case '?': printMenu(); break;
        case '\n': case '\r': break;
        default: break;
    }
}
