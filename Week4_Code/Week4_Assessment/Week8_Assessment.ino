/*
 * MTRN3100 — Week 4 Barebones Movement Assessment
 * ─────────────────────────────────────────────────
 * Task 1: Drive 200mm forward   — LEFT encoder only (enc2 has some issues)
 * Task 2: 4x CCW + 4x CW turns — TIME controlled
 */

#include "Motor.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "PIDController.hpp"
#include "Lidar.hpp"

// ═══════════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════════

const uint16_t CPR         = 690;
const int   DRIVE_SPEED    = 150;
const int   TURN_SPEED     = 120;
const float DEADBAND_RAD   = 0.15;
const int   START_DELAY_MS = 3000;
const int   SETTLE_MS      = 500;

// ms for one 90° turn.
// Robot turns too little → increase. Too much → decrease.
const int TURN_DURATION_MS = 480;

// ═══════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════

#define MOT1_PWM  9
#define MOT1_DIR  10
#define MOT2_PWM  11
#define MOT2_DIR  12
#define ENC1_A    2    // Left encoder interrupt pin — confirmed working
#define ENC1_B    4    // Left encoder direction pin — confirmed working
#define LID_F     19   // Front Lidar input pin
#define LID_L     20   // Left Lidar input pin
#define LID_R     21   // Right Lidar input pin

// ═══════════════════════════════════════════════════════════════════
//  DERIVED CONSTANTS
// ═══════════════════════════════════════════════════════════════════

const float WHEEL_CIRC_MM = PI * 32.0;
const float DRIVE_RADS    = (200.0 / WHEEL_CIRC_MM) * 2.0 * PI;  // ~12.5 rad

// ═══════════════════════════════════════════════════════════════════
//  OBJECTS  (enc2 removed)
// ═══════════════════════════════════════════════════════════════════

mtrn3100::Motor   motorL(MOT1_PWM, MOT1_DIR);
mtrn3100::Motor   motorR(MOT2_PWM, MOT2_DIR);
mtrn3100::Encoder encoderL(ENC1_A, ENC1_B);
mtrn3100::Lidar   lidarF(LID_F);
mtrn3100::Lidar   lidarL(LID_L);
mtrn3100::Lidar   lidarR(LID_R);

// ═══════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════

void stopMotors() {
    motorL.setPWM(0);
    motorR.setPWM(0);
}

// Task 1: drive both motors forward, stop when LEFT encoder hits target.
// Both motors always commanded together — no enc2 dependency.
void driveForward(float targetRads) {
    encoderL.reset();

    Serial.print("  Driving forward "); Serial.print(targetRads, 2); Serial.println(" rad...");

    // Safety timeout: 200mm should take well under 5 seconds
    unsigned long startTime = millis();

    while (true) {
        if (millis() - startTime > 5000) {
            Serial.print(" [TIMEOUT]");
            break;
        }

        float posL = encoderL.getRotation();
        float errL = targetRads - posL;

        if (errL <= DEADBAND_RAD) break;

        // Both motors run together at same speed — straight line
        motorL.setPWM(DRIVE_SPEED);
        motorR.setPWM(DRIVE_SPEED);
        delay(10);
    }

    stopMotors();
    delay(SETTLE_MS);
    Serial.println("  Done.");
}

// Task 2: timed 90° turn — no encoder, no failure modes.
// ccw=true  → left back, right forward  (CCW / left turn)
// ccw=false → left forward, right back  (CW  / right turn)
void turn90(bool ccw) {
    Serial.print(ccw ? "  CCW 90..." : "  CW 90...");

    if (!ccw) {
        motorL.setPWM(-TURN_SPEED);
        motorR.setPWM( TURN_SPEED);
    } else {
        motorL.setPWM( TURN_SPEED);
        motorR.setPWM(-TURN_SPEED);
    }

    delay(TURN_DURATION_MS);

    stopMotors();
    delay(SETTLE_MS);
    Serial.println(" Done.");
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(9600);

    encoderL.counts_per_revolution = CPR;

    Serial.println("=== MTRN3100 Week 4 Assessment ===");
    Serial.print("CPR="); Serial.print(CPR);
    Serial.print("  DRIVE_RADS="); Serial.print(DRIVE_RADS, 2);
    Serial.print("  TURN_MS="); Serial.println(TURN_DURATION_MS);
    Serial.print("Starting in "); Serial.print(START_DELAY_MS / 1000); Serial.println("s...");

    delay(START_DELAY_MS);

    // ── TASK 1: Drive 200mm ──
    Serial.println("\n--- TASK 1: Drive 200mm ---");
    driveForward(DRIVE_RADS);
    delay(500);

    // ── TASK 2: 4x CCW ──
    Serial.println("\n--- TASK 2: 4x CCW ---");
    for (int i = 0; i < 4; i++) {
        Serial.print("CCW "); Serial.print(i+1); Serial.print("/4 ");
        turn90(true);
    }
    delay(500);

    // ── TASK 2: 4x CW ──
    Serial.println("\n--- TASK 2: 4x CW ---");
    for (int i = 0; i < 4; i++) {
        Serial.print("CW "); Serial.print(i+1); Serial.print("/4 ");
        turn90(false);
    }

    stopMotors();
    Serial.println("\n=== COMPLETE ===");
}

void loop() {

  driveForward()

}