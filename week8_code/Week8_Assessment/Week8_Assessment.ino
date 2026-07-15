/*
 * MTRN3100 Week 8 Assessment — Simple Driving
 * z5465761, T2 2026
 *
 * Task 1: Drive straight 1m
 * Task 2: Approach and hold 100mm from front wall (3 challenges)
 * Task 3: 90 deg CW turn, then hold heading against disturbance
 * Task 4: Execute command string (f/l/r)
 *
 * Sensor layout (top plate flipped 180 deg):
 *   Front LiDAR: A2, 0x54
 *   Right LiDAR: A1, 0x56
 *   Left  LiDAR: A0, 0x58
 *
 * Confirmed by testing:
 *   CW rotation -> yaw negative (IMU_CW_SIGN = -1)
 *   Both encoders count up going forward
 *
 * edStem #66: MPU6050_light sets gyro range while sensor is asleep on
 * battery power. Fix: manually write GYRO_CONFIG register after mpu.begin().
 */

// Set this to select the task, then upload
#define TASK_NUM  2   // 1=straight | 2=wall hold | 3=turn | 4=commands

#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"

// Motor pins (DRV8835, Phase/Enable mode)
#define MOT1_PWM  9
#define MOT1_DIR  10
#define MOT2_PWM  11
#define MOT2_DIR  12

// Encoder pins — interrupt on channel A, direction from channel B
#define ENC1_A  2
#define ENC1_B  4
#define ENC2_A  3
#define ENC2_B  5

// LiDAR enable pins (GPIO0 on VL6180X, from PCB schematic)
#define LIDAR_FRONT_EN  A2
#define LIDAR_RIGHT_EN  A1
#define LIDAR_LEFT_EN   A0

// Physical parameters (measured)
const float    WHEEL_RADIUS_M  = 0.016f;
const float    AXLE_LENGTH_M   = 0.092f;
const float    WHEEL_CIRC_MM   = 2.0f * PI * 16.0f;
const float    CELL_MM         = 180.0f;
const uint16_t CPR             = 690;

const float IMU_CW_SIGN        = -1.0f;
const bool  RIGHT_ENC_INVERTED = false;

// Tuning (set by testing)
const int   DRIVE_SPEED        = 140;
const int   TURN_SPEED         = 120;
const float HEADING_KP         = 2.0f;
const float TURN_KP            = 3.0f;
const float FRONT_KP           = 2.0f;
const float FRONT_KI           = 0.02f;
const float FRONT_KD           = 0.3f;
const float TURN_TOLERANCE_DEG = 3.0f;
const float WALL_TOLERANCE_MM  = 5.0f;
const int   START_DELAY_MS     = 3000;
const int   SETTLE_MS          = 400;

mtrn3100::Motor           motorL(MOT1_PWM, MOT1_DIR);
mtrn3100::Motor           motorR(MOT2_PWM, MOT2_DIR);
mtrn3100::DualEncoder     encoder(ENC1_A, ENC1_B, ENC2_A, ENC2_B, RIGHT_ENC_INVERTED);
mtrn3100::EncoderOdometry odometry(WHEEL_RADIUS_M, AXLE_LENGTH_M);

MPU6050 mpu(Wire);
VL6180X lidarFront;
VL6180X lidarRight;
VL6180X lidarLeft;

void stopMotors() { motorL.setPWM(0); motorR.setPWM(0); }
float getYaw()    { mpu.update(); return mpu.getAngleZ(); }


// Task 1: drive distanceMM forward using encoder + IMU heading correction.
// Timeout = 30ms per mm, matching the 30s spec limit.
void driveForward(float distanceMM) {
    float targetRads = (distanceMM / WHEEL_CIRC_MM) * 2.0f * PI;
    encoder.reset();
    float startYaw = getYaw();

    unsigned long start   = millis();
    unsigned long timeout = (unsigned long)(distanceMM * 30UL);

    while (true) {
        if (millis() - start > timeout) { Serial.println("[TIMEOUT]"); break; }

        float avgRads = (encoder.getLeftRotation() + encoder.getRightRotation()) / 2.0f;
        if (avgRads >= targetRads) break;

        // Heading correction: yaw drifted CW -> speed up left motor, slow right
        float yawError   = startYaw - getYaw();
        float correction = constrain(yawError * HEADING_KP, -60.0f, 60.0f);

        motorL.setPWM(DRIVE_SPEED + (int)correction);
        motorR.setPWM(DRIVE_SPEED - (int)correction);

        Serial.print("dist="); Serial.print(avgRads * WHEEL_RADIUS_M * 1000.0f, 0);
        Serial.print("mm  yaw="); Serial.println(getYaw(), 1);
        delay(10);
    }

    stopMotors();
    delay(500);
    Serial.println("Drive done.");
}


// Task 2: approach wall and hold at targetMM.
// Phase 1: drive forward (with heading correction) until LiDAR reads < 255.
//          VL6180X returns 255 when nothing is in range.
// Phase 2: PID hold. error = rawMM - targetMM (positive = too far = drive forward).
//          3-point sensor average and low-pass derivative reduce jitter from sensor noise.
void holdFrontDistance(float targetMM) {

    // Phase 1: approach
    float startYaw = getYaw();
    unsigned long approachStart = millis();
    bool acquired = false;
    int  acquiredMM = 0;

    while (millis() - approachStart < 10000UL) {
        int rawMM = lidarFront.readRangeSingleMillimeters();
        if (!lidarFront.timeoutOccurred() && rawMM < 255) {
            acquiredMM = rawMM;
            acquired   = true;
            break;
        }
        float yawErr = startYaw - getYaw();
        int   corr   = (int)constrain(yawErr * HEADING_KP, -40.0f, 40.0f);
        motorL.setPWM(100 + corr);
        motorR.setPWM(100 - corr);
        delay(20);
    }
    stopMotors();
    delay(50);

    if (!acquired) {
        Serial.println("Wall not found.");
        return;
    }
    Serial.print("Wall at "); Serial.print(acquiredMM); Serial.println("mm.");

    // Phase 2: PID hold
    int   dBuf[3]     = {(int)targetMM, (int)targetMM, (int)targetMM};
    uint8_t dIdx      = 0;
    float integral    = 0.0f;
    float prevError   = 0.0f;
    float smoothDeriv = 0.0f;
    unsigned long holdStart    = millis();
    unsigned long settledSince = 0;
    bool settled = false;

    while (millis() - holdStart < 12000UL) {
        int rawMM = lidarFront.readRangeSingleMillimeters();
        if (lidarFront.timeoutOccurred() || rawMM >= 255) {
            delay(20); continue;
        }

        // 3-point moving average
        dBuf[dIdx] = rawMM;
        dIdx = (dIdx + 1) % 3;
        float avgMM = (dBuf[0] + dBuf[1] + dBuf[2]) / 3.0f;

        float error = avgMM - targetMM;
        float dt    = 0.02f;

        integral += error * dt;
        integral  = constrain(integral, -500.0f, 500.0f);

        // Low-pass filtered derivative to reduce noise spikes
        float rawDeriv = (error - prevError) / dt;
        smoothDeriv    = 0.7f * smoothDeriv + 0.3f * rawDeriv;
        prevError      = error;

        float output = constrain(
            FRONT_KP * error + FRONT_KI * integral + FRONT_KD * smoothDeriv,
            -160.0f, 160.0f);

        motorL.setPWM((int)output);
        motorR.setPWM((int)output);

        Serial.print("raw="); Serial.print(rawMM);
        Serial.print(" avg="); Serial.print(avgMM, 1);
        Serial.print(" err="); Serial.print(error, 1);
        Serial.print(" out="); Serial.println(output, 0);

        // Settle timer with hysteresis: start at <=5mm, reset only if >9mm
        if (fabs(error) <= WALL_TOLERANCE_MM) {
            if (!settled) { settledSince = millis(); settled = true; }
            if (millis() - settledSince >= (unsigned long)SETTLE_MS) {
                stopMotors();
                Serial.println("Settled.");
                return;
            }
        } else if (fabs(error) > WALL_TOLERANCE_MM + 4.0f) {
            settled = false;
        }

        delay(20);
    }

    stopMotors();
    Serial.println("Timeout.");
}


// Task 3: turn degrees CW (positive) or CCW (negative) using IMU.
void turnIMU(float degrees) {
    float targetYaw = getYaw() + IMU_CW_SIGN * degrees;
    Serial.print("Turning to "); Serial.println(targetYaw, 1);

    unsigned long start        = millis();
    unsigned long settledSince = 0;
    bool settled = false;

    while (millis() - start < 10000) {
        float error = targetYaw - getYaw();

        if (fabs(error) <= TURN_TOLERANCE_DEG) {
            stopMotors();
            if (!settled) { settledSince = millis(); settled = true; }
            if (millis() - settledSince >= SETTLE_MS) break;
        } else {
            settled = false;
            float output = constrain(error * TURN_KP, -(float)TURN_SPEED, (float)TURN_SPEED);
            motorL.setPWM( (int)output);
            motorR.setPWM(-(int)output);
        }
        delay(10);
    }

    stopMotors();
    delay(300);
    Serial.print("Turn done. Yaw="); Serial.println(getYaw(), 1);
}

// Task 3 challenge 2: actively hold a heading if the robot is rotated.
void holdHeading(float targetYaw, unsigned long durationMs = 25000) {
    Serial.print("Holding "); Serial.print(targetYaw, 1); Serial.println(" deg.");

    unsigned long start = millis();
    while (millis() - start < durationMs) {
        float error  = targetYaw - getYaw();
        float output = constrain(error * TURN_KP, -(float)TURN_SPEED, (float)TURN_SPEED);

        if (fabs(error) <= TURN_TOLERANCE_DEG) stopMotors();
        else {
            motorL.setPWM( (int)output);
            motorR.setPWM(-(int)output);
        }
        delay(10);
    }

    stopMotors();
}


// Task 4: f=forward 1 cell, l=CCW 90, r=CW 90
void executeCommands(const char* commands) {
    Serial.print("Commands: "); Serial.println(commands);
    for (int i = 0; commands[i] != '\0'; i++) {
        switch (commands[i]) {
            case 'f': driveForward(CELL_MM); break;
            case 'r': turnIMU( 90.0f);       break;
            case 'l': turnIMU(-90.0f);        break;
            default:  Serial.print("Unknown: "); Serial.println(commands[i]);
        }
        delay(200);
    }
    Serial.println("Done.");
}


void initLidars() {
    pinMode(LIDAR_FRONT_EN, OUTPUT);
    pinMode(LIDAR_RIGHT_EN, OUTPUT);
    pinMode(LIDAR_LEFT_EN,  OUTPUT);

    // Pull all LOW so every sensor resets to default I2C address 0x29
    digitalWrite(LIDAR_FRONT_EN, LOW);
    digitalWrite(LIDAR_RIGHT_EN, LOW);
    digitalWrite(LIDAR_LEFT_EN,  LOW);
    delay(20);

    // Bring each sensor up one at a time and assign unique address
    digitalWrite(LIDAR_FRONT_EN, HIGH); delay(50);
    lidarFront.init(); lidarFront.configureDefault();
    lidarFront.setTimeout(250); lidarFront.setAddress(0x54);

    digitalWrite(LIDAR_RIGHT_EN, HIGH); delay(50);
    lidarRight.init(); lidarRight.configureDefault();
    lidarRight.setTimeout(250); lidarRight.setAddress(0x56);

    digitalWrite(LIDAR_LEFT_EN, HIGH); delay(50);
    lidarLeft.init(); lidarLeft.configureDefault();
    lidarLeft.setTimeout(250); lidarLeft.setAddress(0x58);

    Serial.println("LiDARs ready.");
}

void initIMU() {
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("IMU failed ("); Serial.print(status); Serial.println(").");
        while (1);
    }
    // edStem #66: force +/-500 deg/s gyro range (battery-mode bug fix)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Serial.println("Calibrating IMU, hold still...");
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println("IMU ready.");
}


void setup() {
    Serial.begin(115200);
    Wire.begin();
    encoder.counts_per_revolution = CPR;
    stopMotors();
}

// Everything runs from loop() so sensors re-init cleanly on every power-on.
// After task finishes: flip switch OFF within 5s. If left ON, it runs again.
void loop() {
    stopMotors();

    // Reset I2C in case bus was left in a bad state from a previous run
    delay(500);
    TWCR = 0;
    delay(20);
    Wire.begin();

    initLidars();
    initIMU();

    Serial.print("\nTask "); Serial.print(TASK_NUM);
    Serial.print(" starting in "); Serial.print(START_DELAY_MS / 1000); Serial.println("s...");
    delay(START_DELAY_MS);

    #if TASK_NUM == 1
        driveForward(1006.0f);   // 1006mm tuned to account for encoder undershoot

    #elif TASK_NUM == 2
        holdFrontDistance(100.0f);
        stopMotors(); delay(300);
        Serial.println("8s gap...");
        delay(8000);

        holdFrontDistance(100.0f);
        stopMotors(); delay(300);
        Serial.println("8s gap...");
        delay(8000);

        holdFrontDistance(100.0f);
        stopMotors();

    #elif TASK_NUM == 3
        turnIMU(90.0f);
        float setpoint = getYaw();
        delay(500);
        holdHeading(setpoint, 25000);

    #elif TASK_NUM == 4
        executeCommands("lfrfflfr");   // <- change on the day

    #endif

    stopMotors();
    Serial.println("Done. Switch off.");
    delay(5000);
}
