/**
 * Controls the motion of the micromouse
 * 
 * Moves micromouse forward one cell, turns left, turns right 90 degrees
 * Incorporates the odometry, PID, Lidars
 */

#pragma once

#include "Lidars.hpp"
#include "Pose.hpp"
#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"
#include <MPU6050_light.h>
#include <Arduino.h>

// Per-loop telemetry. Leave this at 0 for real runs: at 115200 baud each
// line blocks the control loop for several milliseconds, which shows up as
// jitter in the heading correction. Set to 1 only when diagnosing.
#ifndef MOTION_DEBUG
#define MOTION_DEBUG 0
#endif

namespace mtrn3100 {

class MotionController {

public:
    static constexpr float CELL_MM = 180.0f;
    static constexpr float WHEEL_CIRC_MM = 2.0f * PI * 16.0f;
    static constexpr float TIMEOUT_MS = 28000.0f;

    static constexpr float TURN_TOLERANCE_RAD = 0.05f;
    static constexpr float DISTANCE_TOLERANCE_RAD = 0.025f;
    static constexpr float DEADBAND = 0.0f;
    static constexpr float MINIMUM_PWM = 55.0f;

    // --- Task 4.2 continuous section -------------------------------------
    // Inside the obstacle course there are no walls to snap against, but the
    // front LiDAR is still useful as a last-resort collision guard against a
    // cylinder. Scanning is slow, so it is polled every GUARD_PERIOD loops.
    static constexpr float   EMERGENCY_STOP_MM = 55.0f;
    static constexpr uint8_t GUARD_PERIOD      = 10;    // loops (~100 ms)

    // --- Acceleration limiting -------------------------------------------
    // A proportional controller asks for its largest output when the error is
    // largest, which is the instant the robot starts. For a 400 mm leg that
    // works out well past 255, so the motors go from rest to full power in one
    // control step. With the wheels at the back, that torque spike pitches the
    // nose up and the robot rocks back onto its tail.
    //
    // Ramping the commanded PWM instead of stepping it fixes this. Only the
    // rise is limited -- the controller may always cut power immediately, so
    // braking and the emergency stop stay sharp.
    //
    // Larger PWM_SLEW_UP = brisker start. Drop it to ~4 if it still rocks;
    // raise it to ~10 if the start feels sluggish.
    static constexpr float PWM_SLEW_UP   = 6.0f;    // max PWM increase per loop
    static constexpr float TURN_SLEW_UP  = 10.0f;   // same, for pivoting
    static constexpr float DEFAULT_MAX_PWM = 255.0f;

    MotionController(Pose& pose, Lidars& lidars, MPU6050& imu, Motor& motorL, Motor& motorR,
                        DualEncoder& encoder, PIDController& headingPid, 
                        PIDController& distancePid)
        : pose(pose), lidars(lidars), mpu(imu), motorL(motorL), motorR(motorR),
        encoder(encoder), headingPid(headingPid), distancePid(distancePid) {}

    
    void setLidarCorrection(bool enabled) {lidarCorrectionEnabled = enabled;}

    // Enable the front-LiDAR collision guard. Used for Task 4.2, where a
    // cylinder in front means "stop", not "snap to a wall".
    void setObstacleGuard(bool enabled) { obstacleGuardEnabled = enabled; }

    // Ceiling on drive PWM. Lower it for the obstacle course: there are no
    // walls to re-square against in there, so a slower, steadier run holds its
    // heading better than a fast one.
    void setSpeedLimit(float maxPwm) {
        maxDrivePwm = constrain(maxPwm, MINIMUM_PWM, 255.0f);
    }

    // Returns true if the last motion aborted because the guard tripped.
    bool guardTripped() const { return guardTrip; }

    // ------------------------------------------------------------------
    // Metric primitives — arbitrary distances and angles.
    // forwardOneCell()/turnLeft90()/turnRight90() are grid-specific wrappers
    // around these; Task 4.2 waypoint following uses them directly.
    // ------------------------------------------------------------------

    // Drives straight for an arbitrary distance while holding the current
    // heading. snapAtEnd should only be true inside the walled maze.
    bool driveDistanceMM(float distanceMM, bool snapAtEnd = false) {
        guardTrip = false;
        if (distanceMM <= 1.0f) {
            if (snapAtEnd) correctPoseAtWall();
            return true;
        }

        // Zero the encoders and the odometry reference together so the first
        // updatePose() sees no displacement and cannot double-count.
        encoder.reset();
        pose.resetEncoderReference();
        updatePose();

        const float startH     = pose.getH();
        const float targetRads = (distanceMM / WHEEL_CIRC_MM) * 2.0f * PI;

        headingPid.zeroAndSetTarget(startH, 0.0f);
        distancePid.zeroAndSetTarget(0.0f, targetRads);

        unsigned long start = millis();
        uint8_t guardTick = 0;
        float rampPwm = MINIMUM_PWM;   // start of the acceleration ramp

        while (true) {
            if (millis() - start > TIMEOUT_MS) {
                Serial.println(F("[TIMEOUT] driveDistanceMM"));
                stop();
                return false;
            }

            updatePose();

            float leftRads   = encoder.getLeftRotation();
            float rightRads  = encoder.getRightRotation();
            //float driveSpeed = distancePid.compute(min(leftRads, rightRads));
            float driveSpeed = distancePid.compute((leftRads + rightRads) / 2.0f);
            float correction = headingPid.compute(pose.getH());

            // Stop at target, and never let the PID reverse us on overshoot.
            if (distancePid.atTarget(DISTANCE_TOLERANCE_RAD) || driveSpeed < 0) break;

            // Apply the speed ceiling, then the acceleration ramp. Taking the
            // smaller of (ramp so far + one step) and (what the PID wants)
            // means power rises gradually from rest but is free to fall the
            // instant the controller decides to slow down.
            driveSpeed = min(driveSpeed, maxDrivePwm);
            rampPwm    = min(rampPwm + PWM_SLEW_UP, driveSpeed);
            driveSpeed = rampPwm;

            // Collision guard (Task 4.2 only)
            if (obstacleGuardEnabled && (++guardTick >= GUARD_PERIOD)) {
                guardTick = 0;
                lidars.scan();
                if (!lidars.hasError() && lidars.getFrontMM() < EMERGENCY_STOP_MM) {
                    Serial.print(F("[GUARD] obstacle at "));
                    Serial.print(lidars.getFrontMM());
                    Serial.println(F(" mm — stopping"));
                    stop();
                    guardTrip = true;
                    return false;
                }
            }

            // Cap correction so the slower motor stays >= MINIMUM_PWM, else a
            // large heading error pivots the robot instead of correcting it.
            float maxCorr = max(0.0f, driveSpeed - MINIMUM_PWM);
            correction    = constrain(correction, -maxCorr, maxCorr);

            float leftPWM  = constrain(driveSpeed - correction, 0.0f, 255.0f);
            float rightPWM = constrain(driveSpeed + correction, 0.0f, 255.0f);
            if (leftPWM  > 0 && leftPWM  < MINIMUM_PWM) leftPWM  = MINIMUM_PWM;
            if (rightPWM > 0 && rightPWM < MINIMUM_PWM) rightPWM = MINIMUM_PWM;
            motorL.setPWM((int)leftPWM);
            motorR.setPWM((int)rightPWM);

            delay(10);
        }

        stop();
        delay(700);
        if (snapAtEnd) correctPoseAtWall();
        return true;
    }

    // Turns on the spot by an arbitrary signed angle (radians, +ve clockwise).
    bool turnByRadians(float deltaH) {
        if (fabs(deltaH) < TURN_TOLERANCE_RAD) return true;
        return turn(deltaH);
    }

    // Turns on the spot to an absolute heading, taking the shorter way round.
    bool turnToHeading(float targetH) {
        return turnByRadians(normaliseAngle(targetH - pose.getH()));
    }

    // Wraps an angle into [-PI, PI].
    static float normaliseAngle(float a) {
        while (a >  PI) a -= 2.0f * PI;
        while (a < -PI) a += 2.0f * PI;
        return a;
    }

    // Moves the micromouse forward one cell, returns true if successful
    bool forwardOneCell() {

        // Reset hardware encoders and odometry reference FIRST so that the
        // subsequent updatePose() reads a zero encoder delta and does not
        // double-count displacement accumulated since the last snapToWall().
        encoder.reset();              // zeroes l_count / r_count in hardware
        pose.resetEncoderReference(); // zeroes lastLPos / lastRPos in odometry

        updatePose();                 // now reads 0 delta → pose x/y/h unchanged
        float x = pose.getX() * 1000.0f;
        float y = pose.getY() * 1000.0f;
        float h = pose.getH();

        // Project position onto heading axis to find how far into the current
        // cell the robot sits, then compute remaining distance to next wall.
        //
        // IMPORTANT: snap h to the nearest cardinal (0, π/2, π, 3π/2) first.
        // The turn PID stops within TURN_TOLERANCE_RAD (≈3°), so h is never
        // exactly cardinal after a turn.  With large x/y coordinates the tiny
        // residual gets multiplied into a big projection error — e.g. a 3° error
        // with y=270mm contributes 270×sin(3°)≈14mm to projectedMM, making
        // remainingMM compute as ~14mm instead of 180mm for the first 'f' after
        // a turn.  Snapping eliminates this without affecting accuracy.
        float hCardinal   = round(h / (float(PI) / 2.0f)) * (float(PI) / 2.0f);
        float projectedMM = x * sin(hCardinal) - y * cos(hCardinal);
        float shiftedMM = projectedMM + (CELL_MM / 2.0f);

        float intoCurrentCell = fmod(shiftedMM, CELL_MM);
        if (intoCurrentCell < 0) {
            intoCurrentCell += CELL_MM;
        }

        float remainingMM = CELL_MM - intoCurrentCell;

        // Safety clamp: if LiDAR snap left us past the midpoint, the fmod
        // can give a tiny (<20 mm) remainder. Add a full cell so we always
        // drive at least 20 mm forward.
        if (remainingMM < 20.0f) remainingMM += CELL_MM;

        // Drive the grid-specific remainder, then snap against the wall.
        return driveDistanceMM(remainingMM, true);
    }

    // Turns the micromouse left 90 degrees, returns true if successful
    bool turnLeft90() {
        return turn(-PI / 2.0f);
    }

    // Turns the micromouse right 90 degrees, returns true if successful
    bool turnRight90() {
        return turn(PI / 2.0f);
    }

    // Stops motors
    void stop() {
        motorL.setPWM(0);
        motorR.setPWM(0);

    }

    void scanLidars() {
        lidars.scan();
    }

    bool hasWallFront() {
        return lidars.hasWallFront();
    }

    bool hasWallLeft() {
        return lidars.hasWallLeft();
    }
    bool hasWallRight() {
        return lidars.hasWallRight();
    }


private:

    bool turn(float deltaH) {
        float startH = pose.getGyroYaw();

        headingPid.zeroAndSetTarget(startH, deltaH);
        encoder.reset();
        pose.resetEncoderReference();

        unsigned long start = millis();
        float rampPwm = MINIMUM_PWM;

        while (true) {
            if (millis() - start > TIMEOUT_MS) {
                Serial.println(F("[TIMEOUT] turn"));
                stop();
                return false;
            }

            updatePose();

            float correction = headingPid.compute(pose.getGyroYaw());

            if (headingPid.atTarget(TURN_TOLERANCE_RAD)) break;

            // Ramp the magnitude, keep the sign. A 90 deg turn asks for ~200
            // PWM on the first step, which jerks the chassis round and upsets
            // the gyro; easing into it settles the heading much faster.
            float magnitude = fabs(correction);
            rampPwm   = min(rampPwm + TURN_SLEW_UP, magnitude);
            magnitude = min(rampPwm, maxDrivePwm);
            float command = addPwmFloor(correction >= 0 ? magnitude : -magnitude);

            motorL.setPWM(-(int)command);
            motorR.setPWM(+(int)command);

            delay(10);
        }
        stop();
        delay(700);

        return true;
    }

    // read encoders and mpu, update pose
    void updatePose() {
        float leftRads = encoder.getLeftRotation();
        float rightRads = encoder.getRightRotation();

        mpu.update();
        float gyroYaw = mpu.getAngleZ();
#if MOTION_DEBUG
        Serial.print(F(" | Gyro Yaw: "));
        Serial.print(gyroYaw);
#endif
        pose.update(leftRads, rightRads, gyroYaw);
    }

    // Scans wall and corrects pose, called after maze step executed
    void correctPoseAtWall() {
        if (!lidarCorrectionEnabled) return;
        for (int i = 0; i < mtrn3100::Lidars::BUFFER_SIZE; i++) {
            lidars.scan();
        }
        pose.snapToWall(lidars.getFrontMM(), lidars.getLeftMM(), lidars.getRightMM(),
                            lidars.hasWallFront(), lidars.hasWallLeft(), lidars.hasWallRight());
    }

    // ensures drive Speed is above minimum pwm
    float addPwmFloor(float driveSpeed) {
        if (fabs(driveSpeed) < DEADBAND) {return 0.0;}
        if (driveSpeed > 0 && driveSpeed < MINIMUM_PWM) {driveSpeed = MINIMUM_PWM;}
        if (driveSpeed < 0 && driveSpeed > -MINIMUM_PWM) {driveSpeed = -MINIMUM_PWM;}
        return driveSpeed;
    }

    bool lidarCorrectionEnabled = true;
    bool obstacleGuardEnabled   = false;  // Task 4.2 only
    bool guardTrip              = false;
    float maxDrivePwm           = DEFAULT_MAX_PWM;

    // Declaration order must match the constructor's initialiser list.
    Pose& pose;
    Lidars& lidars;
    MPU6050& mpu;
    Motor& motorL;
    Motor& motorR;
    DualEncoder& encoder;
    PIDController& headingPid;
    PIDController& distancePid;
};

}