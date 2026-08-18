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

    // One cell takes about a second. 28 s meant a runaway spun for nearly half
    // a minute before giving up.
    static constexpr float TIMEOUT_MS = 8000.0f;


    // What the front LiDAR reads when the robot is centred in a cell with a
    // wall ahead: centre is 90 mm from the wall, sensor sits 55 mm forward.
    static constexpr float FRONT_CENTRE_MM = 35.0f;

    // Task 4.1 maze safety. Front and side guards are deliberately separate
    // from the Task 4.2 cylinder guard below.
    static constexpr uint8_t MAZE_FRONT_PERIOD = 2;
    static constexpr float MAZE_FRONT_ARM_FRACTION = 0.60f;
    static constexpr float MAZE_FRONT_SLOW_MM = 80.0f;
    static constexpr float MAZE_FRONT_STOP_MM = 50.0f;
    static constexpr float MAZE_FRONT_HARD_STOP_MM = 25.0f;
    static constexpr float MAZE_FRONT_SLOW_PWM = 85.0f;
    static constexpr uint8_t MAZE_SIDE_PERIOD = 5;
    static constexpr float SIDE_DANGER_MM = 15.0f;
    static constexpr float SIDE_GUARD_PWM = 28.0f;

    // 0.05 rad is 2.9 deg per turn, and it compounds. 0.03 is 1.7 deg.
    static constexpr float TURN_TOLERANCE_RAD = 0.02f;
    // Near the target the 55 PWM floor keeps commanding a full kick for a 1 deg
    // error, so the turn hunts either side instead of settling.
    static constexpr float TURN_FINE_RAD = 0.12f;    // ~7 deg
    static constexpr float TURN_FINE_PWM = 42.0f;
    static constexpr float DISTANCE_TOLERANCE_RAD = 0.025f;
    static constexpr float DEADBAND = 0.0f;
    static constexpr float MINIMUM_PWM = 55.0f;

    // --- Task 4.2 continuous section -------------------------------------
    // Inside the obstacle course there are no walls to snap against, but the
    // front LiDAR is still useful as a last-resort collision guard against a
    // cylinder. Scanning is slow, so it is polled every GUARD_PERIOD loops.
    static constexpr float   EMERGENCY_STOP_MM = 55.0f;
    static constexpr uint8_t GUARD_PERIOD      = 6;     // loops (~70 ms)

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

    // Wait, but keep the IMU integrating.
    //
    // MPU6050_light::update() integrates gyro rate over the time since it was
    // last called, and it is only called from updatePose() -- i.e. only while
    // moving. A bare delay() therefore leaves a gap that the next update()
    // applies as one lump of drift. The controller reads that as a sudden
    // heading error and turns to correct a rotation that never happened.
    //
    // Every wait on the motion path goes through here.
    void idle(unsigned long ms) {
        unsigned long t0 = millis();
        while (millis() - t0 < ms) {
            updatePose();
            delay(5);
        }
    }

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
        uint8_t frontTick = 0, obstacleTick = 0, sideTick = 0;
        uint8_t wallHits = 0, leftDangerHits = 0, rightDangerHits = 0;
        bool frontSlowReported = false;
        float sideCorrection = 0.0f;
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

            // --- walled maze: stop when the wall ahead says we are centred ---
            // Odometry alone cannot tell how far the wall really is, so a
            // direct reading bounds the error that would otherwise become a
            // collision. Only after half the move, so a robot placed close to a
            // wall does not abort instantly while MazeNavigator still counts
            // the cell. Two consecutive readings are required, because a single
            // spurious short reading would stop the move dead.
            if (wallStopEnabled && (++frontTick >= MAZE_FRONT_PERIOD)) {
                frontTick = 0;
                float travelledMM = 0.5f * (leftRads + rightRads) * (WHEEL_CIRC_MM / (2.0f * PI));
                if (travelledMM > MAZE_FRONT_ARM_FRACTION * distanceMM) {
                    float fmm = lidars.scanFrontMM();
                    if (fmm < 255.0f) {
                        if (fmm <= MAZE_FRONT_SLOW_MM) {
                            driveSpeed = min(driveSpeed, MAZE_FRONT_SLOW_PWM);
                            rampPwm = min(rampPwm, driveSpeed);
                            if (!frontSlowReported) {
                                Serial.print(F("[FRONT] slowing at "));
                                Serial.print(fmm); Serial.println(F(" mm"));
                                frontSlowReported = true;
                            }
                        }

                        if (fmm <= MAZE_FRONT_HARD_STOP_MM) {
                            Serial.print(F("[WALL] HARD stop at "));
                            Serial.print(fmm); Serial.println(F(" mm"));
                            break;
                        }

                        if (fmm <= MAZE_FRONT_STOP_MM) {
                            if (wallHits < 2) wallHits++;
                            if (wallHits >= 2) {
                                Serial.print(F("[WALL] stop at "));
                                Serial.print(fmm); Serial.println(F(" mm"));
                                break;
                            }
                        } else {
                            wallHits = 0;
                        }
                    }
                }
            }

            // Collision guard (Task 4.2 only)
            if (obstacleGuardEnabled && (++obstacleTick >= GUARD_PERIOD)) {
                obstacleTick = 0;
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

            // Task 4.1 side-clearance guard. IMU heading remains the main
            // controller; LiDAR only adds a bounded correction when one side
            // is repeatedly very close to a wall. Positive correction steers
            // right with the motor equations below.
            if (wallStopEnabled && (++sideTick >= MAZE_SIDE_PERIOD)) {
                sideTick = 0;
                float leftMM = 255.0f, rightMM = 255.0f;

                if (lidars.scanSidesMM(leftMM, rightMM)) {
                    bool leftDanger = leftMM > 10.0f && leftMM < SIDE_DANGER_MM;
                    bool rightDanger = rightMM > 10.0f && rightMM < SIDE_DANGER_MM;

                    if (leftDanger) {
                        if (leftDangerHits < 2) leftDangerHits++;
                    } else {
                        leftDangerHits = 0;
                    }

                    if (rightDanger) {
                        if (rightDangerHits < 2) rightDangerHits++;
                    } else {
                        rightDangerHits = 0;
                    }

                    sideCorrection = 0.0f;
                    if (leftDangerHits >= 2 && rightDangerHits < 2) {
                        sideCorrection = +SIDE_GUARD_PWM;
                    } else if (rightDangerHits >= 2 && leftDangerHits < 2) {
                        sideCorrection = -SIDE_GUARD_PWM;
                    }
                } else {
                    sideCorrection = 0.0f;
                }
            }

            correction += sideCorrection;

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
        idle(700);                     // settle, gyro still integrating
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
    // One grid 'f' = one centre-to-centre cell transition, always CELL_MM.
    //
    // This used to derive the distance by projecting the pose onto the travel
    // axis and driving "the remainder to the next centre". That only works if
    // the along-track coordinate is trustworthy, and it never really is:
    //
    //   - after a turn the axes swap, so the previously-lateral coordinate
    //     (uncorrected, drifting) becomes along-track. 30 mm of sideways drift
    //     turned the next 'f' into 150 mm. This is why the failure always
    //     appeared right after an 'l' or 'r'.
    //   - even going straight, snapToWall only re-seats it when a wall is in
    //     range, and Ed #51 notes smooth vs frosted walls read up to 22 mm
    //     apart, while Ed #63 confirms the marking maze has stretches with no
    //     walls at all.
    //
    // So the pose is not a sound basis for the distance. Command exactly one
    // cell and let the systems that do measure reality do the correcting:
    // the encoder PID for distance, the IMU for heading, the front LiDAR
    // in-loop stop for the wall ahead, and snapToWall afterwards.
    bool forwardOneCell() {
        // The wall stop is only meaningful in the walled maze -- in the
        // obstacle course the thing ahead would be a cylinder.
        wallStopEnabled = lidarCorrectionEnabled;
        bool moved = driveDistanceMM(CELL_MM, true);
        wallStopEnabled = false;
        return moved;
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
        // Order matters. Zero the encoder reference FIRST: snapToWall() ends
        // with odometry.reset(), which zeroes lastLPos/lastRPos while the
        // hardware counters still hold the last move. An updatePose() in that
        // state would read the whole previous move as fresh displacement and
        // teleport the pose a cell forward.
        encoder.reset();
        pose.resetEncoderReference();

        // Now refresh the IMU before sampling the reference angle. getGyroYaw()
        // only reads the library's stored value; without an update() first it
        // is stale by however long the last wait was, and the first in-loop
        // update then jumps -- so the turn starts from a wrong error.
        updatePose();
        float startH = pose.getGyroYaw();

        headingPid.zeroAndSetTarget(startH, deltaH);

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
            // Ease the floor close in so the turn can settle rather than
            // hunting either side of the target.
            float floorPwm = (fabs(headingPid.getError()) < TURN_FINE_RAD)
                             ? TURN_FINE_PWM : MINIMUM_PWM;
            if (magnitude < floorPwm) magnitude = floorPwm;
            float command = (correction >= 0) ? magnitude : -magnitude;

            motorL.setPWM(-(int)command);
            motorR.setPWM(+(int)command);

            delay(10);
        }
        stop();
        idle(700);                     // settle, gyro still integrating

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

        // Before/after diff (dx/dy/dh) and its Serial output are a tuning
        // aid for confirming snapToWall() is doing something sane -- gated
        // behind MOTION_DEBUG since it's not just strings: the subtraction,
        // fabs/degrees calls, and branch all cost flash whether or not
        // anyone reads Serial during a run.
#if MOTION_DEBUG
        lidars.report();
        float bx = pose.getX() * 1000.0f, by = pose.getY() * 1000.0f;
        float bh = pose.getH();
#endif
        // Argument order is (front, left, right, hasFront, hasLeft, hasRight).
        pose.snapToWall(lidars.getFrontMM(), lidars.getLeftMM(), lidars.getRightMM(),
                        lidars.hasWallFront(), lidars.hasWallLeft(),
                        lidars.hasWallRight());

#if MOTION_DEBUG
        float dx = pose.getX() * 1000.0f - bx;
        float dy = pose.getY() * 1000.0f - by;
        float dh = degrees(pose.getH() - bh);
        if (fabs(dx) > 0.5f || fabs(dy) > 0.5f || fabs(dh) > 0.2f) {
            Serial.print(F("  [SNAP] dx=")); Serial.print(dx, 1);
            Serial.print(F(" dy="));         Serial.print(dy, 1);
            Serial.print(F(" dh="));         Serial.print(dh, 1);
            Serial.println(F(" deg"));
        } else {
            Serial.println(F("  [SNAP] no correction applied"));
        }
#endif

        // snapToWall() ends with odometry.reset(), which zeroes the encoder
        // reference but NOT the hardware counters. Re-sync them so the next
        // updatePose() cannot mistake the last move for new displacement.
        encoder.reset();
        pose.resetEncoderReference();
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
    bool wallStopEnabled        = false;  // set per-move by forwardOneCell()
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