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

#ifndef MOTION_DEBUG
#define MOTION_DEBUG 0
#endif

namespace mtrn3100 {

class MotionController {

public:
    static constexpr float CELL_MM = 180.0f;
    static constexpr float WHEEL_CIRC_MM = 2.0f * PI * 16.0f;
    static constexpr float TIMEOUT_MS = 8000.0f;

    // Expected front-LiDAR reading when centred in a cell with a wall ahead.
    static constexpr float FRONT_CENTRE_MM = 35.0f;

    // Task 4.1 front-wall protection.
    static constexpr uint8_t MAZE_FRONT_PERIOD = 1;
    static constexpr float MAZE_FRONT_ARM_FRACTION = 0.60f;
    static constexpr float MAZE_FRONT_ACCEPT_FRACTION = 0.90f;
    static constexpr float MAZE_FRONT_SLOW_MM = 80.0f;
    static constexpr float MAZE_FRONT_SLOW_PWM = 85.0f;
    static constexpr float MAZE_FRONT_FINE_PWM = 42.0f;
    static constexpr float MAZE_FRONT_TARGET_TOLERANCE_MM = 5.0f;
    static constexpr float MAZE_FRONT_ABSOLUTE_MIN_MM = 8.0f;

    // Task 4.1 side-wall protection.
    static constexpr uint8_t MAZE_SIDE_PERIOD = 1;
    static constexpr float SIDE_DANGER_MM = 23.0f;
    static constexpr float SIDE_RELEASE_MM = 26.0f;
    static constexpr float SIDE_GUARD_PWM = 22.0f;

    static constexpr float TURN_TOLERANCE_RAD = 0.02f;
    static constexpr float TURN_FINE_RAD = 0.12f;
    static constexpr float TURN_FINE_PWM = 42.0f;
    static constexpr float DISTANCE_TOLERANCE_RAD = 0.025f;
    static constexpr float DEADBAND = 0.0f;
    static constexpr float MINIMUM_PWM = 55.0f;

    // Task 4.2 obstacle-course collision guard.
    static constexpr float EMERGENCY_STOP_MM = 55.0f;
    static constexpr uint8_t GUARD_PERIOD = 6;

    // Acceleration limiting.
    static constexpr float PWM_SLEW_UP = 6.0f;
    static constexpr float TURN_SLEW_UP = 10.0f;
    static constexpr float DEFAULT_MAX_PWM = 255.0f;

    MotionController(
        Pose& pose,
        Lidars& lidars,
        MPU6050& imu,
        Motor& motorL,
        Motor& motorR,
        DualEncoder& encoder,
        PIDController& headingPid,
        PIDController& distancePid
    )
        : pose(pose),
          lidars(lidars),
          mpu(imu),
          motorL(motorL),
          motorR(motorR),
          encoder(encoder),
          headingPid(headingPid),
          distancePid(distancePid) {}

    void setLidarCorrection(bool enabled) {
        lidarCorrectionEnabled = enabled;
    }

    // Wait while continuing to update the IMU and pose.
    void idle(unsigned long ms) {
        unsigned long start = millis();

        while (millis() - start < ms) {
            updatePose();
            delay(5);
        }
    }

    // Task 4.2 front-LiDAR collision guard.
    void setObstacleGuard(bool enabled) {
        obstacleGuardEnabled = enabled;
    }

    void setSpeedLimit(float maxPwm) {
        maxDrivePwm = constrain(
            maxPwm,
            MINIMUM_PWM,
            255.0f
        );
    }

    bool guardTripped() const {
        return guardTrip;
    }

    // Drive an arbitrary forward distance while holding the initial heading.
    bool driveDistanceMM(
        float distanceMM,
        bool snapAtEnd = false
    ) {
        guardTrip = false;

        if (distanceMM <= 1.0f) {
            if (snapAtEnd) {
                correctPoseAtWall();
            }

            return true;
        }

        // Reset the hardware encoders and pose encoder reference together.
        encoder.reset();
        pose.resetEncoderReference();
        updatePose();

        const float startH = pose.getH();

        const float targetRads =
            (distanceMM / WHEEL_CIRC_MM) *
            2.0f *
            PI;

        headingPid.zeroAndSetTarget(startH, 0.0f);
        distancePid.zeroAndSetTarget(0.0f, targetRads);

        unsigned long start = millis();

        uint8_t frontTick = 0;
        uint8_t obstacleTick = 0;
        uint8_t sideTick = 0;

        uint8_t leftDangerHits = 0;
        uint8_t rightDangerHits = 0;

        bool leftSideActive = false;
        bool rightSideActive = false;
        bool frontSlowReported = false;
        bool frontApproachActive = false;

        float sideCorrection = 0.0f;
        float rampPwm = MINIMUM_PWM;

        while (true) {
            if (millis() - start > TIMEOUT_MS) {
                Serial.println(F("[TIMEOUT] driveDistanceMM"));
                stop();
                return false;
            }

            updatePose();

            float leftRads = encoder.getLeftRotation();
            float rightRads = encoder.getRightRotation();

            float averageRads =
                (leftRads + rightRads) /
                2.0f;

            float driveSpeed =
                distancePid.compute(averageRads);

            float correction =
                headingPid.compute(pose.getH());

            // Stop at the encoder target and never reverse after overshoot.
            if (
                distancePid.atTarget(DISTANCE_TOLERANCE_RAD) ||
                driveSpeed < 0.0f
            ) {
                break;
            }

            driveSpeed = min(
                driveSpeed,
                maxDrivePwm
            );

            rampPwm = min(
                rampPwm + PWM_SLEW_UP,
                driveSpeed
            );

            driveSpeed = rampPwm;

            // ---------------------------------------------------------
            // Task 4.1 side-clearance guard
            // ---------------------------------------------------------
            //
            // The IMU remains the main heading controller. LiDAR only
            // adds a bounded correction when the robot gets dangerously
            // close to a side wall.
            if (
                wallStopEnabled &&
                (++sideTick >= MAZE_SIDE_PERIOD)
            ) {
                sideTick = 0;

                float leftMM = 255.0f;
                float rightMM = 255.0f;

                if (lidars.scanSidesMM(leftMM, rightMM)) {
                    const bool leftDanger =
                        leftMM <= SIDE_DANGER_MM;

                    const bool rightDanger =
                        rightMM <= SIDE_DANGER_MM;

                    // Require two consecutive close readings before
                    // activating the left-wall correction.
                    if (!leftSideActive) {
                        if (leftDanger) {
                            if (leftDangerHits < 2) {
                                leftDangerHits++;
                            }
                        } else {
                            leftDangerHits = 0;
                        }

                        if (leftDangerHits >= 2) {
                            leftSideActive = true;

                            Serial.print(
                                F("[SIDE] LEFT close at ")
                            );

                            Serial.print(leftMM);

                            Serial.println(
                                F(" mm - steering right")
                            );
                        }
                    } else if (leftMM >= SIDE_RELEASE_MM) {
                        leftSideActive = false;
                        leftDangerHits = 0;

                        Serial.println(
                            F("[SIDE] LEFT clear")
                        );
                    }

                    // Require two consecutive close readings before
                    // activating the right-wall correction.
                    if (!rightSideActive) {
                        if (rightDanger) {
                            if (rightDangerHits < 2) {
                                rightDangerHits++;
                            }
                        } else {
                            rightDangerHits = 0;
                        }

                        if (rightDangerHits >= 2) {
                            rightSideActive = true;

                            Serial.print(
                                F("[SIDE] RIGHT close at ")
                            );

                            Serial.print(rightMM);

                            Serial.println(
                                F(" mm - steering left")
                            );
                        }
                    } else if (rightMM >= SIDE_RELEASE_MM) {
                        rightSideActive = false;
                        rightDangerHits = 0;

                        Serial.println(
                            F("[SIDE] RIGHT clear")
                        );
                    }

                    sideCorrection = 0.0f;

                    // With the current motor equations:
                    // positive correction steers right;
                    // negative correction steers left.
                    if (
                        leftSideActive &&
                        !rightSideActive
                    ) {
                        sideCorrection =
                            +SIDE_GUARD_PWM;
                    } else if (
                        rightSideActive &&
                        !leftSideActive
                    ) {
                        sideCorrection =
                            -SIDE_GUARD_PWM;
                    } else if (
                        leftSideActive &&
                        rightSideActive
                    ) {
                        // Move away from whichever wall is closer.
                        sideCorrection =
                            (leftMM < rightMM)
                                ? +SIDE_GUARD_PWM
                                : -SIDE_GUARD_PWM;
                    }
                } else {
                    // Do not steer from timed-out measurements.
                    sideCorrection = 0.0f;
                }
            }

            // Add the side correction exactly once.
            correction += sideCorrection;

            // ---------------------------------------------------------
            // Task 4.2 cylinder collision guard
            // ---------------------------------------------------------
            if (
                obstacleGuardEnabled &&
                (++obstacleTick >= GUARD_PERIOD)
            ) {
                obstacleTick = 0;
                lidars.scan();

                if (
                    !lidars.hasError() &&
                    lidars.getFrontMM() <
                        EMERGENCY_STOP_MM
                ) {
                    Serial.print(
                        F("[GUARD] obstacle at ")
                    );

                    Serial.print(
                        lidars.getFrontMM()
                    );

                    Serial.println(
                        F(" mm - stopping")
                    );

                    stop();
                    guardTrip = true;
                    return false;
                }
            }

            // ---------------------------------------------------------
            // Task 4.1 front-wall guard
            // ---------------------------------------------------------
            //
            // Progressively slow down near the expected far wall. Once the
            // robot has completed at least 90% of the encoder move and the
            // front reading is within 5 mm of the cell-centre reading, this f
            // has succeeded. Breaking here continues the command chain; it is
            // not an error stop.
            if (
                wallStopEnabled &&
                (++frontTick >= MAZE_FRONT_PERIOD)
            ) {
                frontTick = 0;

                float travelledMM =
                    0.5f *
                    (leftRads + rightRads) *
                    (
                        WHEEL_CIRC_MM /
                        (2.0f * PI)
                    );

                if (
                    travelledMM >
                    MAZE_FRONT_ARM_FRACTION *
                    distanceMM
                ) {
                    float frontMM =
                        lidars.scanFrontMM();

                    if (frontMM < 255.0f) {
                        const float frontTargetMM =
                            FRONT_CENTRE_MM +
                            MAZE_FRONT_TARGET_TOLERANCE_MM;

                        // This should never be reached during a valid, tuned
                        // move. Keep it only as final collision protection for
                        // an incorrect path or a badly misplaced robot.
                        if (
                            frontMM <=
                            MAZE_FRONT_ABSOLUTE_MIN_MM
                        ) {
                            Serial.print(
                                F("[FRONT] absolute safety stop at ")
                            );

                            Serial.print(frontMM);

                            Serial.println(F(" mm"));

                            stop();
                            return false;
                        }

                        if (
                            frontMM <=
                            MAZE_FRONT_SLOW_MM
                        ) {
                            frontApproachActive = true;

                            // Interpolate the speed ceiling from 85 PWM at
                            // 80 mm down to 42 PWM near the centre target.
                            float approachFraction =
                                (frontMM - frontTargetMM) /
                                (MAZE_FRONT_SLOW_MM - frontTargetMM);

                            approachFraction = constrain(
                                approachFraction,
                                0.0f,
                                1.0f
                            );

                            float frontPwmLimit =
                                MAZE_FRONT_FINE_PWM +
                                approachFraction *
                                (MAZE_FRONT_SLOW_PWM - MAZE_FRONT_FINE_PWM);

                            driveSpeed = min(
                                driveSpeed,
                                frontPwmLimit
                            );

                            rampPwm = min(
                                rampPwm,
                                driveSpeed
                            );

                            if (!frontSlowReported) {
                                Serial.print(
                                    F("[FRONT] slowing at ")
                                );

                                Serial.print(frontMM);

                                Serial.println(F(" mm"));

                                frontSlowReported = true;
                            }
                        }

                        if (
                            travelledMM >=
                                MAZE_FRONT_ACCEPT_FRACTION * distanceMM &&
                            frontMM <= frontTargetMM
                        ) {
                            Serial.print(
                                F("[FRONT] cell centre reached at ")
                            );

                            Serial.print(frontMM);
                            Serial.println(F(" mm"));

                            break;
                        }
                    }
                }
            }

            // Prevent a large correction from turning the forward
            // movement into an on-the-spot pivot.
            // During the final front-wall approach, allow the motors to use
            // the same 42-PWM fine-control floor already proven by the turn
            // controller. Otherwise the normal 55-PWM floor would override
            // the progressive slowdown above.
            float activeMinimumPwm =
                frontApproachActive
                    ? MAZE_FRONT_FINE_PWM
                    : MINIMUM_PWM;

            float maxCorr = max(
                0.0f,
                driveSpeed - activeMinimumPwm
            );

            correction = constrain(
                correction,
                -maxCorr,
                maxCorr
            );

            float leftPWM = constrain(
                driveSpeed - correction,
                0.0f,
                255.0f
            );

            float rightPWM = constrain(
                driveSpeed + correction,
                0.0f,
                255.0f
            );

            if (
                leftPWM > 0.0f &&
                leftPWM < activeMinimumPwm
            ) {
                leftPWM = activeMinimumPwm;
            }

            if (
                rightPWM > 0.0f &&
                rightPWM < activeMinimumPwm
            ) {
                rightPWM = activeMinimumPwm;
            }

            motorL.setPWM((int)leftPWM);
            motorR.setPWM((int)rightPWM);

            delay(10);
        }

        stop();
        idle(700);

        if (snapAtEnd) {
            correctPoseAtWall();
        }

        return true;
    }

    // Turn by a signed angle. Positive is clockwise.
    bool turnByRadians(float deltaH) {
        if (
            fabs(deltaH) <
            TURN_TOLERANCE_RAD
        ) {
            return true;
        }

        return turn(deltaH);
    }

    // Turn to an absolute heading using the shorter direction.
    bool turnToHeading(float targetH) {
        return turnByRadians(
            normaliseAngle(
                targetH - pose.getH()
            )
        );
    }

    static float normaliseAngle(float angle) {
        while (angle > PI) {
            angle -= 2.0f * PI;
        }

        while (angle < -PI) {
            angle += 2.0f * PI;
        }

        return angle;
    }

    // One grid f command always requests exactly 180 mm.
    bool forwardOneCell() {
        wallStopEnabled =
            lidarCorrectionEnabled;

        bool moved =
            driveDistanceMM(CELL_MM, true);

        wallStopEnabled = false;

        return moved;
    }

    bool turnLeft90() {
        return turn(-PI / 2.0f);
    }

    bool turnRight90() {
        return turn(PI / 2.0f);
    }

    void stop() {
        motorL.setPWM(0);
        motorR.setPWM(0);
    }

private:
    bool turn(float deltaH) {
        encoder.reset();
        pose.resetEncoderReference();

        updatePose();

        float startH =
            pose.getGyroYaw();

        headingPid.zeroAndSetTarget(
            startH,
            deltaH
        );

        unsigned long start = millis();
        float rampPwm = MINIMUM_PWM;

        while (true) {
            if (
                millis() - start >
                TIMEOUT_MS
            ) {
                Serial.println(F("[TIMEOUT] turn"));
                stop();
                return false;
            }

            updatePose();

            float correction =
                headingPid.compute(
                    pose.getGyroYaw()
                );

            if (
                headingPid.atTarget(
                    TURN_TOLERANCE_RAD
                )
            ) {
                break;
            }

            float magnitude =
                fabs(correction);

            rampPwm = min(
                rampPwm + TURN_SLEW_UP,
                magnitude
            );

            magnitude = min(
                rampPwm,
                maxDrivePwm
            );

            float floorPwm =
                (
                    fabs(headingPid.getError()) <
                    TURN_FINE_RAD
                )
                    ? TURN_FINE_PWM
                    : MINIMUM_PWM;

            if (magnitude < floorPwm) {
                magnitude = floorPwm;
            }

            float command =
                (correction >= 0.0f)
                    ? magnitude
                    : -magnitude;

            motorL.setPWM(
                -(int)command
            );

            motorR.setPWM(
                +(int)command
            );

            delay(10);
        }

        stop();
        idle(700);

        return true;
    }

    void updatePose() {
        float leftRads =
            encoder.getLeftRotation();

        float rightRads =
            encoder.getRightRotation();

        mpu.update();

        float gyroYaw =
            mpu.getAngleZ();

#if MOTION_DEBUG
        Serial.print(F(" | Gyro Yaw: "));
        Serial.print(gyroYaw);
#endif

        pose.update(
            leftRads,
            rightRads,
            gyroYaw
        );
    }

    // Read the walls and correct the estimated pose after a grid move.
    void correctPoseAtWall() {
        if (!lidarCorrectionEnabled) {
            return;
        }

        for (
            int i = 0;
            i < mtrn3100::Lidars::BUFFER_SIZE;
            i++
        ) {
            lidars.scan();
        }

        lidars.report();

        float beforeX =
            pose.getX() * 1000.0f;

        float beforeY =
            pose.getY() * 1000.0f;

        float beforeH =
            pose.getH();

        pose.snapToWall(
            lidars.getFrontMM(),
            lidars.getLeftMM(),
            lidars.getRightMM(),
            lidars.hasWallFront(),
            lidars.hasWallLeft(),
            lidars.hasWallRight()
        );

        float deltaX =
            pose.getX() * 1000.0f -
            beforeX;

        float deltaY =
            pose.getY() * 1000.0f -
            beforeY;

        float deltaH =
            degrees(
                pose.getH() -
                beforeH
            );

        if (
            fabs(deltaX) > 0.5f ||
            fabs(deltaY) > 0.5f ||
            fabs(deltaH) > 0.2f
        ) {
            Serial.print(F("  [SNAP] dx="));
            Serial.print(deltaX, 1);

            Serial.print(F(" dy="));
            Serial.print(deltaY, 1);

            Serial.print(F(" dh="));
            Serial.print(deltaH, 1);

            Serial.println(F(" deg"));
        } else {
            Serial.println(
                F("  [SNAP] no correction applied")
            );
        }

        encoder.reset();
        pose.resetEncoderReference();
    }

    float addPwmFloor(float driveSpeed) {
        if (
            fabs(driveSpeed) <
            DEADBAND
        ) {
            return 0.0f;
        }

        if (
            driveSpeed > 0.0f &&
            driveSpeed < MINIMUM_PWM
        ) {
            driveSpeed = MINIMUM_PWM;
        }

        if (
            driveSpeed < 0.0f &&
            driveSpeed > -MINIMUM_PWM
        ) {
            driveSpeed = -MINIMUM_PWM;
        }

        return driveSpeed;
    }

    bool lidarCorrectionEnabled = true;
    bool obstacleGuardEnabled = false;
    bool wallStopEnabled = false;
    bool guardTrip = false;

    float maxDrivePwm =
        DEFAULT_MAX_PWM;

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
