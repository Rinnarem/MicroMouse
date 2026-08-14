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

    MotionController(Pose& pose, Lidars& lidars, MPU6050& imu, Motor& motorL, Motor& motorR,
                        DualEncoder& encoder, PIDController& headingPid, 
                        PIDController& distancePid)
        : pose(pose), lidars(lidars), mpu(imu), motorL(motorL), motorR(motorR),
        encoder(encoder), headingPid(headingPid), distancePid(distancePid) {}

    // Moves the micromouse forward one cell, returns true if successful
    bool forwardOneCell() {
        
        float targetRads = (CELL_MM / WHEEL_CIRC_MM) * 2.0f * PI;
        encoder.reset();
        float startH = pose.getH();
        headingPid.zeroAndSetTarget(startH, 0.0f);
        distancePid.zeroAndSetTarget(0.0f, targetRads);

        unsigned long start = millis();

        while (true) {

            if (millis() - start > TIMEOUT_MS) {
                Serial.println("[TIMEOUT] forwardOneCell");
                stop();
                return false;
            }

            updatePose();

            // Both wheels must reach target before breaking
            float leftRads = encoder.getLeftRotation();
            float rightRads = encoder.getRightRotation();
            float driveSpeed = distancePid.compute(min(leftRads, rightRads));
            float correction = headingPid.compute(pose.getH());
            Serial.print(" | Correction: ");
            Serial.print(correction);
            Serial.print(" | driveSpeed: ");
            Serial.print(driveSpeed);

            if (distancePid.atTarget(DISTANCE_TOLERANCE_RAD)) break;

            motorL.setPWM((int)addPwmFloor(driveSpeed - correction));
            motorR.setPWM((int)addPwmFloor(driveSpeed + correction));

            delay(10);
        }

        stop();
        delay(700);
        correctPoseAtWall();

        return true;
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


private:

    bool turn(float deltaH) {
        float startH = pose.getGyroYaw();
        float targetH = startH + deltaH;

        headingPid.zeroAndSetTarget(startH, deltaH);

        unsigned long start = millis();

        while (true) {
            if (millis() - start > TIMEOUT_MS) {
                Serial.println("[TIMEOUT] forwardOneCell");
                stop();
                return false;  
            }

            updatePose();

            float correction = headingPid.compute(pose.getGyroYaw());

            if (headingPid.atTarget(TURN_TOLERANCE_RAD)) break;

            motorL.setPWM(-(int)addPwmFloor(correction));
            motorR.setPWM(+(int)addPwmFloor(correction));

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
        Serial.print(" | Gyro Yaw: ");
        Serial.print(gyroYaw);
        pose.update(leftRads, rightRads, gyroYaw);

    }

    // Scans wall and corrects pose, called after maze step executed
    void correctPoseAtWall() {
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

    Pose& pose;
    Lidars& lidars;
    MPU6050& mpu;
    Motor& motorL;
    Motor& motorR;
    PIDController& headingPid;
    PIDController& distancePid;
    DualEncoder& encoder;
};

}