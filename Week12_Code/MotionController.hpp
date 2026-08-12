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
    static constexpr float TURN_TOLERANCE_RAD = 0.03f; // TUNE THIS
    static constexpr float DISTANCE_TOLERANCE_RAD = 0.05f; // TUNE THIS

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
            
            float driveSpeed = distancePid.compute(min(leftRads, rightRads));
            float correction = headingPid.compute(pose.getH());
            
            if (distancePid.atTarget(DISTANCE_TOLERANCE_RADS)) break;

            motorL.setPWM((int)(driveSpeed + correction));
            motorR.setPWM((int)(driveSpeed - correction));

            delay(10);
        }

        stop();
        delay(200);
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
        float startH = pose.getH();
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

            float correction = headingPid.compute(pose.getH());

            if (headingPid.atTarget(TURN_TOLERANCE_RAD)) break;

            motorL.setPWM((int)correction);
            motorR.setPWM(-(int)correction);

            delay(10);
        }

        return true;
    }

    // read encoders and mpu, update pose
    void updatePose() {
        float leftRads = encoder.getLeftRotation();
        float rightRads = encoder.getRightRotation();

        mpu.update();
        float gyroYaw = mpu.getAngleZ();
        pose.update(leftRads, rightRads, gyroYaw);
    }

    // Scans wall and corrects pose, called after maze step executed
    void correctPoseAtWall() {
        lidars.scan();
        pose.snapToWall(lidars.getFrontMM(), lidars.getLeftMM(), lidars.getRightMM(),
                            lidars.hasWallFront(), lidars.hasWallLeft(), lidars.hasWallRight());
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