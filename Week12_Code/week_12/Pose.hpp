/*
* Pose class for Week 12 Assessment
* Uses encoder odometry, imu and lidars to estimate pose of the micromouse
*/
#pragma once

#include <Arduino.h>
#include "EncoderOdometry.hpp"
#include <MPU6050_light.h>

namespace mtrn3100 {

class Pose {

public: 

    static constexpr float IMU_CW_SIGN = -1.0f;
    static constexpr float WHEEL_RADIUS_M  = 0.016f;
    static constexpr float  AXLE_LENGTH_M   = 0.092f;
    static constexpr float FRONT_CENTRE_OFFSET_MM = 90.0f; // TODO: measure front lidar with robot in centre and update value
    static constexpr float ROBOT_WIDTH_MM = 92.0f; //TODO : Determine empirically
    static constexpr float CELL_MM = 180.0f;

    static constexpr float ALPHA = 1.0f; // for complementary filter
    static constexpr float LIDAR_TRUST = 0.67f; 
    static constexpr float LATERAL_TOLERANCE = 10.0f;

    Pose(MPU6050& imu, float startX = 0, float startY = 0, float startH = 0) : mpu(imu), x(startX),
            y(startY), h(startH), odometry(WHEEL_RADIUS_M, AXLE_LENGTH_M) {}

    // updates the estimated pose of the micromouse using encoder odometry and imu and using a complementary filter
    void update(float leftRads, float rightRads, float gyroYaw) {
        
        // update x and y coordinates using wheel odometry
        odometry.update(leftRads, rightRads);
        x = odometry.getX();
        y = odometry.getY();

        // get change in heading from gyro
        float gyroYawRad = radians(gyroYaw) * IMU_CW_SIGN;
        float gyroDelta = gyroYawRad - prevGyroYawRad;
        prevGyroYawRad = gyroYawRad;

        // complementary filter: corrects drift from gyro with heading from wheel encoders
        // note: wheel encoders more accurate long term, gyro more accurate short term
        h = ALPHA * (h + gyroDelta) + (1 - ALPHA) * odometry.getH();

        float hDeg = h * (180 / M_PI);
        float gyroDeltaDeg = gyroDelta * (180 / M_PI);

        Serial.print("h: ");
        Serial.print(hDeg);

        Serial.print("|  Gryo Delta: ");
        Serial.println(gyroDeltaDeg, 4);

        return;
    }

    /* Call at end of movement sequence (going forward a cell or turning)
    * Uses the lidar to estimate the position of the micromouse and adjust the pose based off that reading
    * Accounts for accumulated drift. Has a lidar trust correction factor which minimises noisy readings corrupting pose
    */
    void snapToWall(float frontMM, float leftMM, float rightMM, bool hasFront, 
                        bool hasRight, bool hasLeft) {
        // get current cardinal ( 0 = N,1 = E,2 = S,3 = W) heading and index
        float cardinalH = round(h / (PI / 2.0f)) * (PI/ 2.0f);
        int cardinalIndex = ((int)round(cardinalH / (PI / 2.0f))) & 3;


        // lateral correction:
        // get distance of left and right, if they have walls,
        // if left is closer than right, then closer to walls than expected
        // update pose

        if (hasLeft && hasRight) {
            // we're assuming the micromouse's heading straight, so the total distance should 
            // be leftReading + rightReading + robotWidth = cellWidth
            // if its not heading straight enough, the total distance will be larger

            float lateralDist = leftMM + rightMM + ROBOT_WIDTH_MM;

            if (fabs(lateralDist - CELL_MM) < LATERAL_TOLERANCE) {

                // near cardinal heading, snap heading to that direction
                h += LIDAR_TRUST * (cardinalH - h);

                // compare left and right distance
                float diffMM = leftMM - rightMM;
                float lateralOffsetM = (diffMM / 2.0f) / 1000.0f;

                switch (cardinalIndex) {
                    case 0: // North
                        x += LIDAR_TRUST * lateralOffsetM;
                        break;
                    
                    case 1: // East
                        y += LIDAR_TRUST * lateralOffsetM;
                        break;
                    
                    case 2: //South
                        x -= LIDAR_TRUST * lateralOffsetM;
                        break;
                    
                    case 3: //west
                        y -= LIDAR_TRUST * lateralOffsetM;
                        break;
                }    
            }
        }

        // front correction:
        // get distance of wall in front
        // if closer than expected or further than expected, update distance
        if (hasFront) {
            float frontErrorM = (frontMM - FRONT_CENTRE_OFFSET_MM) / 1000.0f;

            switch (cardinalIndex) {
                case 0: 
                    y += LIDAR_TRUST * frontErrorM;
                    break;
                
                case 1: // East
                    x -= LIDAR_TRUST * frontErrorM;
                    break;

                case 2: //South 
                    y -= LIDAR_TRUST * frontErrorM;
                    break;
                
                case 3: // West
                    x += LIDAR_TRUST * frontErrorM;
                    break;
            }
        }
        // reset wheel odometry with new pose (MAKE SURE ENCODERS ARE RESET TOO)
        odometry.reset(x, y, h);
        return;
    }

    // Resets pose of micromouse
    void reset(float newX = 0, float newY = 0, float newH = 0) {
        x = newX;
        y = newY;
        h = newH;

        odometry.reset(newX, newY, newH);

        prevGyroYawRad = radians(mpu.getAngleZ()) * IMU_CW_SIGN;
    }

    // returns x coordinate
    float getX() const {
        return x;
    }

    // returns y coordinate
    float getY() const {
        return y;
    }

    // returns heading
    float getH() const {
        return h;
    }

    float getGyroYaw() const {
        return radians(mpu.getAngleZ()) * IMU_CW_SIGN;
    }

    private:
    MPU6050& mpu;
    
    float x;
    float y;
    float h;
    
    EncoderOdometry odometry;
    float prevGyroYawRad = 0.0;
};

}