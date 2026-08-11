/*
* Pose class for Week 12 Assessment
* Uses encoder odometry, imu and lidars to estimate pose of the micromouse
*/
#pragma once

#include <Arduino.h>
#include "EncoderOdometry.hpp"

namespace mtrn3100 {

class Pose {

public: 

    static constexpr float WHEEL_RADIUS_M  = 0.016f;
    static constexpr float  AXLE_LENGTH_M   = 0.092f;

    Pose(MPU6050& imu, float startX = 0, float startY = 0, float startH = 0) {
        mpu = imu;
        x = startX;
        y = startY;
        h = startH;
    }

    // updates the estimated pose of the micromouse using encoder odometry and imu and using a complementary filter
    void update(float leftRads, float rightRads, float gyroYaw) {
        odometry(WHEEL_RADIUS_M, AXLE_LENGTH_M);
        return;
    }

    // snaps pose to a position based off lidar readings from front and side walls
    void snapToWall(float frontMM, float leftMM, float rightMM, bool hasFront, 
                        bool hasRight, bool hasLeft) {
        //TODO
        return;
    }

    // Resets pose of micromouse
    void reset(float newX = 0, float newY = 0, float newH = 0) {
        x = newX;
        y = newY;
        h = newH;
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

    private:
    EncoderOdometry odometry;
    MPU6050& mpu;

    float x;
    float y;
    float h;
    float prevGyroYaw = 0.0;
};

}