
/**
 * Scans the lidars of the micromouse
 * 
 */

#pragma once
#include <stdint.h>
#include <Arduino.h>
#include <VL6180X.h>

namespace MTRN3100 {

class Lidars {

    public:

    Lidars(uint8_t front, uint8_t right, uint8_t left, float wallThresholdMM) {

        frontPin = front;
        rightPin = right;
        leftPin = left;
        wallThreshold = wallThreshold;
    } 

    // Assigns each lidar an I2C address, called after Wire.begin()
    void begin() {
        //TODO
    }

    // Scans lidars and updates distances
    void scan() {
        //TODO
    }

    float getFrontMM() const { return frontMM;}
    float getLeftMM() const { return leftMM;}
    float getRightMM() const { return rightMM;}

    bool hasWallFront() const {return frontMM < wallThreshold; }
    bool hasWallLeft() const {return frontMM < wallThreshold; }
    bool hasWallRight() const {return frontMM < wallThreshold; }

    private:

    const uint8_t frontPin;
    const uint8_t rightPin;
    const uint8_t leftPin;
    const float wallThreshold; //mm

    float frontMM = 255.0;
    float rightMM = 255.0;
    float leftMM = 255.0;
    

    VL6180X lidarFront;
    VL6180X lidarRight;
    VL6180X lidarLeft;
}

}