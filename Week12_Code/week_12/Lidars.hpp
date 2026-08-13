
/**
 * Scans the lidars of the micromouse
 * 
 */

#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidars {

public:

    Lidars(uint8_t front, uint8_t right, uint8_t left, float wallThresholdMM) : 
                frontPin(front), rightPin(right), leftPin(left), wallThreshold(wallThresholdMM) {} 

    // Assigns each lidar an I2C address, called after Wire.begin()
    void begin() {
        pinMode(frontPin, OUTPUT);
        pinMode(rightPin, OUTPUT);
        pinMode(leftPin,  OUTPUT);

        // Pull all LOW so every sensor resets to default I2C address 0x29
        digitalWrite(frontPin, LOW);
        digitalWrite(rightPin, LOW);
        digitalWrite(leftPin,  LOW);
        delay(20);

        // Bring each sensor up one at a time and assign unique address
        digitalWrite(frontPin, HIGH); delay(50);
        lidarFront.init(); lidarFront.configureDefault();
        lidarFront.setTimeout(250); lidarFront.setAddress(0x54);

        digitalWrite(rightPin, HIGH); delay(50);
        lidarRight.init(); lidarRight.configureDefault();
        lidarRight.setTimeout(250); lidarRight.setAddress(0x56);

        digitalWrite(leftPin, HIGH); delay(50);
        lidarLeft.init(); lidarLeft.configureDefault();
        lidarLeft.setTimeout(250); lidarLeft.setAddress(0x58);

        Serial.println("LiDARs ready.");

        // fill lidar scans with initial readings
        scan();
        for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
            frontBuffer[i] = frontMM;
            rightBuffer[i] = rightMM;
            leftBuffer[i]  = leftMM;
        }
    }

    // Scans lidars and updates distances. Uses average distances from past 3 readings
    void scan() {

        frontTimedOut = false;
        rightTimedOut = false;
        leftTimedOut  = false;

        // scan front lidar and check for timeout
        int f = lidarFront.readRangeSingleMillimeters();
        if (lidarFront.timeoutOccurred()) {
            frontTimedOut = true;
            Serial.println("LidarArray: front sensor timeout.");
        } else {
            frontBuffer[bufferIndex] = (float)f;
        }

        // scan right lidar and check for timeout
        int r = lidarRight.readRangeSingleMillimeters();
        if (lidarRight.timeoutOccurred()) {
            rightTimedOut = true;
            Serial.println("LidarArray: right sensor timeout.");
        } else {
            rightBuffer[bufferIndex] = (float)r;
        }

        // scan left lidar and check for timeout
        int l = lidarLeft.readRangeSingleMillimeters();
        if (lidarLeft.timeoutOccurred()) {
            leftTimedOut = true;
            Serial.println("LidarArray: left sensor timeout.");
        } else {
            leftBuffer[bufferIndex] = (float)l;
        }

        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        // update lidar readings
        frontMM = average(frontBuffer);
        rightMM = average(rightBuffer);
        leftMM = average(leftBuffer);
    }

    float getFrontMM() const { return frontMM; }
    float getLeftMM() const { return leftMM; }
    float getRightMM() const { return rightMM; }

    bool hasWallFront() const {return frontMM < wallThreshold; }
    bool hasWallLeft() const {return frontMM < wallThreshold; }
    bool hasWallRight() const {return frontMM < wallThreshold; }

    // Returns true if a lidar has timed out
    bool hasError() const {
        return (frontTimedOut || rightTimedOut || leftTimedOut);
    }

private:

    float average(float buffer[]) {
        return (buffer[0] + buffer[1] + buffer[2]) / 3;
    }
    
    static constexpr int BUFFER_SIZE = 3;

    VL6180X lidarFront;
    VL6180X lidarRight;
    VL6180X lidarLeft;

    const uint8_t frontPin;
    const uint8_t rightPin;
    const uint8_t leftPin;
    const float wallThreshold; //mm

    float frontMM = 255.0;
    float rightMM = 255.0;
    float leftMM = 255.0;

    float frontBuffer[BUFFER_SIZE] = {255.0, 255.0, 255.0};
    float leftBuffer[BUFFER_SIZE] = {255.0, 255.0, 255.0};
    float rightBuffer[BUFFER_SIZE] = {255.0, 255.0, 255.0};
    uint8_t bufferIndex = 0;

    bool frontTimedOut;
    bool rightTimedOut;
    bool leftTimedOut;
    
};

}