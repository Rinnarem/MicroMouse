
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
    bool begin() {
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
        lidarFront.init();
        uint8_t frontID = lidarFront.readReg(VL6180X::IDENTIFICATION__MODEL_ID);
        if (lidarFront.last_status != 0 || frontID != 0xB4) {
            Serial.println(F("[LIDAR ERROR] front init failed"));
            return false;
        }
        lidarFront.configureDefault();
        lidarFront.setTimeout(250); lidarFront.setAddress(0x54);

        digitalWrite(rightPin, HIGH); delay(50);
        lidarRight.init();
        uint8_t rightID = lidarRight.readReg(VL6180X::IDENTIFICATION__MODEL_ID);
        if (lidarRight.last_status != 0 || rightID != 0xB4) {
            Serial.println(F("[LIDAR ERROR] right init failed"));
            return false;
        }
        lidarRight.configureDefault();
        lidarRight.setTimeout(250); lidarRight.setAddress(0x56);

        digitalWrite(leftPin, HIGH); delay(50);
        lidarLeft.init();
        uint8_t leftID = lidarLeft.readReg(VL6180X::IDENTIFICATION__MODEL_ID);
        if (lidarLeft.last_status != 0 || leftID != 0xB4) {
            Serial.println(F("[LIDAR ERROR] left init failed"));
            return false;
        }
        lidarLeft.configureDefault();
        lidarLeft.setTimeout(250); lidarLeft.setAddress(0x58);

        // fill lidar scans with initial readings
        scan();
        if (hasError()) {
            Serial.println(F("[LIDAR ERROR] initial scan failed"));
            return false;
        }
        for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
            frontBuffer[i] = frontMM;
            rightBuffer[i] = rightMM;
            leftBuffer[i]  = leftMM;
        }
        Serial.println(F("LiDARs ready."));
        return true;
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
            Serial.println(F("LidarArray: front sensor timeout."));
        } else {
            frontBuffer[bufferIndex] = (float)f;
        }

        // scan right lidar and check for timeout
        int r = lidarRight.readRangeSingleMillimeters();
        if (lidarRight.timeoutOccurred()) {
            rightTimedOut = true;
            Serial.println(F("LidarArray: right sensor timeout."));
        } else {
            rightBuffer[bufferIndex] = (float)r;
        }

        // scan left lidar and check for timeout
        int l = lidarLeft.readRangeSingleMillimeters();
        if (lidarLeft.timeoutOccurred()) {
            leftTimedOut = true;
            Serial.println(F("LidarArray: left sensor timeout."));
        } else {
            leftBuffer[bufferIndex] = (float)l;
        }

        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        // update lidar readings
        frontMM = average(frontBuffer);
        rightMM = average(rightBuffer);
        leftMM = average(leftBuffer);
    }

    // Read the front sensor only. scan() reads all three and blocks for
    // ~30 ms, which is too slow to poll inside a control loop; this is ~10 ms.
    // Returns 255 (i.e. "nothing there") on a timeout so a dead sensor can
    // never stop the robot early.
    float scanFrontMM() {
        int f = lidarFront.readRangeSingleMillimeters();
        if (lidarFront.timeoutOccurred()) { frontTimedOut = true; return 255.0f; }
        frontTimedOut = false;
        return (float)f;
    }

    // Read only the side sensors for a lightweight in-motion clearance guard.
    // Returns raw readings; the controller requires two consecutive close
    // samples before steering, so a single noisy reading is ignored.
    bool scanSidesMM(float& left, float& right) {
        int l = lidarLeft.readRangeSingleMillimeters();
        leftTimedOut = lidarLeft.timeoutOccurred();

        int r = lidarRight.readRangeSingleMillimeters();
        rightTimedOut = lidarRight.timeoutOccurred();

        if (leftTimedOut || rightTimedOut) return false;

        left = (float)l;
        right = (float)r;
        return true;
    }

    // Print the three readings. Without this there is no way to tell a working
    // sensor from a dead one -- nothing else in the code ever shows a number.
    void report() {
        Serial.print(F("  [LIDAR] F=")); Serial.print(frontMM, 0);
        Serial.print(F(" L="));          Serial.print(leftMM, 0);
        Serial.print(F(" R="));          Serial.print(rightMM, 0);
        Serial.print(F("   wall F/L/R: "));
        Serial.print(hasWallFront() ? F("Y") : F("n"));
        Serial.print(hasWallLeft()  ? F("Y") : F("n"));
        Serial.print(hasWallRight() ? F("Y") : F("n"));
        if (hasWallLeft() && hasWallRight()) {
            // With the robot centred in a corridor, ROBOT_WIDTH_MM in Pose.hpp
            // should equal 180 - (L + R). If it does not, every lateral
            // correction is being rejected.
            Serial.print(F("  L+R=")); Serial.print(leftMM + rightMM, 0);
            Serial.print(F(" -> ROBOT_WIDTH_MM should be "));
            Serial.print(180.0f - leftMM - rightMM, 0);
        }
        if (hasError()) Serial.print(F("   *** TIMEOUT ***"));
        Serial.println();
    }

    // Print several stationary samples at startup. Sensor presence was already
    // verified from each model-ID register in begin(); a reading of 255 here
    // can legitimately mean that nothing is in range.
    bool selfTest() {
        Serial.println(F("LiDAR self-test:"));
        bool ok = true;
        for (uint8_t i = 0; i < 3; i++) {
            scan();
            if (hasError()) ok = false;
            report();
            delay(60);
        }
        Serial.println(ok ? F("  startup readings complete")
                          : F("  a sensor timed out - do not drive"));
        return ok;
    }

    // Stream readings so you can wave a hand at each sensor and confirm which
    // channel is which. Set LIDAR_WATCH to 1 in week_12.ino.
    void watch(unsigned long ms) {
        Serial.println(F("LiDAR watch - wave a hand at FRONT, then LEFT, then RIGHT"));
        unsigned long t0 = millis();
        while (millis() - t0 < ms) { scan(); report(); delay(150); }
    }

    float getFrontMM() const { return frontMM; }
    float getLeftMM() const { return leftMM; }
    float getRightMM() const { return rightMM; }

    bool hasWallFront() const {return frontMM < wallThreshold; }
    bool hasWallLeft() const {return leftMM < wallThreshold; }
    bool hasWallRight() const {return rightMM < wallThreshold; }

    // Returns true if a lidar has timed out
    bool hasError() const {
        return (frontTimedOut || rightTimedOut || leftTimedOut);
    }

    static constexpr int BUFFER_SIZE = 3;

private:

    float average(float buffer[]) {
        return (buffer[0] + buffer[1] + buffer[2]) / 3;
    }
    
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

    bool frontTimedOut = false;
    bool rightTimedOut = false;
    bool leftTimedOut  = false;
    
};

}
