#pragma once
#include <Arduino.h>

/*
 * DualEncoder.hpp — Quadrature encoder reader for both wheels.
 *
 * PIN ASSIGNMENTS (confirmed from Calibration.ino):
 *   Left:  interrupt=2, direction=4
 *   Right: interrupt=3, direction=5
 *
 * DIRECTION NOTE:
 *   Both encoders count UP when the robot drives FORWARD.
 *   If the right encoder counts DOWN when going forward, set RIGHT_INVERTED = true.
 *   Verify with Calibration.ino: drive forward, both counts should increase.
 */

namespace mtrn3100 {

class DualEncoder {
public:
    // RIGHT_INVERTED: set true if right encoder counts backwards when driving forward
    DualEncoder(uint8_t enc1_int, uint8_t enc1_dir,
                uint8_t enc2_int, uint8_t enc2_dir,
                bool right_inverted = false)
        : mot1_int(enc1_int), mot1_dir(enc1_dir),
          mot2_int(enc2_int), mot2_dir(enc2_dir),
          rightInverted(right_inverted)
    {
        instance = this;
        pinMode(mot1_int, INPUT_PULLUP); pinMode(mot1_dir, INPUT_PULLUP);
        pinMode(mot2_int, INPUT_PULLUP); pinMode(mot2_dir, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(mot1_int), readLeftISR,  RISING);
        attachInterrupt(digitalPinToInterrupt(mot2_int), readRightISR, RISING);
    }

    // Convert counts to radians of wheel rotation.
    // noInterrupts() guard prevents a corrupted read of the 4-byte long on 8-bit AVR
    // (the ISR can fire mid-read and change the value between byte accesses).
    float getLeftRotation() const {
        noInterrupts();
        long val = l_count;
        interrupts();
        return (float)val / (float)counts_per_revolution * 2.0f * PI;
    }
    float getRightRotation() const {
        noInterrupts();
        long val = r_count;
        interrupts();
        return (float)val / (float)counts_per_revolution * 2.0f * PI;
    }

    // Reset both counters to zero (call before each new movement)
    void reset() {
        noInterrupts();
        l_count = 0;
        r_count = 0;
        interrupts();
    }

    // ISR handlers — called on each encoder pulse
    void onLeftPulse()  { l_count += (digitalRead(mot1_dir) == HIGH) ?  1 : -1; }
    void onRightPulse() { r_count += (digitalRead(mot2_dir) == HIGH) ? (rightInverted ? -1 : 1)
                                                                      : (rightInverted ?  1 : -1); }

public:
    const uint8_t mot1_int, mot1_dir, mot2_int, mot2_dir;
    const bool    rightInverted;
    uint16_t      counts_per_revolution = 690;   // measured from Calibration.ino
    volatile long l_count = 0;
    volatile long r_count = 0;

private:
    static void readLeftISR()  { if (instance) instance->onLeftPulse();  }
    static void readRightISR() { if (instance) instance->onRightPulse(); }
    static DualEncoder* instance;
};

DualEncoder* DualEncoder::instance = nullptr;

}  // namespace mtrn3100
