#pragma once

#include <Arduino.h>

namespace mtrn3100 {

// The encoder class counts pulses from a Hall-effect quadrature encoder.
// encoder1_pin → interrupt pin (must be D2 or D3 on Arduino Nano)
// encoder2_pin → any digital pin; used to determine rotation direction
//
// How direction detection works (quadrature encoding):
//   When encoder1 fires (rising edge), read encoder2:
//     encoder2 = HIGH → shaft rotating forward  → count++
//     encoder2 = LOW  → shaft rotating backward → count--
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        instance = this;  // Store the instance pointer for use in static ISR
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // Attach interrupt: call readEncoderISR every time encoder1 goes LOW→HIGH
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR, RISING);
    }

    // Called by the interrupt service routine — do NOT call this manually.
    // NOTE: No Serial.print, no delay inside here.
    void readEncoder() {
        noInterrupts();
        // If encoder2 is HIGH when encoder1 fires → forward, else backward
        if (digitalRead(encoder2_pin) == HIGH) {
            count++;   // Forward
        } else {
            count--;   // Backward
        }
        interrupts();
    }

    // Convert accumulated encoder count to radians of wheel rotation.
    // Formula: radians = (count / counts_per_revolution) × 2π
    float getRotation() {
        if (counts_per_revolution == 0) return 0;  // Safety: avoid divide-by-zero
        return (float)count / (float)counts_per_revolution * 2.0f * M_PI;
    }

    // Reset count to zero (call before each move to track from current position)
    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }

private:
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction = 0;
    float position = 0;

    // ★ SET THIS BEFORE USE ★
    // counts_per_revolution = encoder pulses per full wheel rotation
    // For DFROBOT FIT083: measure experimentally (see calibration guide)
    // Typical values: 360–600
    uint16_t counts_per_revolution = 360;  // TODO: calibrate for your motor

    volatile long count = 0;
    uint32_t prev_time = 0;
    bool read = false;

private:
    static Encoder* instance;
};

Encoder* Encoder::instance = nullptr;

}  
