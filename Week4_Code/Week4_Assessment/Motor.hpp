#pragma once

#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {

// The motor class is a simple interface designed to assist in motor control.
// Works with the DRV8835 motor driver in Phase/Enable mode:
//   dir_pin  = PHASE pin  (HIGH = forward, LOW = reverse)
//   pwm_pin  = ENABLE pin (PWM duty cycle = speed, 0–255)
class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t in2) : pwm_pin(pwm_pin), dir_pin(in2) {
        // Set both pins as OUTPUT so the Arduino can control them
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        // Ensure motor starts stopped
        analogWrite(pwm_pin, 0);
        digitalWrite(dir_pin, LOW);
    }

    // Set motor speed and direction.
    // pwm > 0  → forward  (up to +255)
    // pwm < 0  → reverse  (down to -255)
    // pwm = 0  → stop
    void setPWM(int16_t pwm) {
        // Clamp to valid range to avoid overflow
        pwm = constrain(pwm, -255, 255);

        if (pwm >= 0) {
            digitalWrite(dir_pin, HIGH);   // PHASE = forward
            analogWrite(pwm_pin, pwm);     // ENABLE = speed
        } else {
            digitalWrite(dir_pin, LOW);    // PHASE = reverse
            analogWrite(pwm_pin, -pwm);    // ENABLE = speed (must be positive)
        }
    }

    // Convenience: stop the motor immediately
    void stop() {
        analogWrite(pwm_pin, 0);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
