#pragma once

#include <math.h>

namespace mtrn3100 {

// Bang-Bang (On/Off) Controller
//
// Logic:
//   error = setpoint - (input - zero_ref)
//   if |error| <= deadband  → output = 0      (within acceptable zone, stop)
//   if  error  >  deadband  → output = +speed  (below target, drive forward)
//   if  error  < -deadband  → output = -speed  (above target, drive backward)
//
// Parameters:
//   speed    — PWM output magnitude when active (0–255)
//   deadband — error threshold below which output = 0 (in same units as input)
class BangBangController {
public:
    BangBangController(float speed, float deadband) : speed(speed), deadband(deadband) {}

    // Compute the PWM output based on current encoder reading (input).
    // Call this every loop iteration.
    float compute(float input) {
        error = setpoint - (input - zero_ref);

        if (fabs(error) <= deadband) {
            output = 0;           // Within deadband — stop
        } else if (error > 0) {
            output = speed;       // Below target — drive forward
        } else {
            output = -speed;      // Above target — drive backward
        }

        return output;
    }

    // Returns true when the controller has reached its target (within deadband).
    bool atTarget() {
        return fabs(error) <= deadband;
    }

    // Get the last calculated error value.
    float getError() {
        return error;
    }

    // Update speed and deadband parameters at runtime.
    void tune(float new_speed, float new_deadband) {
        speed = new_speed;
        deadband = new_deadband;
    }

    // Call this BEFORE starting a new move.
    // zero  → current encoder position (becomes the new reference zero)
    // target → desired travel in radians relative to zero
    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
    }

private:
    float speed, deadband;
    float error = 0, output = 0;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100
