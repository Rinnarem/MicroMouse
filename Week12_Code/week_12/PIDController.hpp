#pragma once

#include <Arduino.h>   // micros(), constrain(), uint32_t
#include <math.h>

namespace mtrn3100 {

/*
 * PIDController — Generic PID controller.
 *
 * output = Kp*error + Ki*integral(error) + Kd*d(error)/dt
 *
 * Call zeroAndSetTarget() before each new movement, then compute() each loop.
 *
 * NOTE: Not used in Week 8 assessment (Task 2 uses an inline PID).
 *       Kept here as a reusable utility for future tasks.
 * 
 * Claude was used to help write and debug this code
 */
class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute PID output from current measurement (input). Call each loop iteration.
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6f;
        prev_time = curr_time;

        // Guard against very large dt on first call or after long pauses
        if (dt <= 0 || dt > 1.0f) dt = 0.01f;

        error = setpoint - (input - zero_ref);

        integral += error * dt;
        // Anti-windup: clamp integral contribution to [-255, 255] effective output
        if (ki != 0.0f) integral = constrain(integral, -255.0f / ki, 255.0f / ki);

        derivative = (error - prev_error) / dt;

        output = kp * error + ki * integral + kd * derivative;
        output = constrain(output, -255.0f, 255.0f);

        prev_error = error;
        return output;
    }

    // Returns true when error is within tolerance band.
    bool atTarget(float tolerance = 0.1f) const {
        return fabs(error) <= tolerance;
    }

    // Get the last calculated error.
    float getError() const {
        return error;
    }

    // Update PID gains at runtime.
    void tune(float p, float i, float d) {
        kp = p; ki = i; kd = d;
        integral = 0;   // reset on retune to avoid windup from old gains
    }

    // Call this BEFORE starting a new move.
    // zero   -> current encoder/sensor reading (new reference zero)
    // target -> desired value relative to zero
    void zeroAndSetTarget(float zero, float target) {
        prev_time  = micros();
        zero_ref   = zero;
        setpoint   = target;
        integral   = 0;
        prev_error = 0;
    }

public:
    uint32_t prev_time = 0, curr_time = 0;
    float    dt = 0;

private:
    float kp, ki, kd;
    float error = 0, derivative = 0, integral = 0, output = 0;
    float prev_error = 0;
    float setpoint   = 0;
    float zero_ref   = 0;
};

}  // namespace mtrn3100
