#pragma once

#include <math.h>

namespace mtrn3100 {

// PID (Proportional-Integral-Derivative) Controller
//
// output = Kp * error + Ki * integral(error) + Kd * d(error)/dt
//
// P term: proportional to current error — main driving force
// I term: accumulates past error — eliminates steady-state error
// D term: rate of change of error — damping, reduces overshoot
class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute PID output from current measurement (input).
    // Call this every loop iteration.
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6f;  // Convert µs → seconds
        prev_time = curr_time;

        // Guard against very large dt on first call or after long pauses
        if (dt <= 0 || dt > 1.0f) {
            dt = 0.01f;  // Default to 10ms if timing is off
        }

        error = setpoint - (input - zero_ref);

        // Integral: accumulate error over time (∫ error dt)
        integral += error * dt;

        // Derivative: rate of change of error (d(error)/dt)
        derivative = (error - prev_error) / dt;

        // PID output
        output = kp * error + ki * integral + kd * derivative;

        // Clamp output to valid PWM range
        output = constrain(output, -255.0f, 255.0f);

        prev_error = error;

        return output;
    }

    // Returns true when error is within a tolerance band.
    bool atTarget(float tolerance = 0.1f) {
        return fabs(error) <= tolerance;
    }

    // Get the last calculated error.
    float getError() {
        return error;
    }

    // Update PID gains at runtime (for tuning).
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
        // Reset integral when retuning to avoid wind-up
        integral = 0;
    }

    // Call this BEFORE starting a new move.
    // zero   → current encoder position (new reference zero)
    // target → desired travel in radians
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
        integral = 0;      // Reset integral wind-up
        prev_error = 0;
    }

public:
    uint32_t prev_time, curr_time;
    float dt = 0;

private:
    float kp, ki, kd;
    float error = 0, derivative = 0, integral = 0, output = 0;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100
