#pragma once
#include <Arduino.h>

/*
 * IMUOdometry.hpp — Dead-reckoning via accelerometer double-integration.
 *
 * NOT USED in Week 8 assessment. Kept as an experimental reference only.
 *
 * WARNING: Double-integrating raw accelerometer data accumulates drift very
 * quickly (seconds). This class is not suitable for practical positioning.
 * Use EncoderOdometry.hpp instead.
 */

namespace mtrn3100 {

class IMUOdometry {
public:
    IMUOdometry() : x(0), y(0), vx(0), vy(0), lastUpdateTime(millis()) {}

    // accel_x / accel_y in m/s^2 (already bias-corrected)
    void update(float accel_x, float accel_y) {
        unsigned long currentTime = millis();
        float dt = (currentTime - lastUpdateTime) / 1000.0f;  // ms -> s
        lastUpdateTime = currentTime;

        // Guard against dt = 0 on first call
        if (dt <= 0.0f || dt > 1.0f) return;

        // Integrate acceleration -> velocity
        vx += accel_x * dt;
        vy += accel_y * dt;

        // Integrate velocity -> position
        x += vx * dt;
        y += vy * dt;
    }

    void reset() { x = 0; y = 0; vx = 0; vy = 0; lastUpdateTime = millis(); }

    float getX() const { return x; }
    float getY() const { return y; }

private:
    float x, y;
    float vx, vy;
    unsigned long lastUpdateTime;
};

}  // namespace mtrn3100
