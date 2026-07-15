#pragma once
#include <Arduino.h>

/*
 * EncoderOdometry.hpp — Differential drive pose estimation.
 *
 * Tracks robot position (x, y) and heading (h) using wheel encoder data.
 * Call update() every loop iteration with the latest encoder readings (radians).
 *
 * PARAMETERS for this robot:
 *   R = 0.016f  (wheel radius 16mm)
 *   B = 0.092f  (axle length 92mm)
 *
 * 
 */

namespace mtrn3100 {

class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase)
        : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

    // Call with latest encoder readings (radians). Both must count UP going forward.
    void update(float leftRads, float rightRads) {
        float dL = leftRads  - lastLPos;   // change in left wheel angle
        float dR = rightRads - lastRPos;   // change in right wheel angle

        // Differential drive forward kinematics
        float ds     = R * (dR + dL) / 2.0f;        // linear displacement (m)
        float dtheta = R * (dR - dL) / B;            // heading change (rad)

        // Integrate pose
        x += ds * cos(h);
        y += ds * sin(h);
        h += dtheta;

        lastLPos = leftRads;
        lastRPos = rightRads;
    }

    void reset() { x = 0; y = 0; h = 0; lastLPos = 0; lastRPos = 0; }

    float getX() const { return x; }   // metres
    float getY() const { return y; }   // metres
    float getH() const { return h; }   // radians (heading)

private:
    float x, y, h;
    const float R, B;
    float lastLPos, lastRPos;
};

}  // namespace mtrn3100
