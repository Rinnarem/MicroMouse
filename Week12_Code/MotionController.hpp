/**
 * Controls the motion of the micromouse
 * 
 * Moves micromouse forward one cell, turns left, turns right 90 degrees
 * Incorporates the odometry, PID, Lidars
 */

#pragma once

#include "Lidars.hpp"
#include "Pose.hpp"
#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"

namespace mtrn3100 {

class MotionController {

public:
    static constexpr float CELL_MM = 180.0f;

    MotionController(Pose& pose, Lidars& lidars, Motor& motorL, Motor& motorR,
                        DualEncoder& encoder, PIDController& headingPid, 
                        PIDController& distancePid)
        : pose(pose), lidars(lidars), motorL(motorL), motorR(motorR),
        encoder(encoder), headingPid(headingPid), distancePid(distancePid) {}

    // Moves the micromouse forward one cell, returns true if successful
    bool forwardOneCell() {
        //TODO
        return true;
    }

    // Turns the micromouse left 90 degrees, returns true if successful
    bool turnLeft90() {
        //TODO
        return true;
    }

    // Turns the micromouse right 90 degrees, returns true if successful
    bool turnRight90() {
        //TODO
        return true;
    }

    // Stops the micromouse, returns true if successful
    bool stop() {
        //TODO
        return true;
    }


private:
    Pose& pose;
    Lidars& lidars;
    Motor& motorL;
    Motor& motorR;
    PIDController& headingPid;
    PIDController& distancePid;
    DualEncoder& encoder;
}

}