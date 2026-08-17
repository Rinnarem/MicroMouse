/**
 * ObstacleNavigator — Task 4.2 continuous section
 *
 * The 5x5 obstacle course is open space with randomly placed 100 mm cylinders,
 * so there is no cell grid to step through and no walls to square up against.
 * obstacle_solver.py plans a continuous route and reduces it to a handful of
 * metric waypoints; this class drives them in series.
 *
 * For each waypoint: work out the bearing from the current pose, turn on the
 * spot by the heading error, then drive the straight-line distance.
 *
 * Coordinate frame (identical to Pose / MazeNavigator):
 *      x  = east,  increases with column
 *      y  = south, increases with row
 *      cell centre = (index + 0.5) * 180 mm
 *      heading: NORTH = 0, EAST = +PI/2, SOUTH = PI, WEST = -PI/2
 *
 * Because Pose integrates x += ds*sin(h) and y -= ds*cos(h), the bearing of a
 * displacement (dx, dy) is atan2(dx, -dy) — note the negated dy.
 *
 * Claude was used to help write and debug this code.
 */

#pragma once

#include <Arduino.h>
#include <math.h>
#include "Pose.hpp"
#include "MotionController.hpp"

namespace mtrn3100 {

// A single target point, in global maze millimetres.
struct Waypoint {
    float x;
    float y;
};

class ObstacleNavigator {

public:

    // Stop refining once we are this close — the planner leaves far more
    // clearance than this, so chasing the last few mm is not worth the time.
    static constexpr float ARRIVAL_TOLERANCE_MM = 12.0f;

    // Skip the turn entirely below this bearing error (~1.7 deg).
    static constexpr float HEADING_DEADBAND_RAD = 0.03f;

    // Speed ceiling inside the section. There are no walls in here to re-square
    // against, so every millimetre of odometry drift is permanent -- a steady
    // run is worth more than a quick one. Raise it if the run feels too slow.
    static constexpr float SECTION_MAX_PWM = 170.0f;

    ObstacleNavigator(Pose& pose, MotionController& motion)
        : pose(pose), motion(motion) {}

    /**
     * Drives the planned waypoint list.
     *
     * waypoints        — array from obstacle_solver.py, global maze mm.
     *                    Element 0 is the entry cell centre, which is where the
     *                    robot already stands, so it is used as the starting
     *                    reference rather than driven to.
     * count            — number of waypoints.
     * exitHeadingRad   — heading to face once the last waypoint is reached, so
     *                    the robot can drive straight out of the section.
     *
     * Returns true only if every leg completed.
     */
    bool executeWaypoints(const Waypoint* waypoints, uint8_t count,
                          float exitHeadingRad) {

        if (waypoints == nullptr || count < 2) {
            Serial.println(F("[OBS] need at least 2 waypoints"));
            return false;
        }

        // No walls in here: LiDAR pose-snapping would latch onto cylinders and
        // corrupt the pose. Keep the front sensor as a collision guard only.
        motion.setLidarCorrection(false);
        motion.setObstacleGuard(true);
        motion.setSpeedLimit(SECTION_MAX_PWM);

        Serial.println(F("[OBS] starting continuous section"));

        for (uint8_t i = 1; i < count; i++) {
            if (!driveToPoint(waypoints[i].x, waypoints[i].y, i)) {
                motion.setObstacleGuard(false);
                motion.setSpeedLimit(MotionController::DEFAULT_MAX_PWM);
                return false;
            }
        }

        // Square up with the exit opening before leaving the section.
        Serial.print(F("[OBS] facing exit heading "));
        Serial.println(degrees(exitHeadingRad));
        if (!motion.turnToHeading(exitHeadingRad)) {
            motion.setObstacleGuard(false);
            motion.setSpeedLimit(MotionController::DEFAULT_MAX_PWM);
            return false;
        }

        motion.setObstacleGuard(false);
        motion.setSpeedLimit(MotionController::DEFAULT_MAX_PWM);
        Serial.println(F("[OBS] continuous section complete"));
        return true;
    }

private:

    // Turn towards a point then drive to it.
    bool driveToPoint(float targetXMM, float targetYMM, uint8_t index) {

        float dx = targetXMM - (pose.getX() * 1000.0f);
        float dy = targetYMM - (pose.getY() * 1000.0f);
        float distanceMM = sqrt(dx * dx + dy * dy);

        if (distanceMM < ARRIVAL_TOLERANCE_MM) {
            Serial.print(F("[OBS] waypoint "));
            Serial.print(index);
            Serial.println(F(" already reached"));
            return true;
        }

        // Bearing in the maze frame. dy is negated because y grows southward
        // while heading 0 points north.
        float desiredHeading = atan2(dx, -dy);
        float headingError =
            MotionController::normaliseAngle(desiredHeading - pose.getH());

        Serial.print(F("[OBS] waypoint "));
        Serial.print(index);
        Serial.print(F(" -> ("));
        Serial.print(targetXMM); Serial.print(F(", ")); Serial.print(targetYMM);
        Serial.print(F(") turn ")); Serial.print(degrees(headingError));
        Serial.print(F(" deg, drive ")); Serial.print(distanceMM);
        Serial.println(F(" mm"));

        if (fabs(headingError) > HEADING_DEADBAND_RAD) {
            if (!motion.turnByRadians(headingError)) {
                Serial.println(F("[OBS] turn failed"));
                return false;
            }
        }

        // Re-measure after turning: the turn is only accurate to
        // TURN_TOLERANCE_RAD and the pose may have shifted slightly.
        dx = targetXMM - (pose.getX() * 1000.0f);
        dy = targetYMM - (pose.getY() * 1000.0f);
        distanceMM = sqrt(dx * dx + dy * dy);

        if (!motion.driveDistanceMM(distanceMM, false)) {
            if (motion.guardTripped()) {
                Serial.println(F("[OBS] STOPPED — obstacle detected ahead"));
            } else {
                Serial.println(F("[OBS] drive failed"));
            }
            return false;
        }

        Serial.print(F("[OBS] at ("));
        Serial.print(pose.getX() * 1000.0f); Serial.print(F(", "));
        Serial.print(pose.getY() * 1000.0f); Serial.println(F(")"));
        return true;
    }

    Pose& pose;
    MotionController& motion;
};

}  // namespace mtrn3100
