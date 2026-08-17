
#include <Wire.h>
#include <MPU6050_light.h>

#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"
#include "Lidars.hpp"
#include "Pose.hpp"
#include "MotionController.hpp"
#include "Maze.hpp"
#include "MazeNavigator.hpp"
#include "ObstacleNavigator.hpp"
/**
 * Week 12 Assessment
 * 
 * Task numbers:
 * 1 = 4.1 Maze race (known path)
 * 2 = 4.2 Continuous planning — obstacle course only      (2 marks)
 * 3 = 4.3 Autonomous mapping of maze and solving
 * 4 = 4.2 Continuous planning — full maze through it      (4 marks)
 *
 * Claude was used to help write and debug this code
 */
#define TASK_NUM 4

// ---- Task 4.1 : Maze race -----------------------------------------------
#define PATH      "rfrffrfflffrfflff"   // update from maze_solver.py on assessment day
#define START_ROW 1
#define START_COL 5
#define START_DIR mtrn3100::Maze::NORTH
#define GOAL_ROW  7
#define GOAL_COL  2

// ---- Task 4.2 : Obstacle course -----------------------------------------
// The 5x5 section is OPEN SPACE with randomly placed 100 mm cylinders, so a
// Cartesian 'f/l/r' cell path cannot be used (Ed #204). Run obstacle_solver.py
// on the lab image and paste its output block below — it emits metric
// waypoints in global maze millimetres, which ObstacleNavigator drives.
//
// Frame: x = east (col), y = south (row); cell centre = (index + 0.5) * 180 mm.

// (the waypoint table itself lives just below the includes, since it needs
//  the mtrn3100::Waypoint type)
#define OBS_ENTRY_ROW   1                       // global maze row of entry cell
#define OBS_ENTRY_COL   6                       // global maze col of entry cell
#define OBS_ENTRY_DIR   mtrn3100::Maze::WEST    // heading as the robot enters
#define OBS_EXIT_ROW    3
#define OBS_EXIT_COL    2
#define OBS_EXIT_DIR    mtrn3100::Maze::WEST    // heading needed to drive out

// ---- Task 4.2 full-maze run (TASK_NUM 4) --------------------------------
// From task42_full.py: start -> entry cell, then the waypoints, then
// exit cell -> goal. Worth the extra 2 marks (Ed #299).
#define PRE_OBSTACLE_PATH   "rfrflffrflflfrflfrflfffflfrflf"
#define POST_OBSTACLE_PATH  "frflflfflfrflfffffrf"
#define MAZE_START_ROW      6
#define MAZE_START_COL      2
#define MAZE_START_DIR      mtrn3100::Maze::NORTH



#define USE_LIDAR_CORRECTION 1 // 0: odometry only, 1: with snapToWall correction



// ---- PASTE FROM obstacle_solver.py — BEGIN ----
// Task 4.2 -- waypoints in GLOBAL maze millimetres.
// Frame: x = east (col), y = south (row); cell centre = (idx+0.5)*180.
// Generated from pic015.jpg. Section rows 0-4, cols 2-6.
const mtrn3100::Waypoint OBSTACLE_WAYPOINTS[] = {
    {  1170.0f,   270.0f },   // entry
    {  1037.5f,   272.5f },
    {   737.5f,   567.5f },
    {   482.5f,   632.5f },
    {   450.0f,   630.0f },   // exit
};


#define OBSTACLE_WAYPOINT_COUNT  5
// ---- PASTE FROM obstacle_solver.py — END ----

// Motor pins (DRV8835, Phase/Enable mode)
#define MOT1_PWM  9
#define MOT1_DIR  10
#define MOT2_PWM  11
#define MOT2_DIR  12

// Encoder pins
#define ENC1_A  2
#define ENC1_B  4
#define ENC2_A  3
#define ENC2_B  5

// LiDAR enable pins
#define LIDAR_FRONT_EN  A2
#define LIDAR_RIGHT_EN  A1
#define LIDAR_LEFT_EN   A0

// PID gains 
#define HEADING_KP  130.0f
#define HEADING_KI  0.0f
#define HEADING_KD  0.1f
#define DISTANCE_KP 14.0f
#define DISTANCE_KI 0.0f
#define DISTANCE_KD 0.1f

// Tuned Parameters
#define WALL_THRESHOLD 100
#define CELL_M 0.18f
const uint16_t CPR = 690;

// Hardware
mtrn3100::Motor motorL(MOT1_PWM, MOT1_DIR);
mtrn3100::Motor motorR(MOT2_PWM, MOT2_DIR);
mtrn3100::DualEncoder encoder(ENC1_A, ENC1_B, ENC2_A, ENC2_B);
MPU6050 mpu(Wire);
mtrn3100::Lidars lidars(LIDAR_FRONT_EN, LIDAR_RIGHT_EN, LIDAR_LEFT_EN, WALL_THRESHOLD);

// PID Controllers
mtrn3100::PIDController headingPid(HEADING_KP, HEADING_KI, HEADING_KD);
mtrn3100::PIDController distancePid(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);

mtrn3100::Pose pose(mpu);
mtrn3100::MotionController motion(pose, lidars, mpu, motorL, motorR, encoder, headingPid, distancePid);
mtrn3100::Maze maze(GOAL_ROW, GOAL_COL);
mtrn3100::MazeNavigator mazeNav(maze, motion);
mtrn3100::ObstacleNavigator obstacleNav(pose, motion);

// Heading in radians for a maze direction (NORTH = 0, EAST = +PI/2, ...)
float headingOf(mtrn3100::Maze::Direction dir) {
    switch (dir) {
        case mtrn3100::Maze::NORTH: return 0.0f;
        case mtrn3100::Maze::EAST:  return M_PI / 2.0f;
        case mtrn3100::Maze::SOUTH: return M_PI;
        default:                    return -M_PI / 2.0f;  // WEST
    }
}

void initHardware() {
    Wire.begin();
    lidars.begin();

    byte status = mpu.begin();
    if (status != 0) {
        Serial.print(F("IMU failed (")); Serial.print(status); Serial.println(F(")."));
        while (1);
    }

    // force ±500 deg/s gyro range (battery-mode bug fix)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Serial.println(F("Calibrating IMU, hold still..."));
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println(F("IMU ready."));

    float x = (START_COL + 0.5f) * CELL_M;
    float y = (START_ROW + 0.5f) * CELL_M;
    pose.reset(x, y, headingOf(START_DIR));

    motion.stop();
}

void setup() {
    Serial.begin(115200);
    initHardware();

    motion.setLidarCorrection(USE_LIDAR_CORRECTION);
}

void loop() {

    // initial delay for 5s
    delay(2000);
    Serial.println(F("Task starting in 3 seconds"));
    delay(3000);

    #if TASK_NUM == 1
        mazeNav.executePath(PATH, START_ROW, START_COL, START_DIR);

    #elif (TASK_NUM == 2)
        // Standalone continuous-section run (2 marks). The robot is placed by
        // hand in the entry cell, facing the way it would have entered.
        //
        // The section has no walls, so snapToWall() is disabled — it would
        // latch onto a cylinder and corrupt the pose. ObstacleNavigator turns
        // the front LiDAR into a plain collision guard instead.
        pose.reset((OBS_ENTRY_COL + 0.5f) * 0.18f,
                   (OBS_ENTRY_ROW + 0.5f) * 0.18f,
                   headingOf(OBS_ENTRY_DIR));

        if (obstacleNav.executeWaypoints(OBSTACLE_WAYPOINTS,
                                         OBSTACLE_WAYPOINT_COUNT,
                                         headingOf(OBS_EXIT_DIR))) {
            // Clear of the cylinders and squared up: drive out of the section.
            motion.setLidarCorrection(true);
            motion.forwardOneCell();
        }

    #elif (TASK_NUM == 3)
        mazeNav.exploreAndSolveMaze();

    #elif (TASK_NUM == 4)
        // Full maze run through the obstacle course (Task 4.2, all 5 marks).
        // Three stages: walled maze -> continuous section -> walled maze.
        pose.reset((MAZE_START_COL + 0.5f) * CELL_M,
                   (MAZE_START_ROW + 0.5f) * CELL_M,
                   headingOf(MAZE_START_DIR));

        // Stage 1 — grid navigation up to the entry cell, walls available.
        motion.setLidarCorrection(true);
        if (mazeNav.executePath(PRE_OBSTACLE_PATH, MAZE_START_ROW,
                                MAZE_START_COL, MAZE_START_DIR)) {

            // Stage 2 — continuous section. executeWaypoints turns LiDAR
            // snapping off itself and re-enables the collision guard.
            if (obstacleNav.executeWaypoints(OBSTACLE_WAYPOINTS,
                                             OBSTACLE_WAYPOINT_COUNT,
                                             headingOf(OBS_EXIT_DIR))) {

                // Stage 3 — back on the grid. The first forwardOneCell of this
                // path drives out through the opening and snaps against a wall,
                // recovering whatever drift the section introduced.
                motion.setLidarCorrection(true);
                mazeNav.executePath(POST_OBSTACLE_PATH, OBS_EXIT_ROW,
                                    OBS_EXIT_COL, OBS_EXIT_DIR);
            }
        }

    #endif

    Serial.println(F("Done. Switch off."));
    while (true) { delay(1000); }   // halt — prevent loop() restarting the path
}