/**
 * Week 12 Assessment
 * 
 * Task numbers:
 * 1 = 4.1 Maze race(known path)
 * 2 = 4.2 Continuous planning obstacle course
 * 3 = 4.3 Autonomous mapping of maze and solving
 * 
 * Claude was used to help write and debug this code
 */

#define TASK_NUM 1
#define PATH "frlrf"

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

// PID gains — placeholders, to be tuned
#define HEADING_KP  3.0f
#define HEADING_KI  0.0f
#define HEADING_KD  0.0f
#define DISTANCE_KP 2.0f
#define DISTANCE_KI 0.02f
#define DISTANCE_KD 0.3f

#define GOAL_ROW  8
#define GOAL_COL  8

// Hardware
mtrn3100::Motor motorL(MOT1_PWM, MOT1_DIR);
mtrn3100::Motor motorR(MOT2_PWM, MOT2_DIR);
mtrn3100::DualEncoder encoder(ENC1_A, ENC1_B, ENC2_A, ENC2_B);
MPU6050 mpu(Wire);
mtrn3100::LidarArray lidars(LIDAR_FRONT_EN, LIDAR_RIGHT_EN, LIDAR_LEFT_EN);

// PID Controllers
mtrn3100::PIDController headingPID(HEADING_KP, HEADING_KI, HEADING_KD);
mtrn3100::PIDController distancePID(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);

mtrn3100::Pose pose(mpu);
mtrn3100::MotionController motion(motorL, motorR, encoder, pose, lidars,
                                   headingPID, distancePID);
mtrn3100::Maze maze(GOAL_ROW, GOAL_COL);
mtrn3100::MazeNavigator mazeNav(maze, motion);
// TODO: add ObstacleNavigator class

void InitHardware() {
    //TODO: initialise hardware
}

void setup() {
    //TODO
}

void loop() {
    #if TASK_NUM == 1 
        mazeNav.executePath(PATH);

    #elif (TASK_NUM == 2) 
        //TODO: implement task 2
        
    #elif (TASK_NUM == 3) 
        mazeNav.exploreAndSolveMaze();
    

    Serial.println("Done. Switch off.");
    delay(5000);

}