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
#define PATH "frrfll"

#define GOAL_ROW  8
#define GOAL_COL  8
#define START_ROW 0
#define START_COL 0
#define START_DIR mtrn3100::Maze::SOUTH

#define USE_LIDAR_CORRECTION 1 // 0: odometry only, 1: with snaptoWall correction

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

// PID gains 
#define HEADING_KP  200.0f
#define HEADING_KI  0.0f
#define HEADING_KD  0.1f
#define DISTANCE_KP 14.0f
#define DISTANCE_KI 0.0f
#define DISTANCE_KD 0.1f

// Tuned Parameters
#define WALL_THRESHOLD 100
#define CELL_M 0.18f;
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
// TODO: add ObstacleNavigator class

void initHardware() {
    Wire.begin();
    lidars.begin();

    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("IMU failed ("); Serial.print(status); Serial.println(").");
        while (1);
    }

    // force ±500 deg/s gyro range (battery-mode bug fix)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Serial.println("Calibrating IMU, hold still...");
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println("IMU ready.");

    float x = (START_COL + 0.5f) * CELL_M;
    float y = (START_ROW + 0.5f) * CELL_M;
    float h;
    switch (START_DIR) {
        case mtrn3100::Maze::NORTH:
            h = 0;
            break;
        case mtrn3100::Maze::EAST:
            h = M_PI / 2;
            break;
        case mtrn3100::Maze::SOUTH:
            h = M_PI;
            break;
        case mtrn3100::Maze::WEST:
            h = -M_PI / 2;
            break;
    }
    pose.reset(x, y, h);

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
    Serial.println("Task starting in 3 seconds");
    delay(3000);

    #if TASK_NUM == 1 
        mazeNav.executePath(PATH, START_ROW, START_COL, START_DIR);

    #elif (TASK_NUM == 2) 
        //TODO: implement task 2

    #elif (TASK_NUM == 3) 
        mazeNav.exploreAndSolveMaze();
    
    #endif

    Serial.println("Done. Switch off.");
    delay(5000);

}