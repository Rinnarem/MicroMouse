/**
 * Uses Maze and MotionController class to navigate the maze
 */

#pragma once

#include "Maze.hpp"
#include "MotionController.hpp"

namespace mtrn3100 {

class MazeNavigator {

public:

    MazeNavigator(Maze& maze, MotionController& motion) : maze(maze), motion(motion) {}

    // 4.1 - given a string of commands, executes the path and returns true if successful
    bool executePath (const char* path) {
        for (int i = 0; commands[i] != '\0'; i++) {
        switch (commands[i]) {
            case 'f': motion.forwardOneCell(); break;
            case 'r': motion.turnRight90();       break;
            case 'l': motion.turnLeft90();        break;
            default:  Serial.print("Unknown: "); Serial.println(commands[i]);
        }
        delay(200);
    }
        return true;
    }

    // 4.3 - autonomously explores the maze, maps walls, and solves the shortest path
    bool exploreAndSolveMaze() {
        //TODO
        return true;
    }



private:

    Maze& maze;
    MotionController& motion;

    uint8_t currRow;
    uint8_t currCol;
    Maze::Direction currDir;
};
}