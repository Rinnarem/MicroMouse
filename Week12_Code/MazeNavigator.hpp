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
        //TODO
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