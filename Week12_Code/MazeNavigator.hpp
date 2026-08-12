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
        for (int i = 0; path[i] != '\0'; i++) {
            bool success = executeCommand(path[i]);

            if (!success) {
                Serial.print("Move failed at command "); 
                Serial.print(i);
                Serial.print(" ('");
                Serial.print(path[i]);
                Serial.println("')");
                return false;
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

    bool executeCommand(char cmd) {
        bool success = true;

        switch (cmd) {
            case 'f' :
                success = motion.forwardOneCell();
                break;
            
            case 'l':
                success = motion.turnLeft90();
                break;

            case 'r':
                success = motion.turnRight90();
                break;
            default:
                Serial.print("Unknown command");
                return false;
        }
    }

    Maze& maze;
    MotionController& motion;

    uint8_t currRow;
    uint8_t currCol;
    Maze::Direction currDir;
};
}