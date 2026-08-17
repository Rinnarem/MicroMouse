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
    bool executePath (const char* path, uint8_t startRow, uint8_t startCol, mtrn3100::Maze::Direction startDir) {

        currRow = startRow;
        currCol = startCol;
        currDir = startDir;

        for (int i = 0; path[i] != '\0'; i++) {
            bool success = executeCommand(path[i]);

            if (!success) {
                Serial.print(F("Move failed at command ")); 
                Serial.print(i);
                Serial.print(F(" ('"));
                Serial.print(path[i]);
                Serial.println(F("')"));
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
                updateLocationForward();
                break;
            
            case 'l':
                success = motion.turnLeft90();
                updateLocationLeftTurn();
                break;

            case 'r':
                success = motion.turnRight90();
                updateLocationRightTurn();

                break;
            default:
                Serial.print(F("Unknown command"));
                return false;
        }
        return success;
    }

    // Updates the cell location going forward one cell based on direction
    void updateLocationForward() {
        
        switch(currDir) {
            case Maze::NORTH:
                currRow--;
                break;
            case Maze::SOUTH: 
                currRow++;
                break;
            case Maze::WEST:
                currCol--;
                break;
            case Maze::EAST:
                currCol++;
                break;
        }
    }

    void updateLocationRightTurn() {
        switch(currDir) {
            case Maze::NORTH:
                currDir = Maze::EAST;
                break;
            case Maze::SOUTH: 
                currDir = Maze::WEST;
                break;
            case Maze::WEST:
                currDir = Maze::NORTH;
                break;
            case Maze::EAST:
                currDir = Maze::SOUTH;
                break;
        }
    }

    void updateLocationLeftTurn() {
        switch(currDir) {
            case Maze::NORTH:
                currDir = Maze::WEST;
                break;
            case Maze::SOUTH: 
                currDir = Maze::EAST;
                break;
            case Maze::WEST:
                currDir = Maze::SOUTH;
                break;
            case Maze::EAST:
                currDir = Maze::NORTH;
                break;
        }
    }

    Maze& maze;
    MotionController& motion;

    uint8_t currRow;
    uint8_t currCol;
    Maze::Direction currDir;
    
};
}