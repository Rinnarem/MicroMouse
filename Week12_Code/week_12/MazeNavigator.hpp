/**
 * Uses Maze and MotionController class to navigate the maze
 */

#pragma once

#include "Maze.hpp"
#include "MotionController.hpp"

namespace mtrn3100 {

class MazeNavigator {

public:

    // display is optional -- pass nullptr (or use the other constructor) for
    // tasks that don't need the OLED (4.1, 4.2, 4.4 all reuse this class via
    // executePath, which never touches display).
    MazeNavigator(Maze& maze, MotionController& motion): maze(maze), motion(motion) {}

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
    bool exploreAndSolveMaze(uint8_t startRow, uint8_t startCol, Maze::Direction startDir) {

        // These were never set before the loop below used them -- executePath
        // sets them at the top of every run, exploreAndSolveMaze needs the
        // same thing since it drives currRow/currCol/currDir directly too.
        currRow = startRow;
        currCol = startCol;
        currDir = startDir;

        // Explore the maze
        bool fullyExplored = false;
        maze.resetCompletionStatus();
        maze.setVisited(currRow, currCol, true);

        while (fullyExplored == false) {

            // update walls surrounding cell
            bool changed = updateSurroundingWalls();

            // update maze
            if (changed) {
                maze.floodFill();
            }

            // determine next step
            turnToNextCell();
            motion.forwardOneCell();
            updateLocationForward();

            maze.setVisited(currRow, currCol, true);

            // update fully explored and %completed status
            fullyExplored = maze.isFullyExplored();
        }
        
        // Get string of commands to navigate back to start position
        // navigate back to start position
        uint8_t goalRow = maze.getGoalRow();
        uint8_t goalCol = maze.getGoalCol();
        // Maze has no setTarget() -- setGoal() is what recomputes floodFill()
        // for a new target cell, which is exactly what's needed to route back
        // to the start. The real goal is restored below once we're there.
        maze.setGoal(startRow, startCol);

        // go back to start position
        const uint8_t MAX_PATH_LEN = 100;
        char path[MAX_PATH_LEN];
        uint8_t pathLen = maze.getPathCommands(currRow, currCol, currDir, path, MAX_PATH_LEN);
        executePath(path, currRow, currCol, currDir);

        // correct start orientation
        while (currDir != startDir) {
            motion.turnLeft90();
            currDir = directionLeftTurn();
        }

        // set target back to goal and get path
        maze.setGoal(goalRow, goalCol);

        // execute shortest path to goal
        pathLen = maze.getPathCommands(currRow, currCol, currDir, path, MAX_PATH_LEN);
        executePath(path, currRow, currCol, currDir);

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
                currDir = directionLeftTurn();
                break;

            case 'r':
                success = motion.turnRight90();
                currDir = directionRightTurn();
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

    Maze::Direction directionRightTurn() {
        switch(currDir) {
            case Maze::NORTH:
                return Maze::EAST;
                break;
            case Maze::SOUTH: 
                return Maze::WEST;
                break;
            case Maze::WEST:
                return Maze::NORTH;
                break;
            case Maze::EAST:
                return Maze::SOUTH;
                break;
        }
    }

    Maze::Direction directionLeftTurn() {
        switch(currDir) {
            case Maze::NORTH:
                return Maze::WEST;
                break;
            case Maze::SOUTH: 
                return Maze::EAST;
                break;
            case Maze::WEST:
                return Maze::SOUTH;
                break;
            case Maze::EAST:
                return Maze::NORTH;
                break;
        }
    }

    Maze::Direction directionBehind() {
        switch(currDir) {
            case Maze::NORTH:
                return Maze::SOUTH;
                break;
            case Maze::SOUTH: 
                return Maze::NORTH;
                break;
            case Maze::WEST:
                return Maze::EAST;
                break;
            case Maze::EAST:
                return Maze::WEST;
                break;
        }
    }

    // Updates the maze and sets walls in current cell from lidar readings, 
    // if there are already walls there doesn't check to update,
    //  or if the cell on the other side has been visited
    bool updateSurroundingWalls() {
        bool updated = false;

        motion.scanLidars();
        bool hasWallLeft = motion.hasWallLeft();
        bool hasWallFront = motion.hasWallFront();
        bool hasWallRight = motion.hasWallRight();

        // loop through each direction robot is facing
        for (int i = 0; i < 3; i++) {
            Maze::Direction dir = getDirectionForUpdating(i);

            if (!maze.hasWall(currRow, currCol, dir)) {
                // check lidar in that direction, and if it has a wall update the maze
                if (i == 0 && hasWallLeft) {
                    maze.setWall(currRow, currCol, dir, true);
                    updated = true;
                } else if (i == 1 && hasWallFront) {
                    maze.setWall(currRow, currCol, dir, true);
                    updated = true;
                } else if (i == 2 && hasWallRight) {
                    maze.setWall(currRow, currCol, dir, true);
                    updated = true;
                }
                    
            }

        }
        return updated;
    }
    

    // Returns cardinal direction from maze orientation, 
    // i = 0 is Robots left
    // i = 1 ahead, i = 2 to right, i = 3 behind
    Maze::Direction getDirectionForUpdating(int i) {
        if (i == 0) {
            return directionLeftTurn();
        } else if (i == 1) {
            return currDir;
        } else if (i == 2) {
            return directionRightTurn();
        } else {
            return directionBehind();
        }
    }

    // Turns to neighbour cell which is unvisited
    // with no wall and closest distance to target
    void turnToNextCell() {
        Maze::Direction targetDir = maze.nextCellToExplore(currRow, currCol);

        if (directionRightTurn() == targetDir) {
            motion.turnRight90();
            currDir = targetDir;
        } else {
            while (currDir != targetDir) {
                motion.turnLeft90();
                currDir = directionLeftTurn();
            }

        }

    }

    Maze& maze;
    MotionController& motion;

    uint8_t currRow;
    uint8_t currCol;
    Maze::Direction currDir;
    
};
}