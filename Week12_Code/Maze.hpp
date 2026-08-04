#pragma once

class Maze {

    public: 

    static const uint8_t SIZE = 9;
    enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3}

    // Constructor for maze class
    Maze() {
        //TODO
        // Initialise all cells with no known walls
    }

    // Given a cell and direction, sets a wall between the two neighbouring cells (for 4.3)
    void setWall(uint8_t row, uint8_t col, Direction dir, bool present) {
        //TODO
    }

    // Returns whether a wall exists at given point in maze
    bool hasWall(uint8_t row, uint8_t col, Direction dir) const {
        // TODO
    }

    // CV output sets known walls for given maze
    void setWalls() {
        //TODO
    }

    // Populates each cell with the distance to the goal cell
    void floodFill(uint8_t goalRow, uint8_t goalCol) {
        //TODO
    }

    // Returns a cell's distance to goal
    uint8_t getDistance(uint8_t row, uint8_t col) const {

    }

    // Determines the next direction the robot should travel from a given cell
    Direction nextStep(uint8_t row, uint8_t col, Direction currDir) const {
        //TODO
    }

    // Returns a string of the path commands the micromouse must follow to reach the goal
    String getPathCommands(uint8_t startRow, uint8_t startCol, Direction startDir) {
        //TODO
    }
}