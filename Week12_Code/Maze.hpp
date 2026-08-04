#pragma once

#include <stdint.h>
#include <stdio.h>

class Maze {

    public: 

    static const uint8_t SIZE = 9;
    static const uint8_t MAX_DIST = 255;
    enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

    // Constructor for maze class
    Maze() {
        
        // Initialise all cells with no known walls, max distance (255) and unvisited
        for (uint8_t i = 0; i < SIZE; i++) {
            for (uint8_t j = 0; j < SIZE; j++) {
                walls[i][j] = 0; 
                distance[i][j] = MAX_DIST;
                visited[i][j] = false;
            }
        }

        // Set boundaries for cells on top and bottom row
        for (uint8_t i = 0; i < SIZE; i++) {
            setWall(0, i, NORTH, true);
            setWall(SIZE - 1, i, SOUTH, true);
        }

        // Set boundaries for cells on left and right row
        for (uint8_t i = 0; i < SIZE; i++) {
            setWall(i, 0, WEST, true);
            setWall(i, SIZE - 1, EAST, true);
        }
    }

    // Given a cell and direction, sets a wall between the two neighbouring cells (for 4.3)
    void setWall(uint8_t row, uint8_t col, Direction dir, bool present) {
        // Set wall on this cell
        if (present) {
            walls[row][col] = walls[row][col] | (1 << dir);
        } else {
            walls[row][col] = walls[row][col] & ~(1 << dir);
        }

        // Find neighbouring cell if it exists
        int8_t nRow = row;
        int8_t nCol = col;
        Direction opposite;

        switch(dir) {
            case NORTH: nRow--; opposite = SOUTH; break;
            case SOUTH: nRow++; opposite = NORTH; break;
            case EAST: nCol++; opposite = WEST; break;
            case WEST: nCol--; opposite = EAST; break;
        }

        // check neighbour cell is within bounds
        if (nRow >= 0 && nRow < SIZE && nCol >= 0 && nCol < SIZE) {
            // set wall bit on neighbouring cell
            if (present) {
                walls[nRow][nCol] = walls[nRow][nCol] | (1 << opposite);
            } else {
                walls[nRow][nCol] = walls[nRow][nCol] & ~(1 << opposite);
            }
        }
    }

    // Returns whether a wall exists at given point in maze
    bool hasWall(uint8_t row, uint8_t col, Direction dir) const {
        uint8_t wall = walls[row][col] & (1 << dir);
        return (wall != 0);
    }

    // CV output sets known walls for given maze
    void setWalls() {
        //TODO
    }

    // Populates each cell with the distance to the goal cell
    void floodFill(uint8_t goalRow, uint8_t goalCol) {
        // mark goal as distance 0
        distance[goalRow][goalCol] = 0;
        uint8_t currExploredVal = 0;
        bool mazeChanged = true;

        // run loop until maze isn't changed
        while (mazeChanged == true) {

            mazeChanged = false;

            for (uint8_t r = 0; r < SIZE; r++) {
                for (uint8_t c = 0; c < SIZE; c++) {
                    
                    // find cell at current explored level and update neighbour values
                    if (distance[r][c] == currExploredVal && floodFillUpdateNeighbours(r, c, currExploredVal)) {
                        mazeChanged = true; 
                    }
                }
            }
            currExploredVal++;
        }
    }

    // Returns a cell's distance to goal
    uint8_t getDistance(uint8_t row, uint8_t col) const {
        return distance[row][col];
    }

    // Determines the next direction the robot should travel from a given cell
    Direction nextStep(uint8_t row, uint8_t col, Direction currDir) const {
        //TODO
    }

    // Writes path commands into out with a null terminator, returns the number of commands written
    // not including null terminator
    uint8_t getPathCommands(uint8_t startRow, uint8_t startCol, Direction startDir) {
        //TODO
    }

    void print() const {
        for (uint8_t r = 0; r < SIZE; r++) {

            // Top wall row for this row of cells)
            for (uint8_t c = 0; c < SIZE; c++) {
                printf("+");
                printf(hasWall(r, c, NORTH) ? "---" : "   ");
            }
            printf("+\n");

            // Cell content row: west wall, distance, east wall,
            for (uint8_t c = 0; c < SIZE; c++) {
                printf(hasWall(r, c, WEST) ? "|" : " ");

                 // unreached cell shown as a dot
                if (distance[r][c] == MAX_DIST) {
                    printf(" . ");  
                } else {
                    printf("%2d ", distance[r][c]);
                }
            }
            printf(hasWall(r, SIZE - 1, EAST) ? "|\n" : " \n");
        }

        // Bottom border 
        for (uint8_t c = 0; c < SIZE; c++) {
            printf("+");
            printf(hasWall(SIZE - 1, c, SOUTH) ? "---" : "   ");
        }
        printf("+\n");
    }

    private: 

        // checks neighbouring cells, and updates their distance if unvisited and no wall between them
        bool floodFillUpdateNeighbours(uint8_t row, uint8_t col, uint8_t currExploredVal) {
            bool updatedMaze = false;

            // check walls in each direction
            for (uint8_t d = 0; d < 4; d++) {
                Direction dir = static_cast<Direction>(d);
                // if no wall between cells, and the neighbour cell hasn't been explored, 
                // set distance of that cell to currExploredVal + 1
                if (!hasWall(row, col, dir)) {
                    int8_t nRow = row;
                    int8_t nCol = col;

                    switch(dir) {
                        case NORTH: nRow--; break;
                        case SOUTH: nRow++; break;
                        case EAST: nCol++;  break;
                        case WEST: nCol--;  break;
                    }
                    
                    if (nRow >= 0 && nRow < SIZE && nCol >= 0 && nCol < SIZE && distance[nRow][nCol] == MAX_DIST) {
                        distance[nRow][nCol] = currExploredVal + 1;
                        updatedMaze = true;
                    }
                }
            }

            return updatedMaze;
        }

        // 4 bit mask: bit 0 north wall, bit 1 east, bit 2 south, bit 3 west
       uint8_t walls[SIZE][SIZE];
       uint8_t distance[SIZE][SIZE];
       bool visited[SIZE][SIZE]; // for 4.3
       
       uint8_t goalRow; 
       uint8_t goalCol;

};