#pragma once
/**
 * Maps the maze and finds shortest path
 * 
 * Claude was used to help write and debug this code
 */

#include <stdint.h>
#include <stdio.h>

namespace mtrn3100 {
class Maze {

    public: 

    static const uint8_t SIZE = 9;
    static const uint8_t MAX_DIST = 255;
    enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

    // Constructor for maze class
    Maze(uint8_t goalRow, uint8_t goalCol) : goalRow(goalRow), goalCol(goalCol) {
        
        // Initialise all cells with no known walls and max distance
        for (uint8_t i = 0; i < SIZE; i++) {
            for (uint8_t j = 0; j < SIZE; j++) {
                walls[i][j] = 0;
                distance[i][j] = MAX_DIST;
            }
        }
        // ...and all cells unvisited (packed one bit per cell)
        for (uint8_t i = 0; i < sizeof(visited); i++) visited[i] = 0;
        cellsVisited = 0;

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

        // set boundaries for corner cells - top left
        setWall(0, 1, EAST, true);
        setWall(0, 1, SOUTH, true);
        setWall(1, 0, SOUTH, true);
        setWall(1, 0, EAST, true);

        // top right
        setWall(0, SIZE - 2, WEST, true);
        setWall(0, SIZE - 2, SOUTH, true);
        setWall(1, SIZE - 1,  SOUTH, true);
        setWall(1, SIZE - 1, WEST, true);

        // bottom left
        setWall(SIZE - 2, 0, EAST, true);
        setWall(SIZE - 2, 0, NORTH, true);
        setWall(SIZE - 1, 1, NORTH, true);
        setWall(SIZE -1 , 1, EAST, true);

        // bottom right
        setWall(SIZE -2, SIZE -1 , WEST, true);
        setWall(SIZE -2, SIZE - 1, NORTH, true);
        setWall(SIZE -1, SIZE -2, WEST, true);
        setWall(SIZE -1, SIZE -2, NORTH, true);
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
    void floodFill() {
        // set all cells to max dist
        for (int i = 0; i < SIZE; i++) {
            for (int j = 0; j < SIZE; j++) {
                distance[i][j] = MAX_DIST;
            }
        }

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

    // --- visited flags, one bit per cell (for 4.3) ---------------------
    // Stored packed rather than as bool[9][9]: the Uno has only 2 KB of SRAM
    // and a byte-per-cell array costs 81 of them for one bit of information.
    bool isVisited(uint8_t row, uint8_t col) const {
        uint8_t idx = row * SIZE + col;
        return (visited[idx >> 3] >> (idx & 7)) & 1;
    }

    void setVisited(uint8_t row, uint8_t col, bool value) {

        // update num cells visited
        if (value == true && !isVisited(row, col)) {
            cellsVisited++;
        }

        uint8_t idx = row * SIZE + col;
        if (value) visited[idx >> 3] |=  (1 << (idx & 7));
        else       visited[idx >> 3] &= ~(1 << (idx & 7));

        
    }

    // Determines the next direction the robot should travel from a given cell for shortest path
    Direction nextStep(uint8_t row, uint8_t col, Direction currDir) const {
        
        Direction best = compareNextSteps(row, col, currDir, NORTH, EAST);
        best = compareNextSteps(row, col, currDir, best, SOUTH);
        best = compareNextSteps(row, col, currDir, best, WEST);

        return best;
    }

    Direction compareNextSteps(uint8_t row, uint8_t col, Direction currDir, Direction a, Direction b) const {

        // return direction that doesnt have wall
        if (hasWall(row, col, a)) {return b;}

        if (hasWall(row, col, b)) {return a;}

        // return direction closer to goal
        uint8_t currDist = getDistance(row, col);

        uint8_t rowA = getNeighbourRow(row, a);
        uint8_t colA = getNeighbourCol(col, a);
        uint8_t distA = getDistance(rowA, colA);
        
        if (distA > currDist) {return b;}
        
        uint8_t rowB = getNeighbourRow(row, b);
        uint8_t colB = getNeighbourCol(col, b);

        uint8_t distB = getDistance(rowB, colB);

        if (distB > currDist) {return a;}

        // return same direction if both are closer to goal (less turning)
        return (a == currDir) ? a : b;
    }

    // Writes path commands into out with a null terminator, returns the number of commands written
    // not including null terminator
    uint8_t getPathCommands(uint8_t startRow, uint8_t startCol, Direction startDir,
                         char* out, uint8_t maxLen) {
        uint8_t row = startRow, col = startCol;
        Direction dir = startDir;
        uint8_t count = 0;

        if (distance[row][col] == MAX_DIST) {
            out[0] = '\0';
            return 0;   // goal not reachable with what's currently mapped
        }

        while (distance[row][col] != 0 && count < maxLen - 1) {
            Direction target = nextStep(row, col, dir);

            if (target == dir) {
                out[count++] = 'f';
                switch (dir) {
                    case NORTH: row--; break;
                    case SOUTH: row++; break;
                    case EAST:  col++; break;
                    case WEST:  col--; break;
                }
            } else if (target == (Direction)((dir + 1) % 4)) {
                out[count++] = 'r';
                dir = target;
            } else if (target == (Direction)((dir + 3) % 4)) {
                out[count++] = 'l';
                dir = target;
            } else {
                // target is directly behind us — two turns
                out[count++] = 'r';
                if (count < maxLen - 1) out[count++] = 'r';
                dir = target;
            }
        }

        out[count] = '\0';
        return count;
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

    // determines which way the robot should go next to explore the maze
    Direction nextCellToExplore(uint8_t row, uint8_t col) {

        // compare the four candidates for next cell
        Direction best = compareCandidates(row, col, NORTH, EAST);
        best = compareCandidates(row, col, best, SOUTH);
        best = compareCandidates(row, col, best, WEST);
        
        return best;     
    }

    // compares two directions and determines which one is better for the next step in exploration
    Direction compareCandidates(uint8_t row, uint8_t col, Direction a, Direction b) {

        // return a if b has wall (this will also return a if they both have walls)
        if (hasWall(row, col, b)) {return a;}

        // return b if a has wall
        if (hasWall(row, col, a)) {return b;}


        // return a if b has been visited (will also return a if its been visited)
        uint8_t bRow = getNeighbourRow(row, b);
        uint8_t bCol = getNeighbourCol(col, b);
        if (isVisited(bRow, bCol)) {return a;}
        
        // return b if a has been visited
        uint8_t aRow = getNeighbourRow(row, a);
        uint8_t aCol = getNeighbourCol(col, a);

        if (isVisited(aRow, aCol)) {return b;}

        // return lower of two distances
        uint8_t distA = getDistance(aRow, aCol);
        uint8_t distB = getDistance(bRow, bCol);

        return (distA <= distB) ? a : b;

    }

    uint8_t getNeighbourRow(uint8_t row, Direction dir) const {

        if (dir == EAST || dir == WEST) {
            return row;
        } else if (dir == NORTH) {
            return row - 1;
        } else {
            return row + 1;
        }
    }

    uint8_t getNeighbourCol(uint8_t col, Direction dir) const {

        if (dir == SOUTH || dir == NORTH) {
            return col;
        } else if (dir == WEST) {
            return col - 1;
        } else {
            return col + 1;
        }
    }

    // Returns true if all cells have been marked as visited
    bool isFullyExplored() const {
        return (cellsVisited == SIZE * SIZE) ? true : false;

    }

    void resetCompletionStatus() {
        cellsVisited = 0;
    }

    uint8_t getNumCellsVisited() const {
        return cellsVisited;
    }

    // Returns completion percentage as an int between 0 - 100
    uint8_t completionPercentage() const {
        float result = float(cellsVisited) / (SIZE * SIZE);

        return static_cast<uint8_t>(result * 100);
    }

    uint8_t getGoalRow() const {
        return goalRow;
    }

    uint8_t getGoalCol() const {
        return goalCol;
    }

    // Sets new target in maze and updates flood fill distances
    void setGoal( uint8_t row, uint8_t col) {
        goalRow = row;
        goalCol = col;

        floodFill();
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
       // 81 cells, one bit each -> 11 bytes instead of 81. Use
       // isVisited()/setVisited() rather than touching this directly.
       uint8_t visited[(SIZE * SIZE + 7) / 8]; // for 4.3
       
       uint8_t goalRow; 
       uint8_t goalCol;
       uint8_t cellsVisited;

};

}