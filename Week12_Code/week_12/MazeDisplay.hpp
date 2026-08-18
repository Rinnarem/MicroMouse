/**
 * OLED (CE09493 / SSD1306 128x64, I2C addr 0x3C) visualisation for Task 4.3.
 *
 * Uses U8g2's 1-page buffer variant rather than Adafruit_SSD1306. A full
 * SSD1306 framebuffer is WIDTH*HEIGHT/8 = 1024 bytes -- half the Uno's 2KB
 * SRAM -- which is a bad trade when Maze, MotionController, the PID structs,
 * and everything else in this project are already competing for that same
 * budget. The page-buffer variant redraws the screen in thin strips (about
 * 128 bytes at a time) via firstPage()/nextPage(), so render() costs more
 * CPU time per call but a tiny fraction of the RAM.
 *
 * Wiring: OLED shares the existing I2C bus (SDA/SCL, already used by the
 * MPU6050) -- just add VCC/GND. No extra pins needed. Call oled.begin()
 * AFTER Wire.begin() has run once (initHardware() already does this for the
 * IMU), and add U8g2lib to your libraries if it isn't installed yet.
 *
 * Flash footprint: two things below are deliberately avoided because on
 * AVR they are surprisingly expensive relative to what they buy you --
 *   - Only ONE small font is loaded (u8g2_font_4x6_tr). Each font's glyph
 *     table is baked into flash independently, "_tf" ("full range", 0-255)
 *     fonts carry roughly twice the glyph data of "_tr" ("restricted",
 *     printable ASCII only) for no benefit here, and a smaller pixel size
 *     (4x6 vs 5x7) means a smaller bitmap per glyph again -- we only ever
 *     print a couple of digits and '%'.
 *   - No sprintf/snprintf. On AVR, ANY use of *printf pulls in avr-libc's
 *     full vfprintf implementation -- including its floating-point
 *     formatter, even though nothing here prints a float -- which costs
 *     roughly 1.5-2 KB of flash by itself. u8g2's Arduino-style print()
 *     (it extends Print, same as Serial) formats integers directly with
 *     none of that baggage.
 */

#pragma once

#include <U8g2lib.h>
#include "Maze.hpp"

namespace mtrn3100 {

class MazeDisplay {
public:
    MazeDisplay() : oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE) {}

    void begin() {
        oled.begin();
        // 4x6 instead of 5x7: same restricted ASCII range, smaller glyph
        // bitmaps, smaller font table in flash. Legible enough for a
        // status readout.
        oled.setFont(u8g2_font_4x6_tr);
        oled.setFontPosTop();
    }

    // Redraws the whole screen: known walls, visited cells, robot position,
    // and completion %. Call this once per explored cell (see
    // MazeNavigator::exploreAndSolveMaze) -- NOT every control-loop tick.
    // Each call is several page redraws over I2C; doing it every PID
    // iteration would steal time from the heading/distance loops the same
    // way an unbuffered Serial.print would.
    void render(const Maze& maze, uint8_t robotRow, uint8_t robotCol) {
        oled.firstPage();
        do {
            drawGrid(maze, robotRow, robotCol);
            drawCompletion(maze);
        } while (oled.nextPage());
    }

private:
    // 9x9 maze at 7px/cell = 63x63px square, left-aligned. Leaves a ~65px
    // strip on the right for the completion %.
    static const uint8_t CELL_PX = 7;
    static const uint8_t GRID_X0 = 0;
    static const uint8_t GRID_Y0 = 0;

    void drawGrid(const Maze& maze, uint8_t robotRow, uint8_t robotCol) {
        for (uint8_t r = 0; r < Maze::SIZE; r++) {
            for (uint8_t c = 0; c < Maze::SIZE; c++) {
                int16_t x0 = GRID_X0 + c * CELL_PX;
                int16_t y0 = GRID_Y0 + r * CELL_PX;

                if (maze.hasWall(r, c, Maze::NORTH)) oled.drawHLine(x0, y0, CELL_PX + 1);
                if (maze.hasWall(r, c, Maze::WEST))  oled.drawVLine(x0, y0, CELL_PX + 1);
                if (maze.hasWall(r, c, Maze::SOUTH)) oled.drawHLine(x0, y0 + CELL_PX, CELL_PX + 1);
                if (maze.hasWall(r, c, Maze::EAST))  oled.drawVLine(x0 + CELL_PX, y0, CELL_PX + 1);

                // shade visited cells so the explored region is visible at a glance
                if (maze.isVisited(r, c) && CELL_PX > 3) {
                    oled.drawBox(x0 + 2, y0 + 2, CELL_PX - 3, CELL_PX - 3);
                }
            }
        }

        // hollow frame marks the robot's current cell over the visited fill
        int16_t rx = GRID_X0 + robotCol * CELL_PX;
        int16_t ry = GRID_Y0 + robotRow * CELL_PX;
        oled.setDrawColor(2); // XOR, so it's visible whether or not the cell is shaded
        oled.drawFrame(rx + 1, ry + 1, CELL_PX - 1, CELL_PX - 1);
        oled.setDrawColor(1);
    }

    void drawCompletion(const Maze& maze) {
        uint8_t pct = maze.completionPercentage();

        // Just the number -- the label and progress bar were dropped to
        // claw back flash. u8g2 objects derive from Arduino's Print (same
        // as Serial), so print(uint8_t) formats the number directly with
        // no sprintf/vfprintf involved.
        oled.setCursor(70, 0);
        oled.print(pct);
        oled.print(F("%"));
    }

    U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled;
};

}