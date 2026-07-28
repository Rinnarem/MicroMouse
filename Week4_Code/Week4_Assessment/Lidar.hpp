#pragma once

#include <Arduino.h>

namespace mtrn3100 {

// The encoder class counts pulses from a Hall-effect quadrature encoder.
// encoder1_pin → interrupt pin (must be D2 or D3 on Arduino Nano)
// encoder2_pin → any digital pin; used to determine rotation direction
//
// How direction detection works (quadrature encoding):
//   When encoder1 fires (rising edge), read encoder2:
//     encoder2 = HIGH → shaft rotating forward  → count++
//     encoder2 = LOW  → shaft rotating backward → count--
class Lidar {
  public:
    Lidar(uint8_t com_pin) : com_pin(com_pin) {}

    static int getReading() {
      
    }

    static void setStopDist(int distance) {

    }

    static void 

  private:
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

  private:
    const uint8_t com_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction = 0;
    float position = 0;

    // ★ SET THIS BEFORE USE ★
    // counts_per_revolution = encoder pulses per full wheel rotation
    // For DFROBOT FIT083: measure experimentally (see calibration guide)
    // Typical values: 360–600
    uint16_t counts_per_revolution = 360;  // TODO: calibrate for your motor

    volatile long count = 0;
    uint32_t prev_time = 0;
    bool read = false;

}