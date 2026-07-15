/*
 * MTRN3100 — Week 8 Sensor Test
 * ─────────────────────────────
 * Run first. Serial Monitor at 115200 baud.
 *
 * SENSOR LAYOUT after top-plate 180° flip:
 *   FRONT_MM (A0, 0x54) = FRONT-facing sensor
 *   RIGHT_MM (A1, 0x56) = RIGHT-facing sensor
 *   LEFT_MM  (A2, 0x58) = LEFT-facing sensor
 *
 * VERIFY:
 *   1. Rotate robot CW       → YAW goes NEGATIVE           ✓ (already confirmed)
 *   2. Hand in front         → FRONT_MM drops              ← check this
 *   3. Hand on right side    → RIGHT_MM drops              ← check this
 *   4. Hand on left side     → LEFT_MM drops               ← check this
 *   5. Push robot forward    → ENC_L and ENC_R both increase ✓ (already confirmed)
 *
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>

#define LIDAR_FRONT_EN  A2   // Front sensor  (TOF3GP0 in schematic)
#define LIDAR_RIGHT_EN  A1   // Right sensor  (TOF2GP0 in schematic)
#define LIDAR_LEFT_EN   A0   // Left sensor   (TOF1GP0 in schematic)

#define ENC1_A  2
#define ENC1_B  4
#define ENC2_A  3
#define ENC2_B  5

MPU6050 mpu(Wire);
VL6180X lidarFront;
VL6180X lidarRight;
VL6180X lidarLeft;

volatile long enc_L = 0;
volatile long enc_R = 0;

void ISR_left()  { enc_L += (digitalRead(ENC1_B) == HIGH) ?  1 : -1; }
void ISR_right() { enc_R += (digitalRead(ENC2_B) == HIGH) ?  1 : -1; }

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC1_A), ISR_left,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), ISR_right, RISING);

    // Pull ALL three enables LOW so every sensor resets to default address 0x29
    pinMode(LIDAR_FRONT_EN, OUTPUT);
    pinMode(LIDAR_RIGHT_EN, OUTPUT);
    pinMode(LIDAR_LEFT_EN,  OUTPUT);
    digitalWrite(LIDAR_FRONT_EN, LOW);
    digitalWrite(LIDAR_RIGHT_EN, LOW);
    digitalWrite(LIDAR_LEFT_EN,  LOW);
    delay(20);

    // Enable FRONT → assign 0x54
    digitalWrite(LIDAR_FRONT_EN, HIGH); delay(50);
    lidarFront.init(); lidarFront.configureDefault();
    lidarFront.setTimeout(250); lidarFront.setAddress(0x54);
    Serial.println("Front LiDAR OK (A0, 0x54).");

    // Enable RIGHT → assign 0x56
    digitalWrite(LIDAR_RIGHT_EN, HIGH); delay(50);
    lidarRight.init(); lidarRight.configureDefault();
    lidarRight.setTimeout(250); lidarRight.setAddress(0x56);
    Serial.println("Right LiDAR OK (A1, 0x56).");

    // Enable LEFT → assign 0x58
    digitalWrite(LIDAR_LEFT_EN, HIGH); delay(50);
    lidarLeft.init(); lidarLeft.configureDefault();
    lidarLeft.setTimeout(250); lidarLeft.setAddress(0x58);
    Serial.println("Left LiDAR OK (A2, 0x58).");

    byte status = mpu.begin();
    if (status != 0) { Serial.println("IMU FAILED"); while(1); }

    // Battery bug fix (edStem #66) — force ±500 deg/s
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Serial.println("Calibrating IMU — keep still...");
    delay(1000);
    mpu.calcOffsets(true, true);

    Serial.println("\nExpected: CW rotation → YAW negative\n");
    Serial.println("YAW(deg) |FRONT_MM |RIGHT_MM |LEFT_MM |ENC_L |ENC_R");
    Serial.println("────────────────────────────────────────────────────────────────────────");
}

void loop() {
    mpu.update();
    Serial.print(mpu.getAngleZ(), 1);                      Serial.print("\t| ");
    Serial.print(lidarFront.readRangeSingleMillimeters());  Serial.print("\t| ");
    Serial.print(lidarRight.readRangeSingleMillimeters());  Serial.print("\t| ");
    Serial.print(lidarLeft.readRangeSingleMillimeters());   Serial.print("\t| ");
    Serial.print(enc_L);                                    Serial.print("\t| ");
    Serial.println(enc_R);
    delay(100);
}
