/*
 * MTRN3100 — Encoder + Motor Calibration Sketch
 * ────────────────────────────────────────────────
 *
 *
 * Serial commands (type in Serial Monitor, press Enter):
 *   r → reset counts to 0
 *   f → spin both motors forward 1 second (tests wiring)
 *   b → spin both motors backward 1 second
 */

#define ENC1_A  2    // Left encoder interrupt pin  — confirmed
#define ENC1_B  4    // Left encoder direction pin  — confirmed
#define ENC2_A  3    // Right encoder interrupt pin — VERIFY
#define ENC2_B  5    // Right encoder direction pin — VERIFY

#define MOT1_PWM  9   // Left motor PWM   — confirmed
#define MOT1_DIR  10  // Left motor DIR   — confirmed
#define MOT2_PWM  11  // Right motor PWM  — VERIFY
#define MOT2_DIR  12  // Right motor DIR  — VERIFY

volatile long enc1_count = 0;
volatile long enc2_count = 0;

void ISR_enc1() {
    if (digitalRead(ENC1_B) == HIGH) enc1_count++;
    else enc1_count--;
}

void ISR_enc2() {
    if (digitalRead(ENC2_B) == HIGH) enc2_count++;
    else enc2_count--;
}

void setup() {
    Serial.begin(9600);

    pinMode(ENC1_A, INPUT_PULLUP);  pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);  pinMode(ENC2_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC1_A), ISR_enc1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), ISR_enc2, RISING);

    pinMode(MOT1_PWM, OUTPUT);  pinMode(MOT1_DIR, OUTPUT);
    pinMode(MOT2_PWM, OUTPUT);  pinMode(MOT2_DIR, OUTPUT);

    Serial.println("=== MTRN3100 Calibration ===");
    Serial.println("Commands: r=reset  f=forward  b=backward");
    Serial.println("Spin a wheel one full turn to find CPR.");
}

void loop() {
    Serial.print("Enc1(L)="); Serial.print(enc1_count);
    Serial.print("  Enc2(R)="); Serial.println(enc2_count);

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'r') {
            noInterrupts(); enc1_count = 0; enc2_count = 0; interrupts();
            Serial.println(">> Counts reset.");
        }
        if (cmd == 'f') {
            Serial.println(">> Motors forward 1s...");
            digitalWrite(MOT1_DIR, HIGH); analogWrite(MOT1_PWM, 150);
            digitalWrite(MOT2_DIR, HIGH); analogWrite(MOT2_PWM, 150);
            delay(1000);
            analogWrite(MOT1_PWM, 0); analogWrite(MOT2_PWM, 0);
            Serial.println(">> Done. Both should have moved forward.");
            Serial.println("   If one went backward: swap M+/M- on that motor terminal.");
        }
        if (cmd == 'b') {
            Serial.println(">> Motors backward 1s...");
            digitalWrite(MOT1_DIR, LOW); analogWrite(MOT1_PWM, 150);
            digitalWrite(MOT2_DIR, LOW); analogWrite(MOT2_PWM, 150);
            delay(1000);
            analogWrite(MOT1_PWM, 0); analogWrite(MOT2_PWM, 0);
            Serial.println(">> Done.");
        }
    }
    delay(200);
}
