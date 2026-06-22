#define motorA 5 //Input 3
#define motorB 6 //Input 4
#define motorC 8 //Input 1
#define motorD 9 //Input 2
// Odd inputs control PWM
// Even inputs control direction
#define encoderPin1 2
#define encoderPin2 3

volatile long encoderCount = 0;
volatile int lastEncoded = 0;

const int countsPerRevolution = 48;  // Adjust to encoder

void setup() {
  Serial.begin (9600);
  pinMode(motorA,OUTPUT);
  pinMode(motorB,OUTPUT);
  pinMode(motorC,OUTPUT);
  pinMode(motorD,OUTPUT);

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
}

void loop() {
   analogWrite(motorA,127);
   digitalWrite(motorB,LOW);
   digitalWrite(motorC,LOW);
   analogWrite(motorD,127);
   long count;

   noInterrupts();
   count = encoderCount;
   interrupts();

   long rotations = count / countsPerRevolution;

   Serial.print("Count = ");
   Serial.print(count);

   Serial.print("   Rotations = ");
   Serial.println(rotations);

   delay(100);
   /*analogWrite(motorA,127);
   digitalWrite(motorB,HIGH);
   digitalWrite(motorC,HIGH);
   analogWrite(motorD,127);
   delay(1000);*/
}

void updateEncoder() {

  int MSB = digitalRead(encoderPin1);
  int LSB = digitalRead(encoderPin2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 ||
      sum == 0b0010 || sum == 0b1011)
    encoderCount++;

  if (sum == 0b1110 || sum == 0b0111 ||
      sum == 0b0001 || sum == 0b1000)
    encoderCount--;

  lastEncoded = encoded;
}