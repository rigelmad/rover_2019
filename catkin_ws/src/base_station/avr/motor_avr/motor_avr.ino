#include <Wire.h>
#include "BSConsts.h"

#define encoder0PinA  2
#define encoder0PinB  3
#define leftDrive     5
#define rightDrive    6

unsigned long timer = 0;
unsigned long prevTime = 0;
volatile signed int encoder0Pos = 0;
volatile signed long totalEncPos = 0;

int prevPos = 0;
int numTicks = 0;

const int SPEED = 100; // Motor speed
const float MOTORRATIO = 823.1*4*2.0; //Enc CPR * GearRatio * Correction
const float SPDCONV = 600; //(sampling time)^-1 * 60
const float alph = .7;
const float beta = 1 - alph;
const float propGain = 2;
const float derivGain = .2;

float rpm = 0;
volatile int target = 0;
float prevError = 0.0;
float prevOut = 0.0;
float globalError = 0.0;

void setup() {
  //Encoder input pins (Need to be hardware interrupt pins)
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  //Analog(PWM) Outputs for directional driving (0-255)
  pinMode(leftDrive, OUTPUT);
  pinMode(rightDrive, OUTPUT);
  //Interrupt Indicator (Not really necessary)
  //pinMode(ledPin, OUTPUT);


  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  timer = millis();

  Wire.begin(MOTOR_AVR_ADDR);
  Wire.onReceive(handleReceive);
  Wire.onRequest(handleRequest);

}

void loop() {
  timer = millis();

  //for driving to angle
  driveAngle(target);

}

// On receive, change the target angle - !!!degrees
void handleReceive(int numBytes) {
  if (Wire.available()) {
    int firstByte = Wire.read(); // Read the byte as an int
    switch (firstByte) {
      case TARGET_CHANGE:
        break;
      default:
        break;
    }
  }

  int data = 0;
  int i = 0;
  while (Wire.available() && i < 2) { // loop through all (should be two byte = int)
    int recvByte = Wire.read();
    data = (data << 8) | recvByte; // Shift the stuff over 4 and insert the new char at the end
    i++;
  }
  target = data;
}

// Used to send encoders
void handleRequest() {
  float send = globalError;
  byte *dataPtr;
  dataPtr = (byte *) &send;     //convert float to array of bytes
  Wire.write(dataPtr,4);
}

void driveAngle(int target) {
  //PD controller?

  //"sample time"
  float deltaT = timer - prevTime;

  //error between current and target based on encoder position
  float error = (float)target - (float)totalEncPos / (MOTORRATIO / 360.0);
  globalError = error;

  int out;
  //implements controller transfer function: o = Kd*de/dt + Kp*e
  if (error > 5)
  {
    out = SPEED;
  }
  else if (error < -5)
  {
    out = -1 * SPEED;
  }
  else {
    out = 0;
  }

  //Checks direction.
  if (out > 0) {
    analogWrite(rightDrive, out);
    analogWrite(leftDrive, 0);
    delay(10);
  }
  else if (out < 0) {
    analogWrite(leftDrive, abs(out));
    analogWrite(rightDrive, 0);
    delay(10);
  }
  else {
    analogWrite(leftDrive, 0);
    analogWrite(rightDrive, 0);
  }
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
      totalEncPos = totalEncPos + 1;
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
      totalEncPos = totalEncPos - 1;
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
      totalEncPos = totalEncPos + 1;
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
      totalEncPos = totalEncPos - 1;
    }
  }
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // check channel B to see which way encoder is turning

  // Look for a high-to-low on channel B

  else {
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}
