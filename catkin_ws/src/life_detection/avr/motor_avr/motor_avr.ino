#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include "LDConsts.h"

#define encoder0PinA  2
#define encoder0PinB  3
#define enableLPin    7
#define enableRPin    8
#define leftDrive     5
#define rightDrive    6

const int SPEED = 75;

unsigned long timer = 0;
unsigned long prevTime = 0;
volatile signed int encoder0Pos = 0;
volatile signed long totalEncPos = 0;

int prevPos = 0;
int numTicks = 0;

volatile int intendedTarget = 0;

bool respondToReq = false;
bool shouldSendEncoder = false;

void setup() {

  //Encoder input pins (Need to be hardware interrupt pins)
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  //pins to enable directions -- always left HIGH to enable 2 dir driving
  pinMode(enableRPin, OUTPUT);
  pinMode(enableLPin, OUTPUT);

  //Analog(PWM) Outputs for directional driving (0-255)
  pinMode(leftDrive, OUTPUT);
  pinMode(rightDrive, OUTPUT);

  // setting direction pins high
  digitalWrite(enableLPin, HIGH);
  digitalWrite(enableRPin, HIGH);

  Wire.begin(MOTOR_AVR_ADDR);
  Wire.onReceive(handleSimpleReceive);
  Wire.onRequest(handleRequest);

}

void loop() {

  //for driving to speed
  driveSpeed(intendedTarget);

}


void driveSpeed(int target) {
  if (target > 0)
  {
    //Corrects output signal to controller limits
    if (target <= 10)
      target = 10;
    else if (target >= 255)
      target = 254;
    analogWrite(rightDrive, target);
    analogWrite(leftDrive, 0);
  }

  else if (target < 0)
  {
    //Corrects output signal to controller limits
    if (target >= -10)
      target = -10;
    else if (target <= -255)
      target = -254;
    analogWrite(leftDrive, abs(target));
    analogWrite(rightDrive, 0);
  }
  else
  {
    analogWrite(leftDrive, 0);
    analogWrite(rightDrive, 0);
  }
}

void handleSimpleReceive(int numBytes) {
  if (Wire.available()) {
    int firstByte = Wire.read();
    switch (firstByte) {
      case 0:
        intendedTarget = 0;
        break;
      case 1:
        intendedTarget = 1 * SPEED;
        break;
      case 2:
        intendedTarget = -1 * SPEED;
        break;
      default: // Garbage data yields a stop
        intendedTarget = 0;
        break;
    }
  }
}

// On receive, change the target angle - !!!degrees
void handleReceive(int numBytes) {
  if (Wire.available()) {
    int firstByte = Wire.read(); // Read the byte as an int
    switch (firstByte) {
      case TARGET_CHANGE:
        break;
      case ENCODER_REQ:
        shouldSendEncoder = true;
        return;
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
  intendedTarget = data;

}

// Used to send encoders
void handleRequest() {
  int send = intendedTarget;
  Wire.write(highByte(send));
  Wire.write(lowByte(send));
}
