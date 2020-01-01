#include <Arduino.h>
#include <Wire.h>
#include "LDConsts.h"

#define LA_FWD 5
#define LA_REV 6
#define ACT_POT A0

volatile int globalTarget = 0;
int act_pos = 0;
const int ACT_HIGH_THRESH = 800; // The highest the position can go
const int ACT_LOW_THRESH = 20; // The lowest the position can go

void setup() {

  //Analog(PWM) Outputs for directional driving (0-255)
  pinMode(LA_FWD, OUTPUT);
  pinMode(LA_REV, OUTPUT);

  Wire.begin(ACTUATOR_AVR_ADDR);
  Wire.onReceive(handleReceive);
  Wire.onRequest(handleRequest);
}

void loop() {
  act_pos = analogRead(ACT_POT);
  actuate(act_pos, globalTarget);
}

void actuate(int pos, int target) {
  if (target == 0) {
    stop();
  } else if (target > 0) { // Assuming this means move upwards, and that the ACT_HIGH_THRESH indicates the max we can move FORWARD
    if (pos <= ACT_LOW_THRESH) stop();
    else goUp(target);
  } else if (target < 0) {
    if (pos >= ACT_HIGH_THRESH) stop();
    else goDown(abs(target));
  }
}

// Helper functions
void stop() {
  analogWrite(LA_FWD, 0);
  analogWrite(LA_REV, 0);
}

void goUp(int speed) {
  analogWrite(LA_FWD, speed);
  analogWrite(LA_REV, 0);
}

void goDown(int speed) {
  analogWrite(LA_FWD, 0);
  analogWrite(LA_REV, speed);
}

// Used to send encoders
void handleRequest() {
  int send = act_pos;
  Wire.write(highByte(send));
  Wire.write(lowByte(send));
}

void handleReceive(int numBytes) {
  if (Wire.available()) {
    int firstByte = Wire.read(); // Read the byte as an int
    switch (firstByte) {
      case TARGET_CHANGE:
        break;
      default:
        return;
    }
  }

  int data = 0;
  int i = 0;
  while (Wire.available() && i < 2) { // loop through all (should be two byte = int)
    int recvByte = Wire.read();
    data = (data << 8) | recvByte; // Shift the stuff over 4 and insert the new char at the end
    i++;
  }
  globalTarget = data;
}
