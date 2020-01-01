#include <Arduino.h>
#include <Wire.h>
#include "LDConsts.h"

#define PIN_PELTIER 6

volatile int peltierState = 0;

void setup() {

  //Analog(PWM) Outputs for directional driving (0-255)
  pinMode(PIN_PELTIER, OUTPUT);

  Wire.begin(SENSOR_AVR_ADDR);
  Wire.onReceive(handleReceive);
}

void loop() {
  if (peltierState == 1) {
    digitalWrite(PIN_PELTIER, HIGH);
  } else {
    digitalWrite(PIN_PELTIER, LOW);
  }
}


void handleReceive(int numBytes) {
  if (Wire.available()) {
    int firstByte = Wire.read(); // Read the byte as an int
    switch (firstByte) {
      case PELTIER_TOGGLE_BIT:
        break;
      default:
        return;
    }
  }

  int data = 0;
  if (Wire.available()) { // Only take 1 byte
    int recvByte = Wire.read();
    data = recvByte;
  }
  peltierState = data;
}
