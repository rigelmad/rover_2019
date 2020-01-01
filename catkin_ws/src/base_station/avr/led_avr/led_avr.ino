#include <Arduino.h>
#include <Wire.h>
#include "BSConsts.h"

#define CTRL_RED_1 5
#define CTRL_GREEN_1 3
#define CTRL_BLUE_1 6
#define CTRL_RED_2 10
#define CTRL_GREEN_2 9
#define CTRL_BLUE_2 11

const int led_pins[] = {
  CTRL_RED_1,
  CTRL_GREEN_1,
  CTRL_BLUE_1,
  CTRL_RED_2,
  CTRL_GREEN_2,
  CTRL_BLUE_2
};

volatile uint8_t led_states[] =  { 0, 0, 0, 0, 0, 0 };

void setup() {

  //Analog(PWM) Outputs for directional driving (0-255)
  for (int i = 0; i < 6; i++) {
    pinMode(led_pins[i], OUTPUT);
    analogWrite(led_pins[i], 0);
  }

  Wire.begin(LED_AVR_ADDR);
  Wire.onReceive(handleReceive);
}

void loop() {
  for (int i = 0; i < 6; i++) {
    analogWrite(led_pins[i], led_states[i]);
  }
}


void handleReceive(int numBytes) {
  uint8_t channel, red, green, blue;
  while (Wire.available()) {
    channel = Wire.read();
    red = Wire.read();
    green = Wire.read();
    blue = Wire.read();
  }

  int offset = (channel == 1) ? 0 : 3;
  led_states[offset + RED] = red;
  led_states[offset + GREEN] = green;
  led_states[offset + BLUE] = blue;
}
