#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_ADDR 6
#define MASTER_ADDR 0x69

#define encoder0PinA  2
#define encoder0PinB  3
#define ledPin        9
#define enableLPin    7
#define enableRPin    8
#define leftDrive     6
#define rightDrive    5

#define ENCODER_REQ   1
#define TARGET_CHANGE 2

unsigned long timer = 0;
unsigned long prevTime = 0;
volatile signed int encoder0Pos = 0;
volatile signed long totalEncPos = 0;

int prevPos = 0;
int numTicks = 0;

float MOTORRATIO;
const float SPDCONV = 600; //(sampling time)^-1 * 60
const float TF = 3 / (1E6);

const int propGain = 4.0; //wrist joint = 4.0
const int derivGain = 0.2;

float rpm = 0;
int target = 0;
float prevError = 0.0;
float prevOut = 0.0;

void setup();
void loop();
void driveSpeed(int target);
void handleReceive();

bool respondToReq = false;


void setup() {

  if (MOTOR_ADDR == 1 || MOTOR_ADDR == 2 || MOTOR_ADDR == 3) {
    MOTORRATIO = 2996.1 * 24.0; //big arm motors (motors 1, 2, 3)
  } else if (MOTOR_ADDR == 4 || MOTOR_ADDR == 6) {
    MOTORRATIO = 4740.4 / 24; //small motor joint (motors 4 and (sort of) 6)
  } else if (MOTOR_ADDR == 5) {
    MOTORRATIO = 4740.4; //wrist (motor 5)
  }
  //Encoder input pins (Need to be hardware interrupt pins)
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  //pins to enable directions -- always left HIGH to enable 2 dir driving
  pinMode(enableRPin, OUTPUT);
  pinMode(enableLPin, OUTPUT);

  //Analog(PWM) Outputs for directional driving (0-255)
  pinMode(leftDrive, OUTPUT);
  pinMode(rightDrive, OUTPUT);
  //Interrupt Indicator (Not really necessary)
  pinMode(ledPin, OUTPUT);

  timer = millis();
  // setting direction pins high
  digitalWrite(enableLPin, HIGH);
  digitalWrite(enableRPin, HIGH);

  Wire.begin(MOTOR_ADDR);
  Wire.onReceive(handleReceive);
}

void loop() {

  //for driving to speed
  driveSpeed(target);


}

void sendEncoder() {
  signed long temp = totalEncPos;
  Wire.beginTransmission(MASTER_ADDR); // Send back to master
  Wire.write(MOTOR_ADDR);
  for (char i = 0; i < 4; i++) { // 4 times to send 4 bytes
    byte s = lowByte(temp);
    Wire.write(lowByte(temp)); // Write the LSByte
    temp = temp >> 8; // Discard the 8 LSBits after sending
  }
  Wire.endTransmission();
}

// On receive, change the target angle - !!!degrees
void handleReceive(int numBytes) {
  if (Wire.available()) {
    int firstByte = Wire.read(); // Read the byte as an int
    switch (firstByte) {
      case TARGET_CHANGE:
        break;
      case ENCODER_REQ:
        return;
      default:
        break;
    }
  }

  int data = 0;
  int i = 0;
  while (Wire.available() && i < 2) { // loop through all (should be two byte = int)
    byte recvByte = Wire.read();
    data = (data << 8) | recvByte; // Shift the stuff over 4 and insert the new char at the end
    i++;
  }
  Serial.print("New target: ");
  Serial.print(data);
  Serial.println();
  target = data;
}

void driveSpeed(int target)
{
  if (target > 0)
  {
    //Corrects output signal to controller limits
    if (target <= 10)
      target = 10;
    else if (target >= 255)
      target = 254;
    analogWrite(rightDrive, target);
  }

  else if (target < 0)
  {
    //Corrects output signal to controller limits
    if (target >= -10)
      target = -10;
    else if (target <= -255)
      target = -254;
    analogWrite(leftDrive, abs(target));
  }
  else
  {
    analogWrite(leftDrive, 0);
    analogWrite(rightDrive, 0);
  }
}
