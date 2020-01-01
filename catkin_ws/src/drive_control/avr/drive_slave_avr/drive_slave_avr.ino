#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_ADDR 3
#define MASTER_ADDR 69

#define encoder0PinA  2
#define encoder0PinB  3
#define ledPin        9
#define enableLPin    7
#define enableRPin    8
#define leftDrive     5
#define rightDrive    6

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

int actualTarget = 0;
int intendedTarget = 0;
unsigned long lastAccelUpdateTime = 0;

/*
   Currently, it will take the rover 400 ms to go from 0-100% speed and back down from 100% to full stop
*/
const int RAMP_TIME_MS = 100; // Time after which the ramp can increase/decrease.
// Max amount that the target can step in one ramp interval...
// We can go from 0% - 100% speed in (RAMP_TIME_MS * 4) ms
const int RAMP_STEP_PWM = 255 / 4;

float rpm = 0;
float prevError = 0.0;
float prevOut = 0.0;

bool respondToReq = false;
bool shouldSendEncoder = false;

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
  //pinMode(ledPin, OUTPUT);


 // encoder pin on interrupt 0 (pin 2)
 attachInterrupt(0, doEncoderA, CHANGE);

 // encoder pin on interrupt 1 (pin 3)
 attachInterrupt(1, doEncoderB, CHANGE);

  timer = millis();
  // setting direction pins high
  digitalWrite(enableLPin, HIGH);
  digitalWrite(enableRPin, HIGH);

  // Serial.begin(57600);

  Wire.begin(MOTOR_ADDR);
  Wire.onReceive(handleReceive);
}

void loop() {

  //Side effect - modifies actualTarget (global)
  controlAcceleration();

  //for driving to speed
  driveSpeed(actualTarget);

  //  if (shouldSendEncoder) {
  //    sendEncoder();
  //    shouldSendEncoder = false;
  //  }

}

// Updates actualTarget based on intendedTarget
void controlAcceleration() {
  if (intendedTarget == actualTarget) return; // Immediate return state when intendedTarget hasnt changed
  int diff = intendedTarget - actualTarget;
  int absDiff = abs(diff);
  if (millis() - lastAccelUpdateTime > RAMP_TIME_MS) {
    if (absDiff < RAMP_STEP_PWM) {
      actualTarget = intendedTarget;
    } else if (intendedTarget < actualTarget) {
      actualTarget -= RAMP_STEP_PWM;
      if (actualTarget < intendedTarget) actualTarget = intendedTarget; // Correct overcompensation
    } else {
      actualTarget += RAMP_STEP_PWM;
      if (actualTarget > intendedTarget) actualTarget = intendedTarget; // Correct overcompensation
    }
    lastAccelUpdateTime = millis();
  }
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
        shouldSendEncoder = true;
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
  intendedTarget = data;

}

void driveAngle(int target) {
  //PD controller?

  //"sample time"
  float deltaT = timer - prevTime;

  //error between current and target based on encoder position
  float error = float(target) - (totalEncPos / MOTORRATIO * 360);

  //implements controller transfer function: o = Kd*de/dt + Kp*e
  float calc_out = error * propGain + (error - prevError) * derivGain / (deltaT / 1000);

  //converts output to integer (rounding)
  int out = calc_out;

  //Checks direction.
  if (out > 0)
  {
    //Corrects output signal to controller limits
    if (out <= 10)
      out = 10;
    else if (out >= 150)
      out = 149;
    analogWrite(rightDrive, out);

    //need to test without delay
    delay(10);
  }

  else if (out < 0)
  {
    //Corrects output signal to controller limits
    if (out >= -10)
      out = -10;
    else if (out <= -255)
      out = -254;
    analogWrite(leftDrive, abs(out));

    //need to test without delay
    delay(10);
  }

  else
  {
    analogWrite(leftDrive, 0);
    analogWrite(rightDrive, 0);
  }

  //Stores prev error for discrete derivative
  prevError = error;
  //prevOut = filt_out;
  prevTime = timer;
}

void doEncoderA() {
  // look for a low-to-high on channel A
  //digitalWrite(9, HIGH);
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
  //digitalWrite(9, LOW);
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

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}
