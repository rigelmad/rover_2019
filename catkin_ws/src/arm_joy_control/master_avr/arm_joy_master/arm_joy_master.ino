#include <ros.h>
#include <std_msgs/Int16.h>
#include <arm_joy_control/ArmJoyMsg.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <rover_diagnostics/current_sense.h>

#define MASTER_ADDR 0x69

#define ENCODER_REQ 1
#define TARGET_CHANGE 2

const int SPEED[] =  { // Edited to accomodate speed control on Motors 1, 4, 5
  2,
  75, //Special speed for the shoulder
  150,
  2,
  2,
  150
};

float target = 0;

/* --- Current Sense --- */
unsigned long lastCurrentSense = 0;
const int CURRENT_SENSE_POLL_INTERVAL = 50; // Poll current sensors once a second
float movingAverageSum[] = { 0, 0, 0, 0, 0, 0 };
float movingAverage[] = { 0, 0, 0, 0, 0, 0 };
const byte averageCount = 20;

/* --- ROS --- */
ros::NodeHandle nh;

// Array of current sensors
Adafruit_INA219 current_sensors[] = {
  Adafruit_INA219(0x45),
  Adafruit_INA219(0x48),
  Adafruit_INA219(0x4C),
  Adafruit_INA219(0x40),
  Adafruit_INA219(0x41),
  Adafruit_INA219(0x44)
};

void servo_cb(const arm_joy_control::ArmJoyMsg &armCommand);
ros::Subscriber<arm_joy_control::ArmJoyMsg> servo("/arm/commands", servo_cb);

rover_diagnostics::current_sense cs;
ros::Publisher currentPub("/arm/current", &cs);

void setup() {

  Wire.begin(); // Special I2C addr for master

  // nh.getHardware()->setBaud(57600);

  for (int i = 0; i < 6; i++) {
    current_sensors[i].begin();
    current_sensors[i].setCalibration_32V_2A();

    // Pre-load MMA
    for (int x = 0; x < averageCount; x++)
      movingAverageSum[i] = movingAverageSum[i] + current_sensors[i].getCurrent_mA();

    // Calculate inital average
    movingAverage[i] = movingAverageSum[i] / averageCount;
  }

  nh.initNode();
  nh.subscribe(servo);
  nh.advertise(currentPub);

  delay(1000); // Delay for ROS comms
}

void servo_cb(const arm_joy_control::ArmJoyMsg &armCommand) {

  int sendMe[] = { //Assumes motors addr 1-3 are left side, motors addr 4-6 are right
    armCommand.shoulder * SPEED[0],
    armCommand.upperArm * SPEED[1],
    armCommand.midArm * SPEED[2],
    armCommand.lowerArm * SPEED[3],
    armCommand.wrist * SPEED[4],
    armCommand.finger * SPEED[5]
  };
  sendToAllSlaves(sendMe);
}

void loop() {
  nh.spinOnce();

  if (millis() - lastCurrentSense > CURRENT_SENSE_POLL_INTERVAL) {
    pollCurrentSense();
    lastCurrentSense = millis();
  }
}

void pollCurrentSense() {
  int sendMe[6];
  for (int i = 0; i < 6; i++) {
    float currentValue = current_sensors[i].getCurrent_mA();
    // Remove previous movingAverage from the sum
    movingAverageSum[i] = movingAverageSum[i] - movingAverage[i];
    // Replace it with the current sample
    movingAverageSum[i] = movingAverageSum[i] + currentValue;
    // Recalculate movingAverage
    movingAverage[i] = movingAverageSum[i] / averageCount;

    sendMe[i] = (int) movingAverage[i];
    // sendMe[i] = (int) currentValue;
  }

  rover_diagnostics::current_sense cs;
  cs.motor1 = sendMe[0];
  cs.motor2 = sendMe[1];
  cs.motor3 = sendMe[2];
  cs.motor4 = sendMe[3];
  cs.motor5 = sendMe[4];
  cs.motor6 = sendMe[5];
  currentPub.publish(&cs);
  // nh.loginfo("Published");
}

/*
  Send each slave the desired position to which it should move.
  @param positions The 6 element array of positions at which each slave should turn
*/
void sendToAllSlaves(int positions[]) { // Addrs [1-6]
  for (int i = 1; i <= 6; i++) {
    int sendAngle = positions[i - 1];
    Wire.beginTransmission(i); // address i+1 (since you can't have a 0 I2C addr)
    Wire.write(TARGET_CHANGE);
    Wire.write(highByte(sendAngle)); // send info in MSB first
    Wire.write(lowByte(sendAngle));
    Wire.endTransmission(); // stop transmitting
  }
  nh.loginfo("Sent to slaves");
}
