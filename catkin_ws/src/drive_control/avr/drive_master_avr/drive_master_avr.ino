#include <ros.h>
#include <std_msgs/Int16.h>
#include <drive_control/DriveCommand.h>
#include <rover_diagnostics/current_sense.h>
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

#define ENCODER_REQ 1
#define TARGET_CHANGE 2

/*
 *
 * Addressing for our Current Sensors
 * Motor 1: hex 40 = bin 0100 0000
 * Motor 2: hex 41 = bin 0100 0001
 * Motor 3: hex 44 = bin 0100 0100
 * Motor 4: hex 45 = bin 0100 0101
 * Motor 5: hex 48 = bin 0100 1000
 * Motor 6: hex 4C = bin 0100 1100
 *
 * CORRECTED:
 * Motor 1: hex 45 = bin 0100 0100
 * Motor 2: hex 48 = bin 0100 1000
 * Motor 3: hex 4C = bin 0100 1100
 * Motor 4: hex 40 = bin 0100 0000
 * Motor 5: hex 41 = bin 0100 0001
 * Motor 6: hex 44 = bin 0100 0100
 *
 */
//ADD CURRENT SENSING

float movingAverageSum[] = { 0, 0, 0, 0, 0, 0 };
float movingAverage[] = { 0, 0, 0, 0, 0, 0 };
const byte averageCount = 20;

Adafruit_INA219 current_sensors[] = {
  Adafruit_INA219(0x45),
  Adafruit_INA219(0x48),
  Adafruit_INA219(0x4C),
  Adafruit_INA219(0x40),
  Adafruit_INA219(0x41),
  Adafruit_INA219(0x44)
};

signed long encoderPos[] = { 0, 0, 0, 0, 0, 0 }; // Global array of encoder positions
unsigned long lastSend = 0;
int interval = 2000;
float target = 0;

unsigned long lastCurrentSense = 0;
const int CURRENT_SENSE_POLL_INTERVAL = 50; // Poll current sensors once a second

const int TIMEOUT = 500; // Timeout at which to stop looking for polls

bool expectedAcks[] = { 0, 0, 0, 0, 0, 0 }; // True if we are expecting an ACK from the channel at each index+1

ros::NodeHandle nh;

void servo_cb(const drive_control::DriveCommand &driveCommand);
ros::Subscriber<drive_control::DriveCommand> servo("/drive/commands", servo_cb);

rover_diagnostics::current_sense cs;
ros::Publisher currentPub("/drive/current", &cs);

void setup() {

  //ADD CURRENT SENSING
  // Serial.begin(9600);

  for (int i = 0; i < 6; i++) {
    current_sensors[i].begin();
    current_sensors[i].setCalibration_32V_2A();
    // Pre-load MMA
    for (int x = 0; x < averageCount; x++)
      movingAverageSum[i] = movingAverageSum[i] + current_sensors[i].getCurrent_mA();

    // Calculate inital average
    movingAverage[i] = movingAverageSum[i] / averageCount;
  }
  //END CURRENT SENSING STUFF


  Wire.begin(); // Special I2C addr for master
  // Wire.onReceive(handleRec);

  // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(servo);
  nh.advertise(currentPub);

  delay(1000);
}

void servo_cb(const drive_control::DriveCommand &driveCommand) {
  float drive_left = driveCommand.left;
  float drive_right = driveCommand.right;

  int leftTarget = (int) (drive_left * 255.0); // The target for left sides
  int rightTarget = (int) (drive_right * 255.0); // The target for left sides

  int sendMe[] = { //Assumes motors addr 1-3 are left side, motors addr 4-6 are right
    leftTarget,
    leftTarget,
    leftTarget,
    rightTarget,
    rightTarget,
    rightTarget
  };

  sendToAllSlaves(sendMe);
}

void loop() {

  //END CURRENT SENSING STUFF
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
    // delay(1);
  }
  nh.loginfo("Sent to slaves");
}

/*

void pollEncoders() {
  if ((millis() - lastSend) > interval) {
    lastSend = millis();
    digitalWrite(13, HIGH);
    pollAllSlavesForEncoderData();
    while (isBusy()) {
      if (millis() - lastSend > TIMEOUT) {
        for (int i = 0; i < 6; i++) {
          expectedAcks[i] = false;
        }
      }
    }
    digitalWrite(13, LOW);
  }
}

bool isBusy() { // Are we expecting encoder data from any channel?
  for (int i = 0; i < 6; i++) {
    if (expectedAcks[i]) {
      return true;
    }
  }
  return false;
}

void handleRec(int numBytes) { // Currently only set up to handle encoder readings
  if (numBytes != 5) {
    return;
  }
  byte data[4] = {0, 0, 0, 0};
  int addr;
  if (Wire.available()) {
    byte first = Wire.read();
    addr = (int) first;
  }

  int bytes = 0;
  while (Wire.available()) {
    byte recv = Wire.read(); // receive in LSByte order
    data[3 - bytes] = recv; // Store the LSB at the end of the array
    bytes++;
  }

  updateEncoderData(data, addr);
  expectedAcks[addr - 1] = false; // No longer expecting an encoder ack from this addr
}

void updateEncoderData(byte data[], int encoderNum) {

  signed long encPos = 0; // Intialize the data
  for (int j = 0; j < 4; j++) { // Traverse the array of 4 bytes
    encPos = (encPos << 8) | data[j]; // Shift the stuff over 4 and insert the new char at the end
  }

  encoderPos[encoderNum - 1] = encPos; // Store this into the array, again need to handle the fact that its a float, not a char
}

void pollAllSlavesForEncoderData() {
  for (int i = 1; i <= 6; i++) { // Addrs [1-6]
    Wire.beginTransmission(i);
    Wire.write(ENCODER_REQ);
    Wire.endTransmission();
    expectedAcks[i-1] = true;
  }
}
*/
