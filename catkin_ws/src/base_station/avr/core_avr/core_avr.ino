/* Rosserial stuff */
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <base_station/GpsMsg.h>
#include <base_station/LedMsg.h>
#include <base_station/MotorDiagnosticMsg.h>

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "BSConsts.h"

#include <Adafruit_MCP9808.h> // Temp sensor
#include <Adafruit_INA219.h> // Current sense

// ROS Objects

base_station::GpsMsg bsPos;
ros::Publisher bsGps_pub("/base_station/gps", &bsPos);

base_station::MotorDiagnosticMsg motorDiag;
ros::Publisher motor_pub("/base_station/motor_diagnostic", &motorDiag);

std_msgs::Float32 temp;
ros::Publisher temp_pub("/base_station/temp", &temp);

void antenna_cb(const std_msgs::Int16& angle);
ros::Subscriber<std_msgs::Int16> antenna_sub("/base_station/antenna_angle", antenna_cb);

void led_cb(const base_station::LedMsg& led_msg);
ros::Subscriber<base_station::LedMsg> bsLed_sub("/base_station/led", led_cb);

// http://wiki.ros.org/rosserial/Overview/Protocol
ros::NodeHandle_<ArduinoHardware, 3, 3, 100, 280> nh;

/*** Globals ****/

// Timing globals
unsigned long timer = 0;
unsigned long lastStatePublish = 0;
unsigned long lastGPSRead = 0;
const int STATE_PUB_INTERVAL_MS = 1000;
const int GPS_READ_INTERVAL_MS = 5000;

unsigned long lastMotorPoll = 0;
const int ENCODER_READ_INTERVAL_MS = 100;
const float STOP_POLLING_THRESHOLD = 5.0;
bool shouldPollMotor = false;

// State globals
volatile float base_station_latitude = 0.00; // Gets updated in polling of GPS
volatile float base_station_longitude = 0.00; // Gets updated in polling of GPS
volatile float base_station_altitude = 0.00;
volatile int base_station_satellites = 0;

// ROS callback-based globals
int antenna_angle = 0; // Comes from coordinate-based calculation
float rover_distance = 0.00; // Comes from coordinate-based calculation
byte led_chan1[3] = { 0, 0, 0 };
byte led_chan2[3] = { 0, 0, 0 };

// Current sense avg.
float movingAverageSum = 0;
float movingAverage = 0;
const byte averageCount = 10;
Adafruit_INA219 current_sensor = Adafruit_INA219(CURRENT_SENSE_ADDR);

// Temp sense
Adafruit_MCP9808 temp_sensor = Adafruit_MCP9808();
const int TEMP_POLL_INTERVAL_MS = 2000;
unsigned long lastTempPoll = 0;

/****************/

void setup() {
  Wire.begin();
  current_sensor.begin();
  current_sensor.setCalibration_32V_2A();

  temp_sensor.begin(TEMP_SENSE_ADDR);
  temp_sensor.setResolution(2);

  nh.initNode();
  nh.subscribe(antenna_sub);
  nh.subscribe(bsLed_sub);
  nh.advertise(bsGps_pub);
  nh.advertise(motor_pub);
  nh.advertise(temp_pub);
  delay(1000);
}

void loop() {
  nh.spinOnce();
  timer = millis();

  if (timer - lastGPSRead > GPS_READ_INTERVAL_MS) {
    pollGPS();
    lastGPSRead = timer;
  }

  if (timer - lastMotorPoll > ENCODER_READ_INTERVAL_MS) {
    if (shouldPollMotor) {
      pollMotorData();
      lastMotorPoll = timer;
    }
  }

  if (timer - lastTempPoll > TEMP_POLL_INTERVAL_MS) {
    pollTempSense();
    lastTempPoll = timer;
  }
}

void antenna_cb(const std_msgs::Int16& angle) {
  sendTarget(MOTOR_AVR_ADDR, angle.data);
  shouldPollMotor = true;
}

void led_cb(const base_station::LedMsg& led_msg) {
  switch (led_msg.channel) {
    case (1):
      led_chan1[RED] = led_msg.r;
      led_chan1[GREEN] = led_msg.g;
      led_chan1[BLUE] = led_msg.b;
      break;
    case (2):
      led_chan2[RED] = led_msg.r;
      led_chan2[GREEN] = led_msg.g;
      led_chan2[BLUE] = led_msg.b;
      break;
    default:
      nh.logwarn("Invalid LED channel");
      return;
  }

  Wire.beginTransmission(LED_AVR_ADDR);
  Wire.write(led_msg.channel);
  Wire.write(led_msg.r);
  Wire.write(led_msg.g);
  Wire.write(led_msg.b);
  Wire.endTransmission();
  nh.loginfo("Sent LED");
}

void sendTarget(int dest_addr, int target) {
  Wire.beginTransmission(dest_addr); // address i+1 (since you can't have a 0 I2C addr)
  Wire.write(TARGET_CHANGE);
  Wire.write(highByte(target)); // send info in MSB first
  Wire.write(lowByte(target));
  Wire.endTransmission(); // stop transmitting
  nh.loginfo("Sent target");
}

void pollTempSense() {
  temp_sensor.wake();   // wake up, ready to read!
  float tempC = temp_sensor.readTempC();

  temp.data = tempC;
  temp_pub.publish(&temp);
}

void pollMotorData() {
  motorDiag.currentMA = pollCurrentSense();
  motorDiag.motorOut = pollEncoder();
  motor_pub.publish(&motorDiag);
}

float pollEncoder() {
  Wire.requestFrom(MOTOR_AVR_ADDR, 4);

  char incomingData[4] = { 0, 0, 0, 0 };
  int i = 0;
  while (Wire.available() && i < 4) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    incomingData[i] = c;
    i++;
  }

  union u_val {
     byte b[4];
     float val;
  } u;

  u.b[0] = incomingData[0];
  u.b[1] = incomingData[1];
  u.b[2] = incomingData[2];
  u.b[3] = incomingData[3];

  float nextVal = u.val;
  if (nextVal < STOP_POLLING_THRESHOLD) {
    shouldPollMotor = false;
  }

  return nextVal;
}

int16_t pollCurrentSense() {
  float currentValue = current_sensor.getCurrent_mA();
  // Remove previous movingAverage from the sum
  movingAverageSum = movingAverageSum - movingAverage;
  // Replace it with the current sample
  movingAverageSum = movingAverageSum + currentValue;
  // Recalculate movingAverage
  movingAverage = movingAverageSum / averageCount;

  int16_t sendMe = (int16_t) movingAverage;

  return sendMe;
}

void pollGPS() { // TODO: Write me
  const int MSG_LENGTH = 16;
  Wire.requestFrom(GPS_AVR_ADDR, MSG_LENGTH);
  int i = 0;
  char incomingData[MSG_LENGTH] = { 0 };
  while (Wire.available() && i < MSG_LENGTH) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    incomingData[i] = c;
    i++;
  }

  interpretGPSData(incomingData, MSG_LENGTH);

  bsPos.latitude = base_station_latitude;
  bsPos.longitude = base_station_longitude;
  bsPos.altitude = base_station_altitude;
  bsPos.satellites = base_station_satellites;

  bsGps_pub.publish(&bsPos);
}

void interpretGPSData(char* incomingData, int msgLength) {
  int counter = 0;
  //interpret 4 bytes as float
  union u_val {
     byte b[4];
     float val;
  } u;

  for (int i = 0; i < msgLength; i += 4) {
    u.b[0] = incomingData[0 + i];
    u.b[1] = incomingData[1 + i];
    u.b[2] = incomingData[2 + i];
    u.b[3] = incomingData[3 + i];
    float nextVal = u.val;

    switch (counter) {
      case (FLAG_LAT):
        base_station_latitude = nextVal;
        break;
      case (FLAG_LONG):
        base_station_longitude = nextVal;
        break;
      case (FLAG_ALT):
        base_station_altitude = nextVal;
        break;
      case (FLAG_SATS):
        base_station_satellites = (int) nextVal;
        break;
      default:
        break;
    }
    counter++; // Onto the next macro value
  }
}
