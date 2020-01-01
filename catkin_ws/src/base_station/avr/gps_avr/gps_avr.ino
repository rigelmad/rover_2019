#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include "BSConsts.h"

//Base Station Slave Code
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 10
// Connect the GPS RX (receive) pin to Digital 9
#define RX_PIN 9
#define TX_PIN 10

//GPS global variables
SoftwareSerial gpsSerial(TX_PIN, RX_PIN);
Adafruit_GPS gps(&gpsSerial);

unsigned long lastGpsRead = 0;
const int GPS_POLLING_INTERVAL_MS = 2000;   //2 second polling interval

double latitude = 0;
double longitude = 0;
double altitude = 0;
int num_satellites = 0;
double last_lat = 0;
double last_long = 0;
double last_alt = 0;
int last_sats = 0;
bool gpsStatusFailed = false;

void setup() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gps.begin(9600);
  initializeGPS();
  Wire.begin(GPS_AVR_ADDR);  //start i2c
  Wire.onRequest(handleRequest);

  lastGpsRead = millis();
}

void loop() {
  char c = gps.read();
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) {
      gpsStatusFailed = true;  // we can fail to parse a sentence in which case we should just wait for another
    }
    else {
      gpsStatusFailed = false;
    }
  }
  //read GPS data only if data is valid
  if (!gpsStatusFailed) {

    // if millis() or lastGpsRead wraps around,just reset it
    if (lastGpsRead > millis())  lastGpsRead = millis();

    // get GPS Data on interval
    if (millis() - lastGpsRead > GPS_POLLING_INTERVAL_MS) {
      lastGpsRead = millis(); // reset the lastGpsRead
      pollGPSData();
    }
  }
}
//send data back to master
void handleRequest() {
  byte *dataPtr;
  double data;
  int numBytes = 0;

  for(int i = 0; i < 4; i++) {                   //send 10 values total (40 bytes)
    switch (i) {    //decide which value to send
      case (FLAG_LAT):
        data = last_lat;
        break;
      case (FLAG_LONG):
        data = last_long;
        break;
      case (FLAG_ALT):
        data = last_alt;
        break;
      case (FLAG_SATS):
        data = (double) last_sats;
        break;
      default:
        data = 0;
        break;
    }
    dataPtr = (byte *) &data;     //convert float to array of bytes
    Wire.write(dataPtr,4);
  }
}
// function to initalize the GPS
void initializeGPS(){
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  gps.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  gpsSerial.println(PMTK_Q_RELEASE);

}
//function to update global gps values
//returns bool if GPS has a fix
bool pollGPSData(){
  String n_s;
  String e_w;
  bool hasFix = false;

  //update last known values
  last_lat = latitude;
  last_long = longitude;
  last_alt = altitude;
  last_sats = num_satellites;
  if (gps.fix) {
    hasFix = true;
    latitude = gps.latitudeDegrees;
    longitude = gps.longitudeDegrees;
    altitude = gps.altitude;
    num_satellites = (int)gps.satellites;
    n_s = gps.lat;
    e_w = gps.lon;
    //check direction of degree
    if (n_s == "S") {
      latitude = -latitude;
    }
    if (e_w == "W") {
      longitude = -longitude;
    }
  }
  return hasFix;
}
