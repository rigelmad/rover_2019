// Routine Selector Defines
#define ROUTINE_INACTIVE 0
#define ROUTINE_ADD_WATER 1
#define ROUTINE_MAIN_SLURRY 2
#define ROUTINE_CLEAN1 3
#define ROUTINE_CLEAN2 4
#define ROUTINE_CLEAN3 5
#define ROUTINE_SITE1 6
#define ROUTINE_SITE2 7
#define ROUTINE_SITE3 8
#define ROUTINE_SITE4 9
#define ROUTINE_SITE5 10
#define ROUTINE_PRED_ADD_WATER 11
#define ROUTINE_PURGE 12


/* Rosserial stuff */
#include <ros.h>
#include <life_detection/LDControl.h>
#include <life_detection/LDSensor.h>

#include <Arduino.h>
#include <Wire.h>
#include <max11604.h> // For sensor ADC
#include <Adafruit_MCP23017.h> // For I/O expanders
#include "LDConsts.h" // Consts/Addrs used throughout

// SENSOR Objects - moved these up above NodeHandle to see if this makes a difference
Adafruit_MCP23017 io_u4;
Adafruit_MCP23017 io_u3;

uint16_t io_addrs[NUM_IO_PINS] = {
  IO_SOL1,
  IO_SOL2,
  IO_SOL3,
  IO_SOL4,
  IO_SOL5,
  IO_SOL6,
  IO_SOL7,
  IO_SOL8,
  IO_SOL9,
  IO_SOL10,
  IO_BV1,
  IO_BV2,
  IO_BV3,
  IO_PUMP1,
  IO_PUMP2,
  IO_PUMP3,
  IO_SOL11,
  IO_SOL12,
  IO_SOL13,
  IO_SOL14,
  IO_SOL15,
  IO_SOL16,
  IO_SOL17,
  IO_SOL18,
  IO_SOL19,
  IO_SOL20,
  IO_PUMP4_FWD,
  IO_PUMP4_REV,
  IO_PUMP4,
  IO_WHITE_LEDS
};

Max11604 adc_u12(NUM_ADC_CHANS);

// ROS Objects

void ldCmd_cb(const life_detection::LDControl& ldcmd);
ros::Subscriber<life_detection::LDControl> ldCmd_sub("/life_detection/commands", ldCmd_cb);

life_detection::LDSensor ldSens;
ros::Publisher ldSens_pub("/life_detection/sensors", &ldSens);

// Theoretically, we shouldn't ever exceed 20 bytes in either direction on this system
// http://wiki.ros.org/rosserial/Overview/Protocol
ros::NodeHandle_<ArduinoHardware, 2, 2, 100, 200> nh;

/*** All other functional objects ***/
// For sending zero commands to slaves
bool scoopMoving = false;
bool actuatorMoving = false;
bool lightsOn = false; // Used to toggle lights
bool boxLightsOn = false;

// Used to select the active routine
uint8_t routineSelector = 0;

// Last time an execution step was taken
unsigned long lastTimeStepTaken = 0;
unsigned long lastSensorReading = 0;
unsigned long timer = millis();
uint8_t stepNum = 0;

// Custom variables for each subroutine
const uint16_t SENSOR_READ_INTERVAL_MS = 500;
const uint16_t ADD_WATER_DELAYS_MS[] = { 0, 1000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t CLEAN1_DELAYS_MS[] = { 0, 5000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t CLEAN2_DELAYS_MS[] = { 0, 7000, 7000, 7000}; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t CLEAN3_DELAYS_MS[] = { 0, 5000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t PRED_ADD_WATER_DELAYS_MS[] = { 0, 3000 };
const uint16_t PURGE_DELAYS_MS[] = { 0, 7000 };
const uint16_t SITE1_DELAYS_MS[] = { 0, 5000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t SITE2_DELAYS_MS[] = { 0, 7000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t SITE3_DELAYS_MS[] = { 0, 7000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t SITE4_DELAYS_MS[] = { 0, 7000 }; // {Between (inactive) & 0, Between 0 - 1}
const uint16_t SITE5_DELAYS_MS[] = { 0, 7000 }; // {Between (inactive) & 0, Between 0 - 1}

// Test to see if setup runs properly with all objects declared
void setup() {
  Wire.begin();
  // --- Moving these above ROS stuff to see if that has any effect also ---
  initEverything();

  nh.initNode();
  nh.subscribe(ldCmd_sub);
  nh.advertise(ldSens_pub);
  delay(1000);

}

void loop() {
  nh.spinOnce();
  timer = millis(); // Used bc idk how long I2C commands take to execute, it could be non-trivial
  switch (routineSelector) {
    case ROUTINE_INACTIVE: // 0 is inactive
      stepNum = 0;
      break;
    case ROUTINE_ADD_WATER:
      if (timer - lastTimeStepTaken > ADD_WATER_DELAYS_MS[stepNum]) {
        doTopAddWater(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_CLEAN1:
      if (timer - lastTimeStepTaken > CLEAN1_DELAYS_MS[stepNum]) {
        doClean1(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_CLEAN2:
      if (timer - lastTimeStepTaken > CLEAN2_DELAYS_MS[stepNum]) {
        doClean2(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_CLEAN3:
      if (timer - lastTimeStepTaken > CLEAN3_DELAYS_MS[stepNum]) {
        doClean3(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_PRED_ADD_WATER:
      if (timer - lastTimeStepTaken > PRED_ADD_WATER_DELAYS_MS[stepNum]) {
        doPredAddWater(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_PURGE:
      if (timer - lastTimeStepTaken > PURGE_DELAYS_MS[stepNum]) {
        doPurge(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_SITE1:
      if (timer - lastTimeStepTaken > SITE1_DELAYS_MS[stepNum]) {
        doSite1(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_SITE2:
      if (timer - lastTimeStepTaken > SITE2_DELAYS_MS[stepNum]) {
        doSite2(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_SITE3:
      if (timer - lastTimeStepTaken > SITE3_DELAYS_MS[stepNum]) {
        doSite3(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_SITE4:
      if (timer - lastTimeStepTaken > SITE4_DELAYS_MS[stepNum]) {
        doSite4(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    case ROUTINE_SITE5:
      if (timer - lastTimeStepTaken > SITE5_DELAYS_MS[stepNum]) {
        doSite5(stepNum);
        stepNum++;
        lastTimeStepTaken = timer; // Last time we took a step was now
      }
      break;
    default:
      nh.loginfo("Unimplemented function");
      routineSelector = ROUTINE_INACTIVE;
      break;
  }
}

void ldCmd_cb(const life_detection::LDControl& ldCmd) {
  if (ldCmd.hardStop) {
    stopEverything();
    routineSelector = ROUTINE_INACTIVE;
    return;
  }

  if (ldCmd.toggleLights) {
    nh.loginfo("Toggle Lights");
    doToggleLights();
    return;
  }

  if (ldCmd.moveSlurry) {
    nh.loginfo("Toggle Box lights");
    doToggleBoxLights();
    return;
  }

  if (routineSelector != ROUTINE_INACTIVE) {
    nh.logwarn("Still Running!");
    return;
  }

  if (ldCmd.scoopSpeed == 150) {
    Wire.beginTransmission(MOTOR_AVR_ADDR);
    Wire.write(1);
    Wire.endTransmission();
    scoopMoving = true;
    nh.loginfo("Motor Pos");
  } else if (ldCmd.scoopSpeed == -150) {
    Wire.beginTransmission(MOTOR_AVR_ADDR);
    Wire.write(2);
    Wire.endTransmission();
    scoopMoving = true;
    nh.loginfo("Motor Neg");
  } else if (scoopMoving) {
    Wire.beginTransmission(MOTOR_AVR_ADDR);
    Wire.write(0);
    Wire.endTransmission();
    scoopMoving = false;
    nh.loginfo("Motor stop");
  }

  if (abs(ldCmd.actuatorSpeed) > 0) {
    sendTarget(ACTUATOR_AVR_ADDR, ldCmd.actuatorSpeed);
    actuatorMoving = true;
  } else if (actuatorMoving) {
    sendTarget(ACTUATOR_AVR_ADDR, 0);
    actuatorMoving = false;
  }

  if (ldCmd.moveSlurry) {
    nh.loginfo("Move slurry");
    routineSelector = ROUTINE_MAIN_SLURRY;
  } else if (ldCmd.site1) {
    nh.loginfo("Site1");
    routineSelector = ROUTINE_SITE1;
  } else if (ldCmd.site2) {
    nh.loginfo("Site2");
    routineSelector = ROUTINE_SITE2;
  } else if (ldCmd.site3) {
    nh.loginfo("Site3");
    routineSelector = ROUTINE_SITE3;
  } else if (ldCmd.site4) {
    nh.loginfo("Site4");
    routineSelector = ROUTINE_SITE4;
  } else if (ldCmd.site5) {
    nh.loginfo("Site5");
    routineSelector = ROUTINE_SITE5;
  } else if (ldCmd.clean1) {
    nh.loginfo("Clean1");
    routineSelector = ROUTINE_CLEAN1;
  } else if (ldCmd.clean2) {
    nh.loginfo("Clean2");
    routineSelector = ROUTINE_CLEAN2;
  } else if (ldCmd.clean3) {
    nh.loginfo("Clean3");
    routineSelector = ROUTINE_CLEAN3;
  } else if (ldCmd.predAddWater) {
    nh.loginfo("Pred Add Water");
    routineSelector = ROUTINE_PRED_ADD_WATER;
  } else if (ldCmd.purge) {
    nh.loginfo("Purge");
    routineSelector = ROUTINE_PURGE;
  }
}

/**** Subroutine Functions ****/
void initEverything() {
  io_u3.begin(U3_ADDR);
  io_u4.begin(U4_ADDR);
  adc_u12.begin();

  for (int i = 0; i < NUM_IO_PINS; i++) { // Setup every channel on the I2C bus to be an output
    writeIOPin(io_addrs[i], -1); // Configure as OUTPUT
    writeIOPin(io_addrs[i], 0); // Write the pin low
  }
}

void stopEverything() {
  for (int i = 0; i < NUM_IO_PINS; i++) { // Setup every channel on the I2C bus to be an output
    writeIOPin(io_addrs[i], 0); // Write the pin low
  }
  writePump(PUMP1_ADDR, 0);
  writePump(PUMP2_ADDR, 0);
  writePump(PUMP3_ADDR, 0);
  writePump(PUMP4_ADDR, 0);
}

void sendTarget(int dest_addr, int target) {
  Wire.beginTransmission(dest_addr); // address i+1 (since you can't have a 0 I2C addr)
  Wire.write(TARGET_CHANGE);
  Wire.write(highByte(target)); // send info in MSB first
  Wire.write(lowByte(target));
  Wire.endTransmission(); // stop transmitting
  nh.loginfo("Sent target");
}

void doTopAddWater(uint8_t step) {
  switch (step) {
    case 0:
      writeIOPin(IO_SOL1, 1); // Activate S1
      writeIOPin(IO_SOL2, 1); // Activate S1
      writeIOPin(IO_SOL19, 1); // Activate S1
      break;
    case 1:
      writeIOPin(IO_SOL1, 0); // Deactivate S2
      writeIOPin(IO_SOL2, 0); // Deactivate S2
      writeIOPin(IO_SOL19, 0); // Deactivate S2
      endRoutine();
      break;
  }
}


// TODO: Make a button for me
void doPredAddWater(uint8_t step) {
  switch (step) {
    case 0:
      writePump(PUMP1_ADDR, 1);
      writeIOPin(IO_SOL1, 1); // Open solenoid 1
      // Wait for 3 sec
      break;
    case 1:
      writeIOPin(IO_SOL1, 0); //close solenoid 1
      writePump(PUMP1_ADDR, 0); // Turn off pump 1
      endRoutine();
      break;

  }
}

// TODO: Make a button for me
void doPurge(uint8_t step) {
  switch (step) {
    case 0:
      // writeIOPin(IO_PUMP4_FWD, 1); // Turn on pump 2 (connected to pump 4 fwd addr)
      writePump(PUMP2_ADDR, 1); // Turn on pump 2
      writePump(PUMP3_ADDR, 1);// TUrn on pump 3
      writeIOPin(IO_SOL2, 1); // Open solenoid 2
      writeIOPin(IO_SOL19, 1); // Open solenoid 19
      // Delay 7s
      break;
    case 1:
      // writeIOPin(IO_PUMP4_FWD, 0); // Turn off pump 2
      writePump(PUMP2_ADDR, 0); // Turn off pump 2
      writePump(PUMP3_ADDR, 0); // TUrn off pump 3
      writeIOPin(IO_SOL2, 0); // Close solenoid 2
      writeIOPin(IO_SOL19, 0); // Close solenoid 19
      endRoutine();
      break;

  }
}

void doMoveSlurry() {
  endRoutine();
}

void doSite1(uint8_t step) {
  switch (step) {
    case 0:
      writePump(PUMP2_ADDR, 1); // Turn pump 2 on (dumb pump)
      writePump(PUMP3_ADDR, 1); // Turn pump 3 on AT 200 ml/min
      writeIOPin(IO_SOL2, 1); // SOlenoid 2 open
      writeIOPin(IO_SOL4, 1); //Solenoid 4 open
      // Delay 7s
      break;
    case 1:
      writeIOPin(IO_SOL4, 0); // Close solenoid 4
      writeIOPin(IO_SOL2, 0); // Close Solenoid 2
      writePump(PUMP2_ADDR, 0); // Turn off pump 2
      writePump(PUMP3_ADDR, 0); // Turn off pump 3
      endRoutine();
      break;
  }
}

void doSite2(uint8_t step) {
  switch (step) {
    case 0:
      writePump(PUMP2_ADDR, 1); // Turn pump 2 on (dumb pump)
      writePump(PUMP3_ADDR, 1); // Turn pump 3 on AT 200 ml/min
      writeIOPin(IO_SOL2, 1); // SOlenoid 2 open
      writeIOPin(IO_SOL7, 1); //Solenoid 4 open
      // Delay 7s
      break;
    case 1:
      writePump(PUMP2_ADDR, 0); // Turn off pump 2
      writePump(PUMP3_ADDR, 0); // Turn off pump 3
      writeIOPin(IO_SOL7, 0); // Close solenoid 4
      writeIOPin(IO_SOL2, 0); // Close Solenoid 2
      endRoutine();
      break;
  }
}

void doSite3(uint8_t step) {
  switch (step) {
    case 0:
      writePump(PUMP2_ADDR, 1); // Turn pump 2 on (dumb pump)
      writePump(PUMP3_ADDR, 1); // Turn pump 3 on AT 200 ml/min
      writeIOPin(IO_SOL2, 1); // SOlenoid 2 open
      writeIOPin(IO_SOL10, 1); //Solenoid 4 open
      // Delay 7s
      break;
    case 1:
      writePump(PUMP2_ADDR, 0); // Turn off pump 2
      writePump(PUMP3_ADDR, 0); // Turn off pump 3
      writeIOPin(IO_SOL10, 0); // Close solenoid 4
      writeIOPin(IO_SOL2, 0); // Close Solenoid 2
      endRoutine();
      break;
  }
}

void doSite4(uint8_t step) {
  switch (step) {
    case 0:
      writePump(PUMP2_ADDR, 1); // Turn pump 2 on (dumb pump)
      writePump(PUMP3_ADDR, 1); // Turn pump 3 on AT 200 ml/min
      writeIOPin(IO_SOL2, 1); // SOlenoid 2 open
      writeIOPin(IO_SOL13, 1); //Solenoid 4 open
      // Delay 7s
      break;
    case 1:
      writePump(PUMP2_ADDR, 0); // Turn off pump 2
      writePump(PUMP3_ADDR, 0); // Turn off pump 3
      writeIOPin(IO_SOL13, 0); // Close solenoid 4
      writeIOPin(IO_SOL2, 0); // Close Solenoid 2
      endRoutine();
      break;
  }
}

void doSite5(uint8_t step) {
  switch (step) {
    case 0:
      writePump(PUMP2_ADDR, 1); // Turn pump 2 on (dumb pump)
      writePump(PUMP3_ADDR, 1); // Turn pump 3 on AT 200 ml/min
      writeIOPin(IO_SOL2, 1); // SOlenoid 2 open
      writeIOPin(IO_SOL16, 1); //Solenoid 4 open
      // Delay 7s
      break;
    case 1:
      writePump(PUMP2_ADDR, 0); // Turn off pump 2
      writePump(PUMP3_ADDR, 0); // Turn off pump 3
      writeIOPin(IO_SOL16, 0); // Close solenoid 4
      writeIOPin(IO_SOL2, 0); // Close Solenoid 2
      endRoutine();
      break;
  }
}

void doClean1(uint8_t step) {
  switch (step) {
    case 0:
      writeIOPin(IO_SOL1, 1); // Activate S1
      writeIOPin(IO_SOL2, 1);
      writeIOPin(IO_SOL19, 1); // S19 Active
      writePump(PUMP1_ADDR, 1); // Turn Pump 1 forward
      writePump(PUMP2_ADDR, 1); // Pump 2 forward
      writePump(PUMP3_ADDR, 1); // Pump 3 forward

      break;
    case 1:
      writeIOPin(IO_SOL1, 0); // DeActivate S1
      writeIOPin(IO_SOL2, 0);
      writeIOPin(IO_SOL19, 0); // S19 deactive
      writePump(PUMP1_ADDR, 0); // Turn Pump 1 off
      writePump(PUMP2_ADDR, 0); // Stop Pump 2
      writePump(PUMP3_ADDR, 0); // Stop Pump 3

      endRoutine();
      break;
  }
}

void doClean2(uint8_t step) {
  switch (step) {
    case 0:
      writeIOPin(IO_BV1, 1); // Ball valve 1 Active
      break;
    case 1:
      writePump(PUMP1_ADDR, 1); // Turn Pump 1 forward
      writeIOPin(IO_SOL18, 1); // Activate S18
      // writeIOPin(IO_SOL19, 1); // Activate S18
      // writeIOPin(IO_SOL3, 1); // Activate S18
      writePump(PUMP3_ADDR, -1); // Pump 3 backward
      break;
    case 2:
      writePump(PUMP1_ADDR, 0); // Turn Pump 1 forward
      writeIOPin(IO_SOL18, 0); // Activate S18
      // writeIOPin(IO_SOL19, 0); // Activate S18
      // writeIOPin(IO_SOL3, 0); // Activate S18
      writePump(PUMP3_ADDR, 0); // Pump 3 backward
      writeIOPin(IO_BV1, 0); // Ball valve 1 Active
      break;
    case 3:
      endRoutine(); // We're done
      break;
  }
}

void doClean3(uint8_t step) {
  switch (step) {
    case 0:
      writeIOPin(IO_SOL2, 1); // Activate S1
      writeIOPin(IO_SOL19, 1); // S19 Active
      break;
    case 1:
      writeIOPin(IO_SOL2, 0); // Activate S1
      writeIOPin(IO_SOL19, 0); // S19 Active
      endRoutine();
      break;
  }
}

void doToggleLights() {
  if (lightsOn) {
    writeIOPin(IO_SOL15, 0); // If on, turn lights off
  } else {
    writeIOPin(IO_SOL15, 1);
  }
  lightsOn = !lightsOn;
}

void doToggleBoxLights() {
  if (boxLightsOn) {
    writeIOPin(IO_WHITE_LEDS, 0); // If on, turn lights off
  } else {
    writeIOPin(IO_WHITE_LEDS, 1);
  }
  boxLightsOn = !boxLightsOn;
}

/*** HELPER FUNCTIONS ***/

void endRoutine() {
  uint8_t lastRoutine = routineSelector;
  ldSens.addr = -1;
  ldSens.value = lastRoutine;
  ldSens_pub.publish(&ldSens);
  nh.loginfo("Done!");
  routineSelector = ROUTINE_INACTIVE;
}

void pubADCVal(uint8_t channel) {
  ldSens.addr = channel;
  ldSens.value = getADCVal(channel);

  ldSens_pub.publish(&ldSens);
}

int getADCVal(uint8_t channel) {
  return adc_u12.readSingleChannel(channel);
}

// Write any one of the defined I/O expander addresed pins
/*
  @param addr - The 2 byte address giving chip|pin
  @param value - 0 for LOW, 1 for HIGH, -1 for pinMode OUTPUT (modified for setup)
*/
void writeIOPin(uint16_t addr, int8_t value) {
  uint8_t chipSelect = highByte(addr);
  uint8_t pinNum = lowByte(addr);

  if (value == -1) { // -1 indicates setting up pinMode OUTPUT
    if (chipSelect == 0x04) io_u4.pinMode(pinNum, OUTPUT);
    else if (chipSelect == 0x03) io_u3.pinMode(pinNum, OUTPUT);
  } else {
    if (chipSelect == 0x04) io_u4.digitalWrite(pinNum, value);
    else if (chipSelect == 0x03) io_u3.digitalWrite(pinNum, value);
  }
}

void writePeltier(uint8_t state) {
  Wire.beginTransmission(SENSOR_AVR_ADDR);
  Wire.write(PELTIER_TOGGLE_BIT);
  Wire.write( (state == HIGH) ? 1 : 0 );
  Wire.endTransmission();
}

/*
   @param pumpAddr The address of the EZO pump
   @param value 0 for stop, 1 for forward, -1 for rev
*/
void writePump(uint8_t pumpAddr, int8_t value) {
  switch(pumpAddr) {
    case PUMP1_ADDR:
      if (value == -1) return;
      writeIOPin(IO_PUMP1, value);
      break;
    case PUMP2_ADDR:
      if (value == -1) return;
      writeIOPin(IO_PUMP2, value);
      break;
    case PUMP3_ADDR:
      if (value == 1) {
        writeIOPin(IO_PUMP4_FWD, 1);
        writeIOPin(IO_PUMP4_REV, 0);
      } else if (value == -1) {
        writeIOPin(IO_PUMP4_FWD, 0);
        writeIOPin(IO_PUMP4_REV, 1);
      } else {
        writeIOPin(IO_PUMP4_FWD, 0);
        writeIOPin(IO_PUMP4_REV, 0);
      }
      break;
    case PUMP4_ADDR:
      if (value == -1) return;
      writeIOPin(IO_PUMP4, value);
      break;
  }
}

void setEXOAddr(uint8_t newAddr) {
  String cmd = String("I2C," + String(newAddr, DEC));
  Wire.beginTransmission(EXO_DEFAULT_ADDR);
  Wire.write(cmd.c_str());
  Wire.endTransmission();
}

// Will call the FIND command on each of the EXOs in order 1,2,3
void findAllEXOs() {
  uint8_t all_exos[] = { PUMP1_ADDR, PUMP3_ADDR, PUMP4_ADDR };
  for (int i = 0; i < 3; i++) {
    findEXO(all_exos[i]);
  }
}

void findEXO(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write("Find"); // Send find cmd
  Wire.endTransmission();

  delay(3000);

  Wire.beginTransmission(addr);
  Wire.write("X"); // Send stop
  Wire.endTransmission();

  delay(300); // Processing delay
}
