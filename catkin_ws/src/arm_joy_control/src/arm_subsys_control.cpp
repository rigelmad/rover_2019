#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controls/ControlCmd.h"
#include "controls/logitech_controller.cpp"
#include <sensor_msgs/Joy.h>
#include <arm_joy_control/ArmJoyMsg.h>
#include <std_msgs/Empty.h>

#include <sstream>
#include <cmath>
#include <cstring>

#define SHOULDER 0
#define UPPER_ARM 1
#define MID_ARM 2
#define LOWER_ARM 3
#define WRIST 4
#define FINGER 5

const float CONTROLLER_ZERO_CUTOFF = 0.05;

char last_cmd[] = { 0, 0, 0, 0, 0, 0 };

bool posTrigMoved = false;
bool negTrigMoved = false;
bool middle_has_been_cleared = true;

//for now saying drive commands are strings, this is going to change later
ros::Subscriber joy_sub;
ros::Publisher arm_commands_pub;
ros::Publisher arm_stop_pub;

LogController l;

std::string translate(int num) {
  switch (num) {
    case 0:
      return "Shoulder";
    case 1:
      return "UpperArm";
    case 2:
      return "MidArm";
    case 3:
      return "LowerArm";
    case 4:
      return "Wrist";
    case 5:
      return "Finger";
    default:
      return "UNKNOWN";
  }
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float reduceResolution(float cmd, bool isTrigger=false) {
  const int kfCount = 9;
  if (!isTrigger) { // If we're reducing the resolution of a joystick
    float keyframes[kfCount] = {
      CONTROLLER_ZERO_CUTOFF , 0.125, 0.25 , 0.375, 0.5  , 0.625, 0.75 , 0.875, 1.0
    };

    float abs = std::abs(cmd);
    for (int i = 0; i < kfCount; i++) {
      float kfVal = keyframes[i];
      if (abs <= kfVal) {
        if (kfVal == CONTROLLER_ZERO_CUTOFF) return 0;
        if (cmd > 0) {
          return kfVal;
        } else if (cmd < 0) {
          return -1.0 * kfVal;
        }
      }
    }
  } else { // If we're reducing the resolution of a trigger
    float keyframes[kfCount] = {
      1.0  ,  0.75,  0.5 ,  0.25,  0.  , -0.25, -0.5 , -0.75, -1.0
    };
    for (int i = 0; i < kfCount; i++) {
      if (cmd >= keyframes[i]) {
        return keyframes[i];
      }
    }
  }
}

// Either a positive or negative value comes in (0-127 each) (positive eval first), and if its digital, a 1 gets added, if its not digital,
void interpretButtonBinary(int jointNum, char pos, char neg, char states[6]) {
  if (pos) {
    states[jointNum] = pos;

    std::string disp = translate(jointNum) + " Pos";
    // ROS_INFO("%s", disp.c_str());
  } else if (neg) {
    states[jointNum] = -1 * neg;

    std::string disp = translate(jointNum) + " Neg";
    // ROS_INFO("%s", disp.c_str());
  } else {
    states[jointNum] = 0;
  }
  return; // Unneccesary but idgaf
}

void interpretStickAnalog(int jointNum, float position, char states[6]) {
  float reduced = reduceResolution(position);
  char mapped = (char) map(reduced, -1.0, 1.0, -127, 127);
  states[jointNum] = mapped;
}

void interpretTriggers(int jointNum, float posTrig, float negTrig, char states[6]) {
  float reducedPos = reduceResolution(posTrig, true);
  char mappedPos = (char) map(reducedPos, 1.0, -1.0, 0, 127);
  float reducedNeg = reduceResolution(negTrig, true);
  char mappedNeg = (char) map(reducedNeg, 1.0, -1.0, 0, -127);

  // Handle the case where if the triggers havent moved, they rest at 0, which is stupid as FUUUUCK
  if (posTrig != 0.0) {
    posTrigMoved = true;
  }

  if (negTrig != 0.0) {
    negTrigMoved = true;
  }

  if (posTrig != 1.0 && posTrigMoved) { // Where 1.0 is the resting state of a trigger
    states[jointNum] = mappedPos;
  } else if (negTrig != 1.0 && negTrigMoved) {
    states[jointNum] = mappedNeg;
  } else {
    states[jointNum] = 0;
  }
}

void publishArmCommand(struct mycmd cmds)
{
  if (cmds.button_middle) {
    if (!middle_has_been_cleared) {
      // ROS_INFO("Middle not clear");
      return;
    }
    middle_has_been_cleared = false;
    std_msgs::Empty stop_msg;
    arm_stop_pub.publish(stop_msg);
    return;
  } else {
    middle_has_been_cleared = true;
  }

  arm_joy_control::ArmJoyMsg arm_cmd;
  char states[] = { 0, 0, 0, 0, 0, 0 };

  interpretStickAnalog(SHOULDER, cmds.axis_left_stick_x, states);
  interpretButtonBinary(UPPER_ARM, cmds.axis_dpad_y > 0.5, cmds.axis_dpad_y < -0.5, states);
  interpretButtonBinary(MID_ARM,  cmds.button_a, cmds.button_y, states);
  interpretStickAnalog(LOWER_ARM, cmds.axis_right_stick_y, states);
  interpretTriggers(WRIST, cmds.axis_lt, cmds.axis_rt, states);
  interpretButtonBinary(FINGER, cmds.button_start, cmds.button_back, states);

  // Detect if smething has changed
  bool advance = false;
  for (int i = 0; i < 6; i++) {
    if (states[i] != last_cmd[i]) {
      advance = true;
    }
  }

  // If nothing new, return
  if (!advance) {
    return;
  }

  for (int i = 0; i < 6; i++) {
    last_cmd[i] = states[i];
  }

  arm_cmd.shoulder = states[SHOULDER];
  arm_cmd.upperArm = states[UPPER_ARM];
  arm_cmd.midArm = states[MID_ARM];
  arm_cmd.lowerArm = states[LOWER_ARM];
  arm_cmd.wrist = states[WRIST];
  arm_cmd.finger = states[FINGER];

  arm_commands_pub.publish(arm_cmd);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  publishArmCommand(l.getCmd(joy));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_bad_control");

  ros::NodeHandle n;

  joy_sub = n.subscribe<sensor_msgs::Joy>("/arm/joy", 10, joyCallback);
  arm_commands_pub = n.advertise<arm_joy_control::ArmJoyMsg>("/arm/base_commands", 1000);
  arm_stop_pub = n.advertise<std_msgs::Empty>("/arm/stop", 10);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
