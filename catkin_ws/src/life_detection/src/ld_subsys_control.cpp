#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controls/ControlCmd.h"
#include "controls/logitech_controller.cpp"
#include <sensor_msgs/Joy.h>
#include <life_detection/LDControl.h>

#include <sstream>
#include <cmath>
#include <cstring>

const int NUM_ACTIVE_BTNS = 15;
int last_active[NUM_ACTIVE_BTNS] = { 0 };

ros::Publisher ld_commands_pub;

LogController l;

const int MAX_PWM_SPEED_SCOOP = 150;
const int MAX_PWM_SPEED_ACTUATOR = 150;

const float CONTROLLER_ZERO_CUTOFF = 0.05;

float reduceResolution(float cmd) {
  float abs = std::abs(cmd);
  if (abs < CONTROLLER_ZERO_CUTOFF) {
    abs = 0;
  } else if (abs <= 0.125) {
    abs = 0.125;
  } else if (abs <= 0.25) {
    abs = 0.25;
  } else if (abs <= 0.375) {
    abs = 0.375;
  } else if (abs <= 0.50) {
    abs = 0.50;
  } else if (abs <= 0.625) {
    abs = 0.625;
  } else if (abs <= 0.75) {
    abs = 0.75;
  } else if (abs <= 0.875) {
    abs = 0.875;
  } else if (abs <= 1.00) {
    abs = 1.0;
  }

  if (cmd > 0) {
    return abs;
  } else if (cmd < 0) {
    return -1.0 * abs;
  }
}

int getLastActiveCmd() {
  for (int i = 0; i < NUM_ACTIVE_BTNS; i++) {
    if (last_active[i] != 0) {
      return i;
    }
  }
  return -1;
}

void publishCommand(struct mycmd cmds) {
  bool activeButtons[NUM_ACTIVE_BTNS] = {
    std::abs(cmds.axis_left_stick_y) > CONTROLLER_ZERO_CUTOFF,
    std::abs(cmds.axis_right_stick_y) > CONTROLLER_ZERO_CUTOFF,
    cmds.button_rb,
    cmds.axis_dpad_x < -0.5,
    cmds.button_a,
    cmds.button_x,
    cmds.button_y,
    cmds.button_b,
    cmds.button_lb,
    cmds.axis_dpad_x > 0.5,
    cmds.axis_dpad_y > 0.5,
    cmds.axis_dpad_y < -0.5,
    cmds.button_start,
    cmds.button_back,
    cmds.button_middle
  };

  // Stop more than one thing from being triggered at once accidentally
  int activeCount = 0;
  int activeInd = -1;
  for (int i = 0; i < NUM_ACTIVE_BTNS; i++) {
    if (activeButtons[i]) {
      activeInd = i;
      activeCount++;
    }
  }

  // Reject more than 1 active button
  if (activeCount > 1) {
    ROS_WARN("Trigger only one LD Command at a time");
    return;
  }

  int lastActiveCmd = getLastActiveCmd();
  // Block other things from being triggered if last sent hasnt been cleared
  for (int i = 0; i < NUM_ACTIVE_BTNS; i++) {
    if (lastActiveCmd == -1) break; // If we're clear, advance
    if (activeInd == lastActiveCmd) { // This cant be -1, since we escape it before
      if (activeInd == 0 || activeInd == 1) break; // Joystick inds are okay to repeat
      return;
    }
  }

  life_detection::LDControl ldmsg;

  switch (activeInd) {
    case -1: {
      if (lastActiveCmd == -1) { // If have already sent a clear command
        // ROS_INFO("Already sent clear command last time, returning");
        return;
      }
      ROS_INFO("Clear");
      // Clear everything
      for (int i = 0; i < NUM_ACTIVE_BTNS; i++) {
        last_active[i] = 0;
      }
      ld_commands_pub.publish(ldmsg); // Publish a clear message AND return
      return;
    }
    case 0: { // Left Stick
      int speedScoop = reduceResolution(cmds.axis_left_stick_y) * MAX_PWM_SPEED_SCOOP;
      if (speedScoop == last_active[activeInd]) return;
      ROS_INFO("Scoooop: %d", speedScoop);
      ldmsg.scoopSpeed = speedScoop;
      last_active[activeInd] = speedScoop;
      break;
    }
    case 1: { // Right Stick
      int speedAct = reduceResolution(cmds.axis_right_stick_y * -1.0) * MAX_PWM_SPEED_ACTUATOR;
      if (speedAct == last_active[activeInd]) return;
      ROS_INFO("Aaactuator: %d", speedAct);
      ldmsg.actuatorSpeed = speedAct;
      last_active[activeInd] = speedAct;
      break;
    }
    case 2: // RB
      ROS_INFO("Toggle Lights");
      ldmsg.toggleLights = 1;
      break;
    case 3: // Left
      ROS_INFO("Move Slurry");
      ldmsg.moveSlurry = 1;
      break;
    case 4: // A
      ROS_INFO("Site 1");
      ldmsg.site1 = 1;
      break;
    case 5: // X
      ROS_INFO("Site 2");
      ldmsg.site2 = 1;
      break;
    case 6: // Y
      ROS_INFO("Site 3");
      ldmsg.site3 = 1;
      break;
    case 7: // B
      ROS_INFO("Site 4");
      ldmsg.site4 = 1;
      break;
    case 8: // LB
      ROS_INFO("Site 5");
      ldmsg.site5 = 1;
      break;
    case 9: // Right
      ROS_INFO("Clean 1");
      ldmsg.clean1 = 1;
      break;
    case 10: // Up
      ROS_INFO("Clean 2");
      ldmsg.clean2 = 1;
      break;
    case 11: // Down
      ROS_INFO("Clean 3");
      ldmsg.clean3 = 1;
      break;
    case 12: // Start
      ROS_INFO("Predecessor Add Water");
      ldmsg.predAddWater = 1;
      break;
    case 13: // Back
      ROS_INFO("Purge");
      ldmsg.purge = 1;
      break;
    case 14: // Middle
      ROS_INFO("Hard Stop");
      ldmsg.hardStop = 1;
      break;
    default:
      ROS_ERROR("Wtf is activeInd: %d", activeInd);
      return;
  }

  ld_commands_pub.publish(ldmsg);
  if (activeInd == 0 || activeInd == 1) return; // Last active has already been set
  last_active[activeInd] = 1;
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  publishCommand(l.getCmd(joy));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ld_control");

  ros::NodeHandle n;

  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/life_detection/joy", 10, joyCallback);
  ld_commands_pub = n.advertise<life_detection::LDControl>("/life_detection/commands", 1000);

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
