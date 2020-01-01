#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controls/ControlCmd.h"
#include "controls/logitech_controller.cpp"
#include <sensor_msgs/Joy.h>
#include <drive_control/DriveCommand.h>
#include <std_msgs/Empty.h>

#include <sstream>
#include <cmath>

ros::Subscriber joy_sub;
ros::Publisher drive_commands_pub;
ros::Publisher drive_stop_pub;

LogController l;

//keeps track of drive sensitivity
float sensitivity = 0.25;

//How much to increment/decrement by on each button press
const float SENS_INC = 0.25;
//Max val for sensitivity
const float SENS_MAX = 1.0;
//Min vall for sensitivity
const float SENS_MIN = 0.25;

bool IS_HARDSTOPPED = false;

const float CONTROLLER_ZERO_CUTOFF = 0.05;

float last_left = 0;
float last_right = 0;
int last_up = 0;
int last_down = 0;
int x_has_been_cleared = true;
int middle_has_been_cleared = true;

void print_cmds(struct mycmd cmds) {
  ROS_INFO("\nLeft Stick: %6.4lf,\nRight Stick: %6.4lf,\nA: %d,\nB: %d,\nY: %d,\nX: %d\n",
            cmds.axis_left_stick_y,
            cmds.axis_right_stick_y,
            cmds.button_a,
            cmds.button_b,
            cmds.button_y,
            cmds.button_x);
}

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

void publishDriveCommand(struct mycmd cmds)
{

  float drive_cmd_left;
  float drive_cmd_right;

  // Handle both X and MIDDLE being stop buttons
  if (cmds.button_x) {
    if (!x_has_been_cleared) {
      ROS_INFO("X not clear");
      return;
    }
    x_has_been_cleared = false;
    std_msgs::Empty drive_stop_msg;
    drive_stop_pub.publish(drive_stop_msg);
    return;
  } else {
    x_has_been_cleared = true;
  }

  // If regular x is pressed, you'll return out before hitting this case
  if (cmds.button_middle) {
    if (!middle_has_been_cleared) {
      ROS_INFO("Middle not clear");
      return;
    }
    middle_has_been_cleared = false;
    std_msgs::Empty drive_stop_msg;
    drive_stop_pub.publish(drive_stop_msg);
    return;
  } else {
    middle_has_been_cleared = true;
  }

  /**
    Handle speedlocking, immediate 0 button
  */
  if (cmds.button_rb) { // speedlocking forward
    drive_cmd_left = cmds.axis_right_stick_y;
    drive_cmd_right = cmds.axis_right_stick_y; // Right is master stick
  } else if (cmds.button_lb) {
    drive_cmd_left = cmds.axis_right_stick_y * -1;
    drive_cmd_right = cmds.axis_right_stick_y; // Right is master stick, invert
  } else {
    drive_cmd_left = cmds.axis_left_stick_y;
    drive_cmd_right = cmds.axis_right_stick_y;
  }

  /**
    Handle sensitivity stuff
  */
  if (cmds.button_y) { //if sensitivityUp -> sensitivity += .25
    if (last_up == 0) {
      ROS_INFO("sensitivty: %2.2f", sensitivity);
      sensitivity += SENS_INC;
      last_up = 1;
    }
  } else {
    last_up = 0;
  }

  if (cmds.button_a) { //if sensitivityDown -> sensitivity -= .25
    if (last_down == 0) {
      sensitivity -= SENS_INC;
      ROS_INFO("sensitivty: %2.2f", sensitivity);
      last_down = 1;
    }
  } else {
    last_down = 0;
  }

  // Make sure the sensitivity stays within bounds
  if (sensitivity > SENS_MAX) {
    sensitivity = SENS_MAX;
  } else if (sensitivity < SENS_MIN) {
    sensitivity = SENS_MIN;
  }

  /**
    Apply sensitivity
  */
  drive_cmd_left *= sensitivity;
  drive_cmd_right *= sensitivity;

  // print_cmds(cmds);

  drive_control::DriveCommand driveCommand;

  drive_cmd_left = reduceResolution(drive_cmd_left);
  drive_cmd_right = reduceResolution(drive_cmd_right);

  driveCommand.left = drive_cmd_left;

  driveCommand.right = (-1 * drive_cmd_right); // CHANGED BECAUSE RIGHT SIDE WAS DRIVING IN REVERSE

  if (driveCommand.left == last_left && driveCommand.right == last_right) {
    // ROS_INFO("SAME COMMAND, RETURNING");
    return;
  } else {
    ROS_INFO("Drive Command Left: %2.3f", drive_cmd_left);
    ROS_INFO("Drive Command Right: %2.3f", drive_cmd_right);
  }

  drive_commands_pub.publish(driveCommand);

  last_left = driveCommand.left;
  last_right = driveCommand.right;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  publishDriveCommand(l.getCmd(joy));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control");

  ros::NodeHandle n;

  joy_sub = n.subscribe<sensor_msgs::Joy>("/drive/joy", 10, joyCallback);
  drive_commands_pub = n.advertise<drive_control::DriveCommand>("/drive/base_commands", 1000);
  drive_stop_pub = n.advertise<std_msgs::Empty>("/drive/stop", 10);

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
