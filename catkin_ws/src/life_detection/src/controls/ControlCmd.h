#ifndef CONTROLCMD_H
#define CONTROLCMD_H

/**
  Defines the state each axis/button on the controller in a cohesive manner.
  Each controller implemented can be guaranteed to have a method that sets these
  fields appropriately, according to a joy message.

  In other words, I tried to create an interface (which C++) doesnt support.
*/
struct mycmd
{
  float axis_left_stick_x;
  float axis_left_stick_y;
  float axis_right_stick_x;
  float axis_right_stick_y;
  float axis_lt;
  float axis_rt;
  float axis_dpad_x;
  float axis_dpad_y;
  bool button_a;
  bool button_b;
  bool button_x;
  bool button_y;
  bool button_lb;
  bool button_rb;
  bool button_back;
  bool button_start;
  bool button_middle;
  bool button_l3;
  bool button_r3;
};

/**
 Defines the interface axes, buttons of a Logitech controller
 (the one we have in wireless)
*/
#define LOG_AXIS_LEFT_X 0
#define LOG_AXIS_LEFT_Y 1
#define LOG_AXIS_LT 2
#define LOG_AXIS_RIGHT_X 3
#define LOG_AXIS_RIGHT_Y 4
#define LOG_AXIS_RT 5
#define LOG_AXIS_DPAD_X 6
#define LOG_AXIS_DPAD_Y 7
#define LOG_BUTTON_A 0
#define LOG_BUTTON_B 1
#define LOG_BUTTON_X 2
#define LOG_BUTTON_Y 3
#define LOG_BUTTON_LB 4
#define LOG_BUTTON_RB 5
#define LOG_BUTTON_BACK 6
#define LOG_BUTTON_START 7
#define LOG_BUTTON_MIDDLE 8
#define LOG_BUTTON_L3 9
#define LOG_BUTTON_R3 10

#endif
