#include <sensor_msgs/Joy.h>
#include "ControlCmd.h"

class LogController {
public:
  struct mycmd getCmd(const sensor_msgs::Joy::ConstPtr& joy);
};

/**
 Function to read the joy command and set things accordingly
*/
struct mycmd LogController::getCmd(const sensor_msgs::Joy::ConstPtr& joy) {
  mycmd ret;
  ret.axis_left_stick_x = joy->axes[LOG_AXIS_LEFT_X];
  ret.axis_left_stick_y = joy->axes[LOG_AXIS_LEFT_Y];
  ret.axis_right_stick_x = joy->axes[LOG_AXIS_RIGHT_X];
  ret.axis_right_stick_y = joy->axes[LOG_AXIS_RIGHT_Y];
  ret.axis_rt = joy->axes[LOG_AXIS_RT];
  ret.axis_lt = joy->axes[LOG_AXIS_LT];
  ret.axis_dpad_x = joy->axes[LOG_AXIS_DPAD_X];
  ret.axis_dpad_y = joy->axes[LOG_AXIS_DPAD_Y];
  ret.button_a = joy->buttons[LOG_BUTTON_A] == 1.0;
  ret.button_b = joy->buttons[LOG_BUTTON_B] == 1.0;
  ret.button_x = joy->buttons[LOG_BUTTON_X] == 1.0;
  ret.button_y = joy->buttons[LOG_BUTTON_Y] == 1.0;
  ret.button_lb = joy->buttons[LOG_BUTTON_LB] == 1.0;
  ret.button_rb = joy->buttons[LOG_BUTTON_RB] == 1.0;
  ret.button_back = joy->buttons[LOG_BUTTON_BACK] == 1.0;
  ret.button_middle = joy->buttons[LOG_BUTTON_MIDDLE] == 1.0;
  ret.button_l3 = joy->buttons[LOG_BUTTON_L3] == 1.0;
  ret.button_r3 = joy->buttons[LOG_BUTTON_R3] == 1.0;
  return ret;
}
