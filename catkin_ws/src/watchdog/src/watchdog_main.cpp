#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>

// multithreading/callbacks
#include <ros/callback_queue.h>

// drive/arm/ld messages
#include <drive_control/DriveCommand.h>
#include <arm_joy_control/ArmJoyMsg.h>

// sensor message inputs probably
#include <power_sensor/PowerMsg.h>
#include <rover_diagnostics/current_sense.h>

// Current thresholds
const uint16_t DRIVE_CURRENT_HIGH_MA = 5000;
const uint16_t ARM_CURRENT_HIGH_MA = 5000;

// Ping time threshold
const double PING_STOP_TIMEOUT_SEC = 1.5;

// pub subs
ros::Subscriber base_station_ping_sub;
ros::Subscriber power_board_sub;

ros::Subscriber drive_current_sense_sub;
ros::Subscriber arm_current_sense_sub;

ros::Subscriber drive_commands_sub;
ros::Subscriber drive_stop_toggle_sub;
ros::Publisher drive_commands_pub;

ros::Subscriber arm_commands_sub;
ros::Subscriber arm_stop_toggle_sub;
ros::Publisher arm_commands_pub;

ros::Time lastPingTime(0);

bool isPingStopped = false;
bool hasBeenCeased = false;
bool isDriveStopped = true; // Start in a stopped state
bool isArmStopped = true; // Start in a stopped state

/******* Helper functions ***********/
void stopDrive() {
  if (!isDriveStopped) {
    drive_control::DriveCommand stop_cmd;
    stop_cmd.left = 0;
    stop_cmd.right = 0;
    drive_commands_pub.publish(stop_cmd);
    isDriveStopped = true;
    ROS_WARN("Drive Stopped!");
  }
}

void stopArm() {
  if (!isArmStopped) {
    arm_joy_control::ArmJoyMsg arm_cmd; // Initializes to 0
    arm_commands_pub.publish(arm_cmd);
    isArmStopped = true;
    ROS_WARN("Arm Stopped!");
  }
}

// Sends stop commands to all systems that need to be stopped.
void ceaseAllMotorFunctions() {
  stopDrive();
  stopArm();
  return;
}

/******* POWER BOARD MESSAGE HANDLING ********/

void adc_check(int8_t *adc, int size) {
    int8_t ADC_THRESHOLD_LOW = 0;

    // + 1 is needed since indexed at 1
    for(int a = 0; a < size; a++) {
        if(adc[a] < ADC_THRESHOLD_LOW) {
            ROS_WARN("TOO LOW ADC%d = %d", a + 1, adc[a]);
        }
    }
}

void orientation_check(int8_t x_or, int8_t y_or, int8_t z_or) {
    // need logic for here
    // absolute values?
    // or are low and high thresholds distinct
}

void ultrasonic_check(int8_t *us, int size) {
    // need logic
    // what are expected values

    for(int u = 0; u < size; u++)
    {
        //need something here
    }
}

void temperature_check(int8_t temp) {
    // need logic
    // need expected values
}

void power_board_cb(const power_sensor::PowerMsg::ConstPtr& msg) {
    ROS_INFO("power_board_sensed");

    ROS_INFO("adc1: %d", msg->adc1);
    ROS_INFO("adc2: %d", msg->adc2);
    ROS_INFO("adc3: %d", msg->adc3);
    ROS_INFO("adc4: %d", msg->adc4);

    ROS_INFO("latitude: %d", msg->latitude);
    ROS_INFO("longitude: %d", msg->longitude);
    ROS_INFO("satellites: %d", msg->satellites);

    ROS_INFO("x-orientation: %d", msg->orientX);
    ROS_INFO("y-orientation: %d", msg->orientY);
    ROS_INFO("z-orientation: %d", msg->orientZ);

    ROS_INFO("ultrasonic1: %d", msg->ultrasonic1);
    ROS_INFO("ultrasonic2: %d", msg->ultrasonic2);
    ROS_INFO("ultrasonic3: %d", msg->ultrasonic3);
    ROS_INFO("ultrasonic4: %d", msg->ultrasonic4);

    ROS_INFO("temperatureC: %d", msg->temperatureC);

    int8_t adc_list[] = {msg->adc1, msg->adc2, msg->adc3, msg->adc3, msg->adc4};
    int8_t us_list[] = {msg->ultrasonic1, msg->ultrasonic2, msg->ultrasonic3, msg->ultrasonic4};

    adc_check(adc_list, 4);
    orientation_check(msg->orientX, msg->orientY, msg->orientZ);
    ultrasonic_check(us_list, 4);
    temperature_check(msg->temperatureC);
}

/**********************************************/

/************* CURRENT SENSE MESSAGES *********/

void drive_current_sense_cb(const rover_diagnostics::current_sense::ConstPtr& msg) {
  int16_t drive_currents[] = { msg->motor1,msg->motor2,msg->motor3,msg->motor4,msg->motor5,msg->motor6 };

  bool shouldStop = false; // Added this toggle to be able to see all offending motors before stopping
  for (int c = 0; c < 6; c++) {
    if (drive_currents[c] >= DRIVE_CURRENT_HIGH_MA) {
      ROS_WARN("TOO HIGH current on motor%d = %d", c + 1, drive_currents[c]);
      shouldStop = true;
    }
  }

  if (shouldStop) {
    stopDrive();
  }
}

void arm_current_sense_cb(const rover_diagnostics::current_sense::ConstPtr& msg) {
  int16_t arm_currents[] = { msg->motor1,msg->motor2,msg->motor3,msg->motor4,msg->motor5,msg->motor6 };

  bool shouldStop = false; // Added this toggle to be able to see all offending motors before stopping
  for (int c = 0; c < 6; c++) {
    if (arm_currents[c] >= ARM_CURRENT_HIGH_MA) {
      ROS_WARN("TOO HIGH current on motor%d = %d", c + 1, arm_currents[c]);
      shouldStop = true;
    }
  }

  if (shouldStop) {
    stopArm();
  }
}


/**********************************************/

void ping_cb(const std_msgs::Time::ConstPtr& msg) {
  // ros::Time pingTime = msg->data;
  ros::Time pingTime = ros::Time::now();
  // ROS_INFO("Ping received: %6.4f", pingTime.toSec());
  lastPingTime = pingTime;
}

void drive_stop_toggle_cb(const std_msgs::Empty::ConstPtr& msg) {
  if (!isDriveStopped) { //  Not yet stopped, we should stop
    stopDrive();
    isDriveStopped = true; // Left this duplicate functionality here for verbosity
  } else { // It is set, we should unset
    isDriveStopped = false;
  }

  ROS_INFO("Drive hardstop %s", isDriveStopped ? "ENGAGED" : "CLEAR");
  return;
}

void publish_drive_cmd(const drive_control::DriveCommand::ConstPtr& msg) {
  if (isPingStopped) {
    ROS_WARN("Ping stopped, not sending cmd!");
    return;
  } else if (isDriveStopped) {
    ROS_WARN("Drive hardstopped, not sending cmd!");
    return;
  } else {
    drive_commands_pub.publish(msg);
  }
}

void arm_stop_toggle_cb(const std_msgs::Empty::ConstPtr& msg) {
  if (!isArmStopped) { //  Not yet stopped, we should stop
    stopArm();
    isArmStopped = true; // Left this duplicate functionality here for verbosity
  } else { // It is set, we should unset
    isArmStopped = false;
  }

  ROS_INFO("Arm hardstop %s", isArmStopped ? "ENGAGED" : "CLEAR");
  return;
}

void publish_arm_cmd(const arm_joy_control::ArmJoyMsg::ConstPtr& msg) {
  if (isPingStopped) {
    ROS_WARN("Ping stopped, not sending cmd!");
    return;
  } else if (isArmStopped) {
    ROS_WARN("Arm hardstopped, not sending cmd!");
    return;
  } else {
    arm_commands_pub.publish(msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "watchdog");

  ros::NodeHandle nh;
  // nh just uses the global sensor queue

  base_station_ping_sub = nh.subscribe<std_msgs::Time>("diagnostics/ping", 1, ping_cb);
  power_board_sub = nh.subscribe<power_sensor::PowerMsg>("power_board_sensors", 1, power_board_cb);

  drive_current_sense_sub = nh.subscribe<rover_diagnostics::current_sense>("/drive/current", 1000, drive_current_sense_cb);
  arm_current_sense_sub = nh.subscribe<rover_diagnostics::current_sense>("/arm/current", 1000, arm_current_sense_cb);

  drive_commands_pub = nh.advertise<drive_control::DriveCommand>("/drive/commands", 1000);
  drive_commands_sub = nh.subscribe<drive_control::DriveCommand>("/drive/base_commands", 10, publish_drive_cmd);
  drive_stop_toggle_sub = nh.subscribe<std_msgs::Empty>("/drive/stop", 10, drive_stop_toggle_cb);

  arm_commands_pub = nh.advertise<arm_joy_control::ArmJoyMsg>("/arm/commands", 1000);
  arm_commands_sub = nh.subscribe<arm_joy_control::ArmJoyMsg>("/arm/base_commands", 10, publish_arm_cmd);
  arm_stop_toggle_sub = nh.subscribe<std_msgs::Empty>("/arm/stop", 10, arm_stop_toggle_cb);

  ROS_INFO("watchdog started");

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    ros::Duration timeSinceLastPing = now - lastPingTime;
    if (timeSinceLastPing.toSec() > PING_STOP_TIMEOUT_SEC) {
      if (!isPingStopped) ROS_WARN("Ping lost!");
      if (!hasBeenCeased) {
        ceaseAllMotorFunctions();
        hasBeenCeased = true;
      }
      isPingStopped = true;
    } else {
      if (isPingStopped) ROS_INFO("Ping ff restored!");
      hasBeenCeased = false;
      isPingStopped = false;
    }


    loop_rate.sleep();
  }

  return 0;
}
