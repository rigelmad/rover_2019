#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

ros::Publisher base_station_ping_pub;

ros::Time lastPingTime(0);

const double PING_SEND_INTERVAL_SEC = 1.0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "watchdog");

  ros::NodeHandle nh;
  // nh just uses the global sensor queue

  base_station_ping_pub = nh.advertise<std_msgs::Time>("diagnostics/ping", 1);

  ROS_INFO("pinger started");

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    ros::Duration timeSinceLastPing = now - lastPingTime;

    if (timeSinceLastPing.toSec() > PING_SEND_INTERVAL_SEC) {
      // ROS_INFO("Publishing ping: %6.4f", now.toSec());
      std_msgs::Time pubMe;
      pubMe.data = now;
      base_station_ping_pub.publish(pubMe);
      lastPingTime = now;
    }

    loop_rate.sleep();
  }

  return 0;
}
