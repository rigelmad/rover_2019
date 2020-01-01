#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <power_sensor/PowerMsg.h>
#include <base_station/GpsMsg.h>
#include <base_station/TrajectoryMsg.h>

#include <math.h>
#include <sstream>
#include <cmath>
#include <cstring>

ros::Publisher antenna_pos_pub;
ros::Publisher rover_trajectory_pub;
ros::Subscriber power_msg_sub;
ros::Subscriber bs_gps_sub;
ros::Subscriber antenna_zero_sub;

int16_t initial_zero_offset = 0;
bool newMessageReceived = false;

float lastSendAngle = 0;
// Threshold above which we send a new command to the base station to move its antennae
float MOVEMENT_THRESHOLD = 0.05;

double roverGps[2]; // { lat, lon }
double bsGps[2]; // { lat, lon }

/***** MATH HELPER FUNCTIONS *****/
double getRad(double deg) {
	const double halfC = M_PI/180;
	return deg * halfC;
}

double getDeg(double rad) {
	const double conv = 180/M_PI;
	return rad * conv;
}

//Uses haversine method to calculate distance between two decimal degree coordinates.
double getDistance(double coord1[], double coord2[]) {
	//necessary values to calcuate distance, must be in radians.
	double lat1, lat2, deltaLat, deltaLon;

	lat1 = getRad(coord1[0]);
	lat2 = getRad(coord2[0]);
	deltaLat = getRad(coord1[0] - coord2[0]);
	deltaLon = getRad(coord1[1] - coord2[1]);

	//a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
	double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
		cos(lat1) * cos(lat2) * sin(deltaLon / 2) * sin(deltaLon / 2);

	//c = 2 ⋅ atan2( √a, √(1−a) )
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	//distance = R_earth*c
	double d = 6378 * c;
	return d;
}

//returns compass bearing of vector between two coords, double degrees
double getBearing(double coord1[], double coord2[]) {
	//necessary values to calcuate distance, must be in radians.
	double lat1, lat2, deltaLat, deltaLon;

	lat1 = getRad(coord1[0]);
	lat2 = getRad(coord2[0]);
	deltaLat = getRad(coord1[0] - coord2[0]);
	deltaLon = getRad(coord1[1] - coord2[1]);

	double x = sin(deltaLon) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(deltaLon));

	//in radians
	double bear = fmod(atan2(x,y)+(2*M_PI),2*M_PI);
	//double bear = atan2(x, y);
	return getDeg(bear);
}

void bs_gps_cb(const base_station::GpsMsg::ConstPtr& msg) {
  bsGps[0] = (double) msg->latitude;
  bsGps[1] = (double) -1.0 * msg->longitude;
  newMessageReceived = true;
  return;
}

void power_cb(const power_sensor::PowerMsg::ConstPtr& msg) {
  roverGps[0] = (double) msg->latitude;
  roverGps[1] = (double) msg->longitude;
  newMessageReceived = true;
  return;
}

void antenna_zero_cb(const std_msgs::Int16::ConstPtr& msg) {
  initial_zero_offset = msg->data;
  ROS_WARN("NEW ANTENNA ZERO: %d", initial_zero_offset);
  return;
}

// Also corrects for the initial offset angle
void moveAntenna(int angle) {
  angle = angle - initial_zero_offset;
  ROS_INFO("Moving antenna to: %d", angle);
  std_msgs::Int16 sendAngle;
  sendAngle.data = angle;
  antenna_pos_pub.publish(sendAngle);
}

void processNewInfo() {
  // double roverDistance = getDistance(roverGps, bsGps);
  // double roverBearing = getBearing(roverGps, bsGps);
  double roverDistance = getDistance(bsGps, roverGps);
  double roverBearing = getBearing(bsGps, roverGps);

  if (std::abs(roverBearing - lastSendAngle) > MOVEMENT_THRESHOLD) {
    moveAntenna((int) round(roverBearing));
    lastSendAngle = roverBearing;
  }

  base_station::TrajectoryMsg traj;
  traj.roverDistance = roverDistance;
  traj.roverBearing = roverBearing;
  rover_trajectory_pub.publish(traj);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bs_control");

  ros::NodeHandle nh;
  antenna_pos_pub = nh.advertise<std_msgs::Int16>("/base_station/antenna_angle", 100);
  rover_trajectory_pub = nh.advertise<base_station::TrajectoryMsg>("diagnostics/rover_trajectory", 100);

  power_msg_sub = nh.subscribe<power_sensor::PowerMsg>("/power", 100, power_cb);
  bs_gps_sub = nh.subscribe<base_station::GpsMsg>("/base_station/gps", 100, bs_gps_cb);
  antenna_zero_sub = nh.subscribe<std_msgs::Int16>("/base_station/zero", 10, antenna_zero_cb);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();

    if (newMessageReceived) {

			// Ensure we only take action if we don't have garbage data
			bool shouldProcess = true;
			if (bsGps[0] == 0.00 && bsGps[1] == 0.00) {
				ROS_WARN("Garbage base station coords, not interpreting.");
				shouldProcess = false;
			}
			if (roverGps[0] == 0.00 && roverGps[1] == 0.00) {
				ROS_WARN("Garbage rover coords, not interpreting.");
				shouldProcess = false;
			}

			if (shouldProcess) processNewInfo();

			newMessageReceived = false;
    }

    loop_rate.sleep();
  }


  return 0;
}
