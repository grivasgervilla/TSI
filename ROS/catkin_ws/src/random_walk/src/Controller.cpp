#include "Controller.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>

Controller::Controller() {
	command_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	laser_sub = node.subscribe("base_scan", 1, &Controller::scanCallback, this);
}

void Controller::moveFoward() {
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	command_pub.publish(msg);
};

void Controller::turn() {
	geometry_msgs::Twist msg;
	msg.angular.z = FORWARD_SPEED_MPS;
	command_pub.publish(msg);
}

void Controller::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan) {
	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan-> angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	//ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M) {
		ROS_INFO("Turn for a while!");
		keep_moving = false;
	}
}

void Controller::startMoving() {
	ros::Rate rate(10);
	int turn_time = rand()%16 + 15;
	ROS_INFO("Start moving");

	while (ros::ok()) {
		if (keep_moving)
			moveFoward();
		else {
			turn();
			turn_time--;
			if (turn_time == 0) {
				turn_time = rand()%16 + 15;
				keep_moving = true;
			}
		}

		ros::spinOnce();
		rate.sleep();
	}
}
