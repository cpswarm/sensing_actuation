/*
 * battery_monitor.cpp
 *
 *  Created on: Sep 10, 2018
 *      Author: coriasco
 */


#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/BatteryState.h"
#include "ros/console.h"

std_msgs::Empty Empty;

float limit, period;
sensor_msgs::BatteryState current;
uint8_t panic = 0;

void panic_callback(const std_msgs::Empty msg) {
	ROS_INFO("Forced battery panic\n");
	panic = 1;
}

void check_battery(sensor_msgs::BatteryState msg) {
	current = msg;
}


int main(int argc, char **argv) {


	uint8_t published = 0, just_once = 0;
	current.percentage = 1.0;
	ros::init(argc, argv, "battery_monitor");
	ros::NodeHandle n;

	n.getParam(ros::this_node::getName() + "/battery_limit", limit);
	n.getParam(ros::this_node::getName() + "/advertise_period", period);

	printf("Got parameters:\n\tlimit: %f\n\tperiod: %f\n", limit, period);

	if (period == 0) {
		just_once = 1;
		period = 1.0;
	}
	ros::Subscriber battery_level = n.subscribe(ros::this_node::getNamespace() + "/mavros/battery", 10, check_battery);
	ros::Subscriber force_panic = n.subscribe(ros::this_node::getNamespace() + "/debug/battery_panic", 10, panic_callback);

	ros::Publisher DANGER_publisher = n.advertise<std_msgs::Empty> (ros::this_node::getNamespace() + "/danger/battery", 10);

	ros::Rate loop_rate(1.0/period);

	while(ros::ok()) {
		if((current.percentage <= limit || panic) && published == 0) {
			ROS_INFO("current percentage = %f", current.percentage);
			DANGER_publisher.publish(Empty);
			if(just_once)
				published = 1;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
