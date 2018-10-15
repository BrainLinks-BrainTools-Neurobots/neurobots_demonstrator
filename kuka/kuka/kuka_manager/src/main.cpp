/*
 * main.cpp
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */
#include <iostream>
#include "kuka_manager/KukaManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv,"KukaController");
    ros::NodeHandle nh;

	std::string robotname;
    nh.getParam("kuka_manager/robot_name", robotname);
	KUKA::Robotname robot;
	if (robotname == "Omnirob") {
		robot = KUKA::Robotname::Omnirob;
	} else if (robotname == "IIWA") {
		robot = KUKA::Robotname::IIWA;
	} else if (robotname == "Both") {
		robot = KUKA::Robotname::Both;
	} else {
		ROS_INFO("Robotname is not available..");
		ROS_INFO("You entered: %s", robotname.c_str());
		ROS_INFO("Available robots: [Omnirob] [IIWA] [Both]");
		return 1;
	}

	std::string host;
	nh.param<std::string>("kuka_manager/host", host, "localhost");
	ROS_INFO("Host: %s", host.c_str());

	// Init omnirob manager
	KukaManager kukaManager;
	kukaManager.init(host.c_str(), nh, robot);
	kukaManager.startThreads();

	// Spin
	kukaManager.spin();

	return 0;
}

