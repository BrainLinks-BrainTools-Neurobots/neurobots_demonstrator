/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 31, 2017
 *      Author: kuhnerd
 * 	  Filename: main.cpp
 */
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <kuka_hand2eye_calibration/calibration.h>

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "kuka_hand2eye_calibration");
	ros::NodeHandle n;

	if (argc < 9)
	{
		ROS_ERROR("Usage: robot marker_file marker_size flange_frame world_frame [camera_prefix]");
		return 1;
	}

	std::string robot = argv[1];
	std::string markerFile = argv[2];
	double markerSize = std::atof(argv[3]);
	std::string flangeFrame = argv[4];
	std::string worldFrame = argv[5];
	std::string jointStateService = argv[6];
	std::string trajectoryFile = argv[7];
	std::string calibrationFile = argv[8];
	std::string cameraPrefix = "";

	if (argc == 10)
	{
		cameraPrefix = argv[9];
	}

	kuka_hand2eye_calibration::Calibration cal(robot,
			markerFile,
			markerSize,
			flangeFrame,
			worldFrame,
			jointStateService,
			trajectoryFile,
			calibrationFile,
			cameraPrefix);
	
	ros::AsyncSpinner spinner(4);
	spinner.start();	

	cal.run();

	ros::waitForShutdown();
}

