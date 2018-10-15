/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 11, 2017
 *      Author: kuhnerd
 * 	  Filename: detect_mouth.cpp
 */

#include <mouth_detection/mouth_detector.h>
#include <ros/init.h>

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "mouth_detector");

	MouthDetector md;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Rate rate(10.0);

	while (ros::ok())
	{
		rate.sleep();
	}

	ros::waitForShutdown();

	return 0;
}
