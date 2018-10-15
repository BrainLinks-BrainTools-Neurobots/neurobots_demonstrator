/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: main.cpp
 */

#include <neurobots_database/database_server.h>
#include <ros/ros.h>

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "neurobots_database");
	ros::NodeHandle n;

//	for (size_t i = 0; i< argc; ++i)
//		std::cout << "Argument [" << i << "] : " << argv[i] << std::endl;
//

//if you don't provide the argument you can use the parameter
//server to specify the files directly (see database.cpp, init)
	std::string arg = argc > 1 ? argv[1] : "";
	neurobots_database::DatabaseServer world_database_server = neurobots_database::DatabaseServer(arg);

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}

