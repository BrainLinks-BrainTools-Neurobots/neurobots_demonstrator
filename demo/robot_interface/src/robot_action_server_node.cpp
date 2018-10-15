/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: robot_action_server_node.cpp
 */

#include <robot_interface/robot_action_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_interface_server");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  robot_interface::RobotInterfaceServer interface_server;

  ros::waitForShutdown();

  return 0;
}
