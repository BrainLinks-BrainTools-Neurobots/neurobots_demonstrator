/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: main.cpp
 */

#include <ros/ros.h>
#include <neurobots_database_gui/gui.h>
#include <qt5/QtWidgets/qapplication.h>
#include <csignal>

QApplication* app;

void signalHandler(int signum)
{
	ros::shutdown();
	app->exit();
	delete app;
	exit(0);
}

int main(int argc,
		char** argv)
{
	signal(SIGINT, signalHandler);

	app = new QApplication(argc, argv);
	ros::init(argc, argv, "neurobots_database_gui", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	neurobots_database_gui::GUI gui;
	gui.showMaximized();
	gui.setWindowTitle("Neurobots Database GUI");

	return app->exec();
}

