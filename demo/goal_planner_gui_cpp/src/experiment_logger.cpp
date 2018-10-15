/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: experiment_logger.cpp
 */

#include <goal_planner_gui/experiment_logger.h>
#include <boost/filesystem.hpp>
#include <goal_planner_gui/pddl/utils.h>

#include <time.h>

#define ROS_EXPERIMENT_LOG_FOLDER "experiment_log/"

namespace goal_planner_gui
{

ExperimentLogger::ExperimentLogger(bool autoLog)
{
	createLogFiles();
}

ExperimentLogger::~ExperimentLogger()
{
}

void ExperimentLogger::createLogFiles()
{
	using namespace boost::filesystem;
	char* envPath = getenv("ROS_HOME");
	if (envPath == NULL)
	{
		envPath = getenv("HOME");
	}

	//create dir
	path directory = path(envPath) / path(".ros") / path(ROS_EXPERIMENT_LOG_FOLDER);
	if (!exists(directory))
		create_directory(directory);

	//get time string
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%%y_%m_%d_%H_%M_%S", timeinfo);
	std::string currentTimeString(buffer);

	m_navigationFilePath = (directory / path(currentTimeString + "_nav.log")).string();
	m_actionFilePath = (directory / path(currentTimeString + "_act.log")).string();
	m_performanceFilePath = (directory / path("performance.log")).string();

	LOG_INFO("performance file = " + m_performanceFilePath);
}

void ExperimentLogger::logPerformance(const std::string& logstring,
		bool execute)
{
}

} /* namespace goal_planner_gui */

