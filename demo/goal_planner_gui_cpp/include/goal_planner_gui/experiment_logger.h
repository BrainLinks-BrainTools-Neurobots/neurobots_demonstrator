/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: experiment_logger.h
 */

#ifndef HD764D76C_1ECE_4989_BF0B_E65DFB672DBB
#define HD764D76C_1ECE_4989_BF0B_E65DFB672DBB
#include <string>

namespace goal_planner_gui
{

class ExperimentLogger
{
public:
	ExperimentLogger(bool autoLog = false);
	virtual ~ExperimentLogger();

	void logPerformance(const std::string& logstring,
			bool execute = false);

private:
	void createLogFiles();

private:
	std::string m_navigationFilePath;
	std::string m_actionFilePath;
	std::string m_performanceFilePath;
};

} /* namespace goal_planner_gui */

#endif /* HD764D76C_1ECE_4989_BF0B_E65DFB672DBB */
