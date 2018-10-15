/*
 * command_line.h
 *
 *  Created on: Oct 2, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_GOAL_PLANNER_GUI_CPP_INCLUDE_GOAL_PLANNER_GUI_COMMAND_LINE_H_
#define DEMO_GOAL_PLANNER_GUI_CPP_INCLUDE_GOAL_PLANNER_GUI_COMMAND_LINE_H_
#include <qapplication.h>
#include <qcommandlineparser.h>

namespace goal_planner_gui
{

class CommandLine
{
public:
	CommandLine();
	virtual ~CommandLine();
	void process(QApplication& app);

	QCommandLineParser commandLineParser;
	QCommandLineOption* debugOption;
	bool debug;

	QCommandLineOption* autoPerformExecutionOption;
	bool autoPerformExecution;
};

} /* namespace goal_planner_gui */

#endif /* DEMO_GOAL_PLANNER_GUI_CPP_INCLUDE_GOAL_PLANNER_GUI_COMMAND_LINE_H_ */
