/*
 * command_line.cpp
 *
 *  Created on: Oct 2, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/command_line.h>
#include <goal_planner_gui/pddl/utils.h>

namespace goal_planner_gui
{

//ap.add_argument('--serialized-partitions', nargs=1, default=None, type=str, help="the filename of the serialized partition (without extension)")
//ap.add_argument('-d', '--debug', action="store_true")
//ap.add_argument('-r', '--refs', action="store_true", help="print possible references for types and exit")
//ap.add_argument('-t', '--dump-tree', action="store_true", help="generate and print all goals and exit")
//ap.add_argument('-i', '--interactive', action="store_true", help="interactive debug mode")
//ap.add_argument('-T', '--type', type=str, help="use <type> for any operations that operate on types")
//ap.add_argument('-e', '--evaluate', action="store_true", help="evaluate on the goals of the pddl problem")
//ap.add_argument('-x', '--executable', action="store_true", help="only executable actions")
//ap.add_argument('-o', '--objects', action="store_true", help="print references to each object in the problem and exit")
//ap.add_argument('-g', '--gui', action="store_true", help="show GUI")
//ap.add_argument('-m', '--mute', action="store_true", help="whether to mute or not the console output")
//ap.add_argument('--autoexecute', action="store_true", help="autoexecute next action in planning mode")
//ap.add_argument('--auto_perf_ex', action="store_true", help="auto select menu entries to test performance")
//ap.add_argument('-ros', '--ros', nargs='*', default=False)
//ap.add_argument('-it', '--image-topic', nargs='*', default=None, type=str, help="start execution with a specified" +
//                                                                                " list of image topics to subscribe.")
//ap.add_argument('-cit', '--cimage-topic', nargs='*', default=None, type=str,
//                help="start execution with a specified list of compressed image topics to subscribe.")
CommandLine::CommandLine()
{
	commandLineParser.addHelpOption();
	commandLineParser.addVersionOption();
	commandLineParser.setApplicationDescription("A PDDL goal generation tool");

	commandLineParser.addPositionalArgument("domain", "Planning domain");
	commandLineParser.addPositionalArgument("references", "file that contains information on what is referable");
	commandLineParser.addPositionalArgument("problem", "problem description as a pddl problem");

	debugOption = new QCommandLineOption(QStringList() << "d" << "debug", "Debug mode", QString(), "true");
	commandLineParser.addOption(*debugOption);
	debug = false;

	autoPerformExecutionOption = new QCommandLineOption("auto_perf_ex", "auto select menu entries to test performance", QString(), "false");
	commandLineParser.addOption(*autoPerformExecutionOption);
	autoPerformExecution = false;
}

CommandLine::~CommandLine()
{
	delete debugOption;
}

void CommandLine::process(QApplication& app)
{
	commandLineParser.process(app);
	if (commandLineParser.positionalArguments().size() != 3)
		commandLineParser.showHelp(1);

	if (commandLineParser.isSet(*debugOption))
		debug = true;

	if (commandLineParser.isSet(*autoPerformExecutionOption))
		autoPerformExecution = true;
}

} /* namespace goal_planner_gui */
