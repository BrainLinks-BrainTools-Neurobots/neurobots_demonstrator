/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 *
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: main.cpp
 */

#include <goal_planner_gui/command_line.h>
#include <goal_planner_gui/experiment_logger.h>
#include <goal_planner_gui/goals/goal_context.h>
#include <goal_planner_gui/gui/selector.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/ros_database.h>
#include <goal_planner_gui/reference_list.h>
#include <ros/init.h>
#include <ros/package.h>

using namespace goal_planner_gui;

std::shared_ptr<GoalContextResult> createGoalContext(std::shared_ptr<pddl::Domain> domain,
		std::shared_ptr<ROSDatabase> ros,
		const std::string& referenceFile)
{
	LOG_INFO("Fetching problem from ROS database");

	std::shared_ptr<GoalContextResult> res(new GoalContextResult());

	std::shared_ptr<pddl::Problem> problem;

	ros->getProblemFromDatabase(domain, problem, res->neurobotsDict);
	std::shared_ptr<ReferenceList> refs(new ReferenceList(problem));
	refs->init(referenceFile);
	refs->createAtomicPartitions();
	refs->createExtendedPartitions();
	refs->createOptimisticPartitions();

	res->goalContext.reset(new goal_planner_gui::GoalContext(problem, refs));

	return res;
}

//#include <unordered_map>
//#include <vector>
//#include <goal_planner_gui/pddl/types.h>
int main(int argc,
		char** argv)
{
//	std::unordered_map<pddl::TypedObjectPtr, pddl::TypedObjectVector> values;
//
//	pddl::TypedObjectPtr a(new pddl::TypedObject("test", std::shared_ptr<pddl::Type>(new pddl::Type("object"))));
//	pddl::TypedObjectPtr b(new pddl::TypedObject("test", pddl::DefaultTypes::objectType()));
//	pddl::TypedObjectPtr c(new pddl::TypedObject("test2", pddl::DefaultTypes::objectType()));
//	pddl::TypedObjectPtr d(new pddl::Parameter("?newbase", pddl::TypePtr(new pddl::Type("room"))));
//	d->instantiate(pddl::TypedObjectPtr(new pddl::TypedObject("dining-table", pddl::TypePtr(new pddl::Type("table")))));
//	pddl::TypedObjectPtr e(new pddl::Parameter("?newbase", pddl::TypePtr(new pddl::Type("room"))));
//	e->instantiate(pddl::TypedObjectPtr(new pddl::TypedObject("dining-table", pddl::TypePtr(new pddl::Type("table")))));
//	auto f = e;
//	auto g = f;
//	g.reset();
//
//	values[a].push_back(a);
//	values[a];
//	values[b].push_back(b);
//	values[d];
//	values[d];
//	values[e];
//	values[g];
//
//	LOG_INFO((a == c));
//	LOG_INFO((a == b));
//	LOG_INFO((c == b));
//	LOG_INFO("Contains: " << (CONTAINS(c, values)));
//	LOG_INFO("Contains: " << (CONTAINS(d, values)));
//	LOG_INFO("Contains: " << (CONTAINS(e, values)));
//	LOG_INFO("Contains: " << (CONTAINS(f, values)));
//	LOG_INFO("Contains: " << (CONTAINS(g, values)));
//
//	LOG_INFO(values.size());
//	LOG_INFO(values[a].size());
//
//	for (auto& it : values)
//	{
//		LOG_INFO("Contains: " << (CONTAINS(it.first, values)));
//	}
//	return 0;

	QApplication app(argc, argv);
	QApplication::setApplicationName("goal_planner_gui");
	QApplication::setApplicationVersion("1.0");
	QApplication::setQuitOnLastWindowClosed(true);

	QObject::connect( qApp, SIGNAL(lastWindowClosed()), qApp, SLOT(quit()) );

	ros::init(argc, argv, "goal_planner_gui");
	ros::NodeHandle n;

	CommandLine commandLine;
	commandLine.process(app);

	const QStringList args = commandLine.commandLineParser.positionalArguments();
	const std::string domainFile = args[0].toStdString();
	const std::string referencesFile = args[1].toStdString();
	const std::string problemFile = args[2].toStdString();

	LOG_INFO("Loading domain: " + domainFile);
	LOG_INFO("Loading references: " + referencesFile);
	LOG_INFO("Loading problem: " + problemFile);

	std::shared_ptr<pddl::Domain> domain = pddl::Domain::loadDomain(domainFile);
	std::shared_ptr<ROSDatabase> rosDatabase(new ROSDatabase);
	ExperimentLogger experimentLogger(commandLine.autoPerformExecution);
	LOG_INFO(std::string("Created logger with auto = ") + (commandLine.autoPerformExecution ? "true" : "false"));
	experimentLogger.logPerformance("App started");

	//get path
	const std::string configPath = ros::package::getPath("goal_planner_gui_cpp") + "/";

	std::shared_ptr<GoalContextResult> goalContext = createGoalContext(domain, rosDatabase, referencesFile);
	rosDatabase->setNeurobotsWorld(goalContext->neurobotsDict);

	std::shared_ptr<goal_planner_gui::Selector> plannerWidget(new goal_planner_gui::Selector(goalContext, configPath));
	plannerWidget->setupUI();
	rosDatabase->setSelector(plannerWidget);
	plannerWidget->setRosHandler(rosDatabase);

	return app.exec();
}

