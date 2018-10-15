/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Feb 15, 2018
 *      Author: kuhnerd
 * 	  Filename: planner.cpp
 */

#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/planning/planner.h>
#include <goal_planner_gui/planning/writer.h>
#include <boost/filesystem.hpp>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/state/state.h>

namespace goal_planner_gui
{

std::string exec(const std::string& cmd,
		const std::string& workingDir = "")
{
	std::array<char, 128> buffer;
	std::string result;

	std::string c = cmd;
	if (!workingDir.empty())
	{
		c = "cd " + workingDir + "; " + c;
	}

	std::shared_ptr<FILE> pipe(popen(c.c_str(), "r"), pclose);
	if (!pipe)
	{
		throw std::runtime_error("popen() failed!");
	}

	while (!feof(pipe.get()))
	{
		if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
			result += buffer.data();
	}

	return result;
}

Planner::Planner(const pddl::StatePtr& state,
		const std::string& path) :
				m_state(state),
				m_problem(state->getProblem()),
				m_packagePath(path),
				m_domainPath(path + "tmp/domain.pddl"),
				m_problemPath(path + "tmp/problem.pddl"),
				m_planPath(path + "tmp/sas_plan"),
				m_writer(new Writer(path + "/tmp/domain.pddl", path + "tmp/problem.pddl")),
				m_searchBin("fast-downward/fast-downward.py"),
				m_searchArgs("--search \"astar(cea())\""),
				m_tmpPath(m_packagePath + "tmp")
{
	setupFastDownward();
}

Planner::~Planner()
{
}

void Planner::setupFastDownward()
{
	if (!boost::filesystem::is_directory(m_tmpPath))
	{
		boost::filesystem::create_directory(m_tmpPath);
	}

	std::string p = m_packagePath + "fast-downward/builds/release64/bin";
	m_fastDownwardBuild = "release64";

	if (!boost::filesystem::is_directory(p))
	{
		p = m_packagePath + "fast-downward/builds/release32/bin";
		m_fastDownwardBuild = "release32";

		if (!boost::filesystem::is_directory(p))
		{
			throw pddl::Exception("You might need to compile fast-downward in this "
					"package by running build.py on the console!");
		}
	}
}

bool Planner::plan(const std::shared_ptr<GoalSpec>& goal)
{
	auto problem = m_problem->copy();
	pddl::ConditionPtr pddlGoal = goal->pddlGoal();

	m_writer->write(problem, problem->getDomain(), pddlGoal);

	const std::string planCmd = m_packagePath + m_searchBin + " --build " + m_fastDownwardBuild + " "
			+ m_domainPath + " " + m_problemPath + " " + m_searchArgs;
	LOG_INFO("plan cmd: " << planCmd);
//	std::string planOut =
	exec(planCmd, m_tmpPath);

	m_plan = Plan::parse(m_planPath, problem);

	if (m_plan)
	{
		return true;
	}
	else
	{
		return false;
	}
}

const std::shared_ptr<Plan>& Planner::getPlan() const
{
	return m_plan;
}

const pddl::ProblemPtr& Planner::getProblem() const
{
	return m_problem;
}

const std::string& Planner::getProblemPath() const
{
	return m_problemPath;
}

bool Planner::executeAction(const std::shared_ptr<PlanNode>& planNode)
{
	LOG_INFO("Execute action");

	auto& action = planNode->getAction();
	std::unordered_map<pddl::ParameterPtr, pddl::TypedObjectPtr, pddl::TypedObjectHasher> mapping;
	auto& args = planNode->getArgs();
	ASSERT(action->m_args.size() == args.size());

	for (size_t i = 0; i < action->m_args.size(); ++i)
	{
		mapping[action->m_args[i]] = args[i];
//		LOG_INFO("map " << action->m_args[i]->str() << " to " << args[i]->str())
	}

//	LOG_INFO("current: "<< m_state->str());
	action->instantiate(mapping, m_problem);
	if (!m_state->isExecutable(action))
	{
//		LOG_INFO("after inst: "<< m_state->str());
		action->uninstantiate();
		return false;
	}
	std::shared_ptr<pddl::State> newState = m_state->copy();
	newState->applyEffect(action->m_effect);
	action->uninstantiate();
//	LOG_INFO("after: "<< newState->str());

	pddl::InitLiteralUnorderedSet initFacts;
	for (auto& f: newState->iterfacts()) {
		initFacts.insert(f->toInit());
	}

	pddl::ProblemPtr newProb(new pddl::Problem(m_problem->m_name, m_problem->m_objects, m_problem->getDomain(), initFacts));

	m_problem = newProb;
	m_state = newState;

	return true;
}

} /* namespace goal_planner_gui */

