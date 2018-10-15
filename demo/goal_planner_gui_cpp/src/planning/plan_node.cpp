/*
 * plan_node.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: kuhnerd
 */

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <goal_planner_gui/planning/plan_node.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/exception.h>

namespace goal_planner_gui
{

PlanNode::PlanNode(const pddl::ActionPtr& action,
		const pddl::TypedObjectVector& args) :
				m_action(action),
				m_args(args),
				m_status(ACTION_STATUS_EXECUTABLE)
{
}

PlanNode::~PlanNode()
{
}

std::shared_ptr<PlanNode> PlanNode::parse(const std::string& line,
		const pddl::ProblemPtr& problem)
{
	std::vector<std::string> parts;
	boost::algorithm::split(parts, line, boost::algorithm::is_any_of(" "));

	std::string actionName = parts[0];
	std::vector<std::string> argNames;
	for (size_t i = 1; i < parts.size(); ++i)
		argNames.push_back(parts[i]);

	pddl::ActionPtr action = problem->getDomain()->getAction(actionName);

	if (!action)
	{
		throw pddl::Exception("Unknown action " + actionName);
	}

	pddl::TypedObjectVector args;
	for (auto& it : argNames)
	{
		pddl::TypedObjectPtr arg = problem->get(it);
		if (!arg)
		{
			throw pddl::Exception("Unknown argument " + it);
		}
		args.push_back(arg);
	}

	return std::shared_ptr<PlanNode>(new PlanNode(action, args));
}

const pddl::ActionPtr& PlanNode::getAction() const
{
	return m_action;
}

const pddl::TypedObjectVector& PlanNode::getArgs() const
{
	return m_args;
}


const std::string& PlanNode::getStatus() const
{
	return m_status;
}

void PlanNode::setStatus(const std::string& status)
{
	m_status = status;
}

std::string PlanNode::str()
{
	std::string args;
	for (auto& it: m_args) {
		args += it->str() + ", ";
	}
	REMOVE_LAST_COMMA(args);

	return m_action->m_name + " (" + args + ")";
}

} /* namespace goal_planner_gui */

