/*
 * plan.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: kuhnerd
 */

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/regex.hpp>
#include <goal_planner_gui/planning/plan.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/utils.h>
#include <fstream>
#include <sstream>
#include <string>

namespace goal_planner_gui
{

Plan::Plan(const std::vector<std::shared_ptr<PlanNode>>& plan) :
				m_plan(plan)
{
}

Plan::~Plan()
{
}

std::shared_ptr<Plan> Plan::parse(const std::string& file,
		const pddl::ProblemPtr& problem)
{
	static const boost::regex re { "\\((.*)\\)" };

	std::ifstream f(file);
	if (!f.is_open())
	{
		LOG_ERROR("Cannot open file: " << file);
		return std::shared_ptr<Plan>();
	}

	std::string line;
	std::vector<std::shared_ptr<PlanNode>> plan;
	bool first = true;
	while (std::getline(f, line))
	{
		boost::algorithm::trim(line);
		if (boost::algorithm::starts_with(line, ";"))
		{
			continue;
		}

		boost::smatch m;
		boost::regex_search(line, m, re);
		if (m.size() != 2)
		{
			LOG_INFO(line);
			LOG_INFO("Size: " << m.size());
			throw pddl::Exception("Parse error: more/less than 2 elements in smatch");
		}

		plan.push_back(PlanNode::parse(m[1], problem));
	}

	return std::shared_ptr<Plan>(new Plan(plan));
}

std::vector<std::shared_ptr<PlanNode> >::iterator Plan::begin()
{
	return m_plan.begin();
}

std::vector<std::shared_ptr<PlanNode> >::const_iterator Plan::begin() const
{
	return m_plan.begin();
}

std::vector<std::shared_ptr<PlanNode> >::iterator Plan::end()
{
	return m_plan.end();
}

std::vector<std::shared_ptr<PlanNode> >::const_iterator Plan::end() const
{
	return m_plan.end();
}

std::shared_ptr<PlanNode>& Plan::operator [](const int index)
{
	return m_plan[index];
}

size_t Plan::size() const
{
	return m_plan.size();
}

} /* namespace goal_planner_gui */
