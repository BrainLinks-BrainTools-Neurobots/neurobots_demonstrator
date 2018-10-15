/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: goal_spec.cpp
 */

#include <goal_planner_gui/existential_partition_entry.h>
#include <goal_planner_gui/universal_partition_entry.h>
#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>

namespace goal_planner_gui
{

struct GoalSorter
{
	GoalSorter(const std::shared_ptr<GoalContext>& context) :
					context(context)
	{
	}
	bool operator()(const std::shared_ptr<GoalSpec>& a,
			const std::shared_ptr<GoalSpec>& b)
	{
		double aScore = context->getGoalScore(a);
		double bScore = context->getGoalScore(b);
		return aScore < bScore || (aScore == bScore && a->str() < b->str());
	}

	const std::shared_ptr<GoalContext>& context;
};

GoalSpec::GoalSpec(const std::shared_ptr<GoalContext>& context,
		bool initial) :
				m_context(context),
				m_childrenComputed(false),
				m_fixScore(0),
				m_initial(initial)
{
}

GoalSpec::~GoalSpec()
{
}

bool GoalSpec::isEmpty()
{
	return next().empty();
}

const std::vector<std::shared_ptr<GoalSpec>>& GoalSpec::next()
{
	if (!m_childrenComputed)
	{
		getNext(m_children);
		GoalSorter sorter(m_context);
		std::sort(m_children.begin(), m_children.end(), sorter);
		m_childrenComputed = true;
	}

//	LOG_INFO("Children of " << str());
//	for (auto& it : m_children)
//	{
//		LOG_INFO("\t - " << it->str() << " with score: " << m_context->getGoalScore(it));
//	}

	return m_children;
}

const std::vector<std::shared_ptr<GoalSpec> >& GoalSpec::nextFlattened()
{
	return next();
}

std::string GoalSpec::str() const
{
	return "--";
}

bool GoalSpec::isReached()
{
	return getNumReachable() <= getNumReached();
}

bool GoalSpec::isReached(int numReachable)
{
	return numReachable <= getNumReached();
}

bool GoalSpec::isReachable()
{
	return getNumReachable() > 0;
}

bool GoalSpec::isReachable(int numReachable)
{
	return numReachable > 0;
}

double GoalSpec::getFixScore() const
{
	return m_fixScore;
}

void GoalSpec::setFixScore(double fixScore)
{
	m_fixScore = fixScore;
}

bool GoalSpec::isInitial() const
{
	return m_initial;
}

bool GoalSpec::isUnique()
{
	return next().size() == 1;
}

bool GoalSpec::isUniversial()
{
	for (auto& pe : getArgs())
	{
		if (isInstance(pe, UniversalPartitionEntry))
		{
			return true;
		}
	}

	if (m_value && isInstance(m_value, UniversalPartitionEntry))
	{
		return true;
	}

	return false;
}

bool GoalSpec::isExistential()
{
	for (auto& pe : getArgs())
	{
		if (isInstance(pe, ExistentialPartitionEntry))
		{
			return true;
		}
	}

	if (m_value && isInstance(m_value, ExistentialPartitionEntry))
	{
		return true;
	}

	return false;
}

const std::shared_ptr<GoalContext>& GoalSpec::getContext() const
{
	return m_context;
}

} /* namespace goal_planner_gui */
