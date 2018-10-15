/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: current_goal_item.cpp
 */

#include <goal_planner_gui/goals/action_goal.h>
#include <goal_planner_gui/goals/function_goal.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/gui/current_goal_item.h>
#include <goal_planner_gui/pddl/exception.h>
#include <memory>

namespace goal_planner_gui
{

CurrentGoalItem::CurrentGoalItem(const std::shared_ptr<GoalSpec>& goal) :
				m_goal(goal),
				m_current(0)
{
	auto& refs = goal->getContext()->m_refs;
	if (isInstanceSharedCast(goal, fgoal, FunctionGoal))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(goal, agoal, ActionGoal))
	{
		m_nameItem = new ImageTextItem(refs->getName(agoal->getAction()->m_name), refs->getImage(agoal->getAction()->m_name));
	}

	m_current = goal->argIndex(goal->getCurrentArg());
	auto& args = goal->getArgs();
	for (auto& arg : args)
	{
		QList<QObject*> items;
		std::vector<PartitionEntry::DescriptionEntry> descriptions;
		arg->description(descriptions);
		for (auto& it : descriptions)
		{
			items.push_back(new ImageTextItem(it.text, it.image));
		}
		m_args.push_back(QVariant::fromValue(items));
	}
}

CurrentGoalItem::CurrentGoalItem(const CurrentGoalItem& other) :
				RefItem(other),
				m_current(other.m_current),
				m_goal(other.m_goal)
{
}

CurrentGoalItem::~CurrentGoalItem()
{
}

int CurrentGoalItem::getCurrentParameter() const
{
	return m_current;
}

} /* namespace goal_planner_gui */
