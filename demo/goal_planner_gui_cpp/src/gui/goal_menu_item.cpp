/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: goal_menu_item.cpp
 */

#include <goal_planner_gui/goals/action_goal.h>
#include <goal_planner_gui/goals/alternative_action_goal.h>
#include <goal_planner_gui/goals/alternative_function_goal.h>
#include <goal_planner_gui/goals/function_goal.h>
#include <goal_planner_gui/gui/goal_menu_item.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/utils.h>
#include <memory>

namespace goal_planner_gui
{

GoalMenuItem::GoalMenuItem()
{
}

GoalMenuItem::GoalMenuItem(const std::shared_ptr<GoalSpec>& goal,
		int currentIndex) :
				m_goal(goal)
{
	if (goal)
	{
		if (isInstanceSharedCast(goal, agoal, AlternativeActionGoal))
		{
			m_arg.push_back(new ImageTextItem(QString("Get other attribute"), QString("")));
		}
		else if (isInstanceSharedCast(goal, agoal, AlternativeFunctionGoal))
		{
			m_arg.push_back(new ImageTextItem(QString("Get other attribute"), QString("")));
		}
		else
		{
			if (goal->isInitial())
			{
				auto& refs = m_goal->getContext()->m_refs;
				if (isInstanceSharedCast(goal, fgoal, FunctionGoal))
				{
					throw pddl::NotImplementedException(FILE_AND_LINE);
//					m_arg.push_back(new ImageTextItem(refs->getName(fgoal->)))
				}
				else if (isInstanceSharedCast(goal, agoal, ActionGoal))
				{
					m_arg.push_back(new ImageTextItem(refs->getName(agoal->getAction()->m_name), refs->getImage(agoal->getAction()->m_name)));
				}
				else
				{
					throw pddl::Exception("Unknown goal type: " + goal->str());
				}
			}
			else
			{
				auto arg = goal->getArg(currentIndex);
				std::vector<PartitionEntry::DescriptionEntry> descs;
				arg->description(descs);
				for (auto& it: descs)
				{
					m_arg.push_back(new ImageTextItem(it.text, it.image));
				}
			}

			if (goal->isUnique() and currentIndex < goal->allPartitions().size() - 1)
			{
				m_goal = goal->next()[0];
				auto& allPart = m_goal->allPartitions();
				for (size_t i = currentIndex + 1; i < allPart.size(); ++i)
				{
					std::vector<PartitionEntry::DescriptionEntry> argDescs;
					allPart[i]->description(argDescs);
					for (auto& it: argDescs)
					{
						m_arg.push_back(new ImageTextItem(it.text, it.image));
					}
				}
			}
		}
	}
}

GoalMenuItem::GoalMenuItem(const GoalMenuItem& other)
{
}

GoalMenuItem::~GoalMenuItem()
{
	for (auto& it : m_arg)
	{
		delete it;
	}
	m_arg.clear();
}

bool GoalMenuItem::initial() const
{
	LOG_INFO("Implement!");
	return true;
}

QString GoalMenuItem::getAction() const
{
	return "";
}

QVariant GoalMenuItem::getArgument() const
{
	return QVariant::fromValue(m_arg);
}

bool GoalMenuItem::more() const
{
	if (!m_goal || m_goal->isUnique())
		return false;
	return true;
}

bool GoalMenuItem::back() const
{
	return false;
}

} /* namespace goal_planner_gui */

