/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: alternative_action_goal.cpp
 */

#include <goal_planner_gui/goals/alternative_action_goal.h>

namespace goal_planner_gui
{

AlternativeActionGoal::AlternativeActionGoal(const std::shared_ptr<GoalContext>& context,
		const pddl::ActionPtr& action,
		const pddl::SimpleEffectVector& effects,
		const PartitionEntryVector& args,
		const pddl::TypedObjectVector& usedArgs,
		const std::shared_ptr<Partition>& forbidPartition,
		bool initial) :
				ActionGoal(context, action, effects, args, usedArgs, initial),
				m_forbidPartition(forbidPartition)
{
}

AlternativeActionGoal::~AlternativeActionGoal()
{
}

std::shared_ptr<GoalSpec> AlternativeActionGoal::getChild(const PartitionEntryVector& args)
{
	return std::shared_ptr<GoalSpec>(new ActionGoal(m_context, m_action, m_effects, args, m_usedArguments));
}

} /* namespace goal_planner_gui */
