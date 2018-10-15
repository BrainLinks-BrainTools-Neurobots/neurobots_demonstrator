/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: alternative_function_goal.cpp
 */

#include <goal_planner_gui/goals/alternative_function_goal.h>

namespace goal_planner_gui
{

AlternativeFunctionGoal::AlternativeFunctionGoal(const std::shared_ptr<GoalContext>& context,
		const pddl::FunctionPtr& function,
		const PartitionEntryVector& args,
		const PartitionEntryPtr& value,
		bool initial) :
				FunctionGoal(context, function, args, value, initial)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

AlternativeFunctionGoal::~AlternativeFunctionGoal()
{
}

std::shared_ptr<GoalSpec> AlternativeFunctionGoal::getChild(const PartitionEntryVector& args)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

} /* namespace goal_planner_gui */
