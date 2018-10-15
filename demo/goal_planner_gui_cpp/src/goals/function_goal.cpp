/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: function_goal.cpp
 */

#include <goal_planner_gui/goals/function_goal.h>

namespace goal_planner_gui
{

FunctionGoal::FunctionGoal(const std::shared_ptr<GoalContext>& context,
		const pddl::FunctionPtr& function,
		const PartitionEntryVector& args,
		const PartitionEntryPtr& value,
		bool initial) :
				GoalSpec(context, initial)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

FunctionGoal::~FunctionGoal()
{
}

void FunctionGoal::getNext(std::vector<std::shared_ptr<GoalSpec>>& res)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

void FunctionGoal::getCompletedArgs(PartitionEntryVector& args)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

int FunctionGoal::argIndex(const PartitionEntryPtr& arg)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

const PartitionEntryVector& FunctionGoal::getArgs()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

PartitionEntryPtr FunctionGoal::getArg(int index)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

pddl::ConditionPtr FunctionGoal::pddlGoal()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

void FunctionGoal::getFutureArgs(PartitionEntryVector& args)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

std::string FunctionGoal::str() const
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

const PartitionEntryVector& FunctionGoal::allPartitions()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

PartitionEntryPtr FunctionGoal::getCurrentArg()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

int FunctionGoal::getNumReachable()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

int FunctionGoal::getNumReached()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

const std::vector<pddl::FactVector>& FunctionGoal::getReachableGoals()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

} /* namespace goal_planner_gui */

