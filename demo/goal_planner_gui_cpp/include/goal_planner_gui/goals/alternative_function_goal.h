/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: alternative_function_goal.h
 */

#ifndef HE4AF1938_52A2_4E0A_A0C4_3736318F2E5C
#define HE4AF1938_52A2_4E0A_A0C4_3736318F2E5C

#include <goal_planner_gui/goals/function_goal.h>

namespace goal_planner_gui
{

class AlternativeFunctionGoal: public FunctionGoal
{
public:
	AlternativeFunctionGoal(const std::shared_ptr<GoalContext>& context,
			const pddl::FunctionPtr& function,
			const PartitionEntryVector& args,
			const PartitionEntryPtr& value,
			bool initial = false);
	virtual ~AlternativeFunctionGoal();

protected:
	virtual std::shared_ptr<GoalSpec> getChild(const PartitionEntryVector& args);
};

} /* namespace goal_planner_gui */

#endif /* HE4AF1938_52A2_4E0A_A0C4_3736318F2E5C */
