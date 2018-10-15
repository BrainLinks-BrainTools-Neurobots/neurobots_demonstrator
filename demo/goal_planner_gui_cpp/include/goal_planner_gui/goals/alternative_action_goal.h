/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: alternative_action_goal.h
 */

#ifndef HF577B7B7_DD9A_43F2_9156_3897E85143FC
#define HF577B7B7_DD9A_43F2_9156_3897E85143FC

#include <goal_planner_gui/goals/action_goal.h>

namespace goal_planner_gui
{

class AlternativeActionGoal: public ActionGoal
{
public:
	AlternativeActionGoal(const std::shared_ptr<GoalContext>& context,
			const pddl::ActionPtr& action,
			const pddl::SimpleEffectVector& effects,
			const PartitionEntryVector& args,
			const pddl::TypedObjectVector& usedArgs,
			const std::shared_ptr<Partition>& forbidPartition,
			bool initial = false);
	virtual ~AlternativeActionGoal();

protected:
	virtual std::shared_ptr<GoalSpec> getChild(const PartitionEntryVector& args);

private:
	std::shared_ptr<Partition> m_forbidPartition;
};

} /* namespace goal_planner_gui */

#endif /* HF577B7B7_DD9A_43F2_9156_3897E85143FC */
