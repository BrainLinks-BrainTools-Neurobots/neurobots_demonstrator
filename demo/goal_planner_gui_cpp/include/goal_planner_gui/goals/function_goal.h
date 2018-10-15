/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: function_goal.h
 */

#ifndef H7B8E3EC9_3DED_475F_9839_094460034DF5
#define H7B8E3EC9_3DED_475F_9839_094460034DF5

#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/partition_entry.h>

namespace goal_planner_gui
{

class FunctionGoal: public GoalSpec
{
public:
	FunctionGoal(const std::shared_ptr<GoalContext>& context,
			const pddl::FunctionPtr& function,
			const PartitionEntryVector& args,
			const PartitionEntryPtr& value,
			bool initial = false);
	virtual ~FunctionGoal();

	virtual void getNext(std::vector<std::shared_ptr<GoalSpec>>& res);
	virtual int getNumReachable();
	virtual int getNumReached();

	virtual std::string str() const;

	virtual const std::vector<pddl::FactVector>& getReachableGoals();
	virtual const PartitionEntryVector& allPartitions();

	virtual PartitionEntryPtr getCurrentArg();
	virtual const PartitionEntryVector& getArgs();

	virtual int argIndex(const PartitionEntryPtr& arg);
	virtual PartitionEntryPtr getArg(int index);

	virtual pddl::ConditionPtr pddlGoal();

protected:
	virtual void getCompletedArgs(PartitionEntryVector& args);
	virtual void getFutureArgs(PartitionEntryVector& args);
};

} /* namespace goal_planner_gui */

#endif /* H7B8E3EC9_3DED_475F_9839_094460034DF5 */
