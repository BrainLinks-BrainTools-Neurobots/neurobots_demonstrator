/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: goal_spec.h
 */

#ifndef H9C543489_2F05_4ECA_983E_40B5C3F482F7
#define H9C543489_2F05_4ECA_983E_40B5C3F482F7
#include <goal_planner_gui/goals/goal_context.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/pddl/conditions/condition.h>
#include <goal_planner_gui/pddl/state/facts.h>

namespace goal_planner_gui
{

class GoalSpec: public std::enable_shared_from_this<GoalSpec>
{
public:
	GoalSpec(const std::shared_ptr<GoalContext>& context,
			bool initial = false);
	virtual ~GoalSpec();

	virtual const std::vector<std::shared_ptr<GoalSpec>>& next();
	virtual const std::vector<std::shared_ptr<GoalSpec>>& nextFlattened();
	virtual bool isEmpty();

	virtual std::string str() const;
	double getFixScore() const;
	void setFixScore(double fixScore);

	virtual int getNumReachable() = 0;
	virtual int getNumReached() = 0;

	virtual bool isReachable();
	virtual bool isReachable(int numReachable);
	virtual bool isReached();
	virtual bool isReached(int numReachable);

	virtual const PartitionEntryVector& allPartitions() = 0;

	virtual const std::vector<pddl::FactVector>& getReachableGoals() = 0;

	virtual void getNext(std::vector<std::shared_ptr<GoalSpec>>& res) = 0;
	bool isInitial() const;
	bool isUnique();
	bool isUniversial();
	bool isExistential();
	const std::shared_ptr<GoalContext>& getContext() const;

	virtual PartitionEntryPtr getArg(int index) = 0;
	virtual PartitionEntryPtr getCurrentArg() = 0;
	virtual int argIndex(const PartitionEntryPtr& arg) = 0;

	virtual const PartitionEntryVector& getArgs() = 0;

	virtual pddl::ConditionPtr pddlGoal() = 0;

protected:
	virtual void getCompletedArgs(PartitionEntryVector& args) = 0;
	virtual void getFutureArgs(PartitionEntryVector& args) = 0;

protected:
	std::shared_ptr<GoalContext> m_context;
	std::vector<std::shared_ptr<GoalSpec>> m_children;
	std::shared_ptr<GoalSpec> m_value;
	bool m_childrenComputed;
	double m_fixScore;
	bool m_initial;
};

} /* namespace goal_planner_gui */

#endif /* H9C543489_2F05_4ECA_983E_40B5C3F482F7 */
