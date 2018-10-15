/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: current_goal_item.h
 */

#ifndef H769B3E27_2EE7_4FFB_94C1_9438AFA48FC3
#define H769B3E27_2EE7_4FFB_94C1_9438AFA48FC3

#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/gui/ref_item.h>

namespace goal_planner_gui
{

class CurrentGoalItem: public RefItem
{
Q_OBJECT

Q_PROPERTY(int current_param READ getCurrentParameter CONSTANT)

public:
	CurrentGoalItem(const std::shared_ptr<GoalSpec>& goal);
	CurrentGoalItem(const CurrentGoalItem& other);
	virtual ~CurrentGoalItem();

	virtual int getCurrentParameter() const;

private:
	std::shared_ptr<GoalSpec> m_goal;
	int m_current;

};

} /* namespace goal_planner_gui */

//Q_DECLARE_METATYPE(goal_planner_gui::CurrentGoalItem);

#endif /* H769B3E27_2EE7_4FFB_94C1_9438AFA48FC3 */
