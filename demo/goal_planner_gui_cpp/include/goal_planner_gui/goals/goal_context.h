/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: goal_context.h
 */

#ifndef HCC827B5B_65F7_4819_AA25_CC2054B0AA89
#define HCC827B5B_65F7_4819_AA25_CC2054B0AA89
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/state/state.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/ros_database.h>

namespace goal_planner_gui
{

class ReferenceList;
class GoalSpec;
class PlanGraph;

class GoalContext
{
public:
	GoalContext(const std::shared_ptr<pddl::Problem>& problem,
			std::shared_ptr<ReferenceList>& refs);
	virtual ~GoalContext();

	double getGoalScore(const std::shared_ptr<GoalSpec>& goal);

private:
	void calculateReachability();

public:
	pddl::ProblemPtr m_problem;
	std::shared_ptr<ReferenceList> m_refs;
	pddl::StatePtr m_init;
	pddl::TypedObjectUnorderedSet m_objects;
	pddl::FunctionUnorderedSet m_mutableFunctions;
	std::shared_ptr<PlanGraph> m_rpg;
};

struct GoalContextResult
{
	std::vector<ROSDatabase::StrippedMap> neurobotsDict;
	std::shared_ptr<GoalContext> goalContext;
};

} /* namespace goal_planner_gui */

#endif /* HCC827B5B_65F7_4819_AA25_CC2054B0AA89 */
