/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: goal_context.cpp
 */

#include <goal_planner_gui/goals/goal_context.h>
#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/relaxed_exploration/plan_graph.h>
#include <memory>

namespace goal_planner_gui
{

GoalContext::GoalContext(const std::shared_ptr<pddl::Problem>& problem,
		std::shared_ptr<ReferenceList>& refs) :
				m_problem(problem),
				m_refs(refs),
				m_init(pddl::State::fromProblem(m_problem)),
				m_rpg(new PlanGraph(problem))
{
	for (auto& o : m_problem->m_objects)
		if (o != pddl::DefaultTypedObjects::unknownObject())
			m_objects.insert(o);
	for (auto& o : m_problem->getDomain()->m_constants)
		if (o != pddl::DefaultTypedObjects::unknownObject())
			m_objects.insert(o);

	calculateReachability();

	//define rpg
}

GoalContext::~GoalContext()
{
}

void GoalContext::calculateReachability()
{
	for (auto& a : m_problem->getDomain()->m_actions)
	{
		pddl::SimpleEffectVector res;
		if (a->m_effect)
		{
			a->m_effect->collectEffects(res);
			for (auto& eff : res)
			{
				m_mutableFunctions.insert(pddl::Function::getFunction(eff));
			}
		}
	}
}

double GoalContext::getGoalScore(const std::shared_ptr<GoalSpec>& goal)
{
	double fixScore = goal->getFixScore();
	if (fixScore > 0)
	{
		return fixScore;
	}

	if (goal->getNumReachable() == 0)
	{
		return 0;
	}

	auto& reachable = goal->getReachableGoals();
	int numReachable = goal->getNumReachable();
	int numReached = goal->getNumReached();
	double totalRelaxedDist = 0;
	for (auto& f : reachable)
	{
		totalRelaxedDist += m_rpg->getFFDistance(f);
	}

	double avgCost = (totalRelaxedDist + 1) / numReachable;

	return ((double)(numReachable - numReached)) / sqrt(avgCost);
}

} /* namespace goal_planner_gui */

