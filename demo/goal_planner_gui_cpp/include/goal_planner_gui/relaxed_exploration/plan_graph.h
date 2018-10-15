/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 19, 2017
 *      Author: kuhnerd
 * 	  Filename: plan_graph.h
 */

#ifndef H8083DC49_8B92_48E2_89BC_0B5D11286AE8
#define H8083DC49_8B92_48E2_89BC_0B5D11286AE8
#include <boost/variant/variant.hpp>
#include <goal_planner_gui/relaxed_exploration/proposition.h>
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/conditions/condition.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <goal_planner_gui/pddl/state/state.h>

namespace goal_planner_gui
{

class PlanGraph
{
public:
	struct SmartInstantiateFunctionData
	{
		std::unordered_set<pddl::ConditionPtr, pddl::BaseElementWithInstanceHasher, pddl::BaseElementWithInstanceEqualTo> checked;
		std::unordered_map<pddl::TypedObjectPtr, pddl::TypedObjectPtr, pddl::TypedObjectHasher> prevMapping;
		std::unordered_map<pddl::TypedObjectPtr, std::unordered_set<pddl::ConditionPtr, pddl::BaseElementWithInstanceHasher, pddl::BaseElementWithInstanceEqualTo>, pddl::TypedObjectHasher> condByArg;
		std::unordered_map<pddl::ConditionPtr, std::unordered_set<pddl::ParameterPtr, pddl::TypedObjectHasher>, pddl::BaseElementHasher> freeArgs;
//		std::unordered_map<pddl::TypedObjectPtr, std::unordered_set<pddl::ConditionPtr, pddl::BaseElementWithInstanceHasher, pddl::BaseElementWithInstanceEqualTo>, pddl::TypedObjectHasher> condByArg;
	};

	PlanGraph(const pddl::ProblemPtr& problem);
	virtual ~PlanGraph();

	bool isRelaxedReachable(const pddl::FactPtr& fact);
	double getFFDistance(const pddl::FactVector& facts);

protected:

	void computeActionPreconds();
	void computeReachable();
	void computeRelaxedDistance();

	pddl::Scope::SmartInstantiateFunction getInstFunc(const pddl::ActionPtr& action,
			const pddl::FactUnorderedSet& reachable,
			const pddl::FunctionUnorderedSet& staticFunctions,
			SmartInstantiateFunctionData& data);

private:
	void collectRelaxedPlan(const PropositionPtr& prop,
			UnaryOpVector& plan);

private:

	pddl::ProblemPtr m_problem;
	pddl::DomainPtr m_domain;
	pddl::StatePtr m_init;
	UnaryOpVector m_ops;

	std::unordered_map<pddl::ActionPtr,
			std::unordered_map<pddl::FunctionPtr,
					std::vector<boost::variant<int, pddl::TypedObjectPtr>>,
					pddl::FunctionHasher>,
			pddl::ActionHasher> m_precondLookup;
	std::unordered_map<pddl::FactPtr, PropositionPtr, pddl::FactHasher> m_props;
	std::vector<PropositionPtr> m_facts;
	pddl::FactUnorderedSet m_reachable;
};

} /* namespace goal_planner_gui */

#endif /* H8083DC49_8B92_48E2_89BC_0B5D11286AE8 */
