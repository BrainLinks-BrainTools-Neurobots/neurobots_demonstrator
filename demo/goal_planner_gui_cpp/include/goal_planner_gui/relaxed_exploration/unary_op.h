/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 19, 2017
 *      Author: kuhnerd
 * 	  Filename: unary_op.h
 */

#ifndef H789C11DE_0533_42E2_BD75_AF278DFDD9FD
#define H789C11DE_0533_42E2_BD75_AF278DFDD9FD
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>

namespace goal_planner_gui
{

class Proposition;

class UnaryOp
{
public:
	UnaryOp(const pddl::ActionPtr& action,
			const pddl::TypedObjectVector& args,
			const pddl::FactVector& conditions,
			const pddl::FactPtr& effect,
			double costs);
	virtual ~UnaryOp();

	static void fromAction(const pddl::ActionPtr& action,
			const pddl::TypedObjectVector& args,
			std::vector<std::shared_ptr<UnaryOp>>& ops);

	void reset();
	virtual std::string str() const;

	pddl::FactPtr m_effect;
	std::shared_ptr<Proposition> m_effectProposition;
	pddl::FactVector m_conditions;

	pddl::ActionPtr m_action;
	pddl::TypedObjectVector m_args;
	double m_opCosts;
	int m_numConditions;
	double m_hAddCosts;
};

POINTER_DEF(UnaryOp);
VECTOR_DEF(UnaryOp);

} /* namespace goal_planner_gui */

#endif /* H789C11DE_0533_42E2_BD75_AF278DFDD9FD */
