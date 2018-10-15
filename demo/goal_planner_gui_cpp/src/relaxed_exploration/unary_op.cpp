/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 19, 2017
 *      Author: kuhnerd
 * 	  Filename: unary_op.cpp
 */

#include <goal_planner_gui/relaxed_exploration/unary_op.h>
#include <goal_planner_gui/relaxed_exploration/proposition.h>
#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>

namespace goal_planner_gui
{

UnaryOp::UnaryOp(const pddl::ActionPtr& action,
		const pddl::TypedObjectVector& args,
		const pddl::FactVector& conditions,
		const pddl::FactPtr& effect,
		double costs) :
				m_action(action),
				m_args(args),
				m_conditions(conditions),
				m_effect(effect),
				m_opCosts(costs)
{
	reset();
}

UnaryOp::~UnaryOp()
{
}

void UnaryOp::fromAction(const pddl::ActionPtr& action,
		const pddl::TypedObjectVector& args,
		std::vector<std::shared_ptr<UnaryOp>>& ops)
{
	ops.clear();

	std::vector<std::shared_ptr<pddl::LiteralCondition>> conds;
	pddl::FactVector conditions;

	action->m_precondition->collectConditions(conds);

	for (auto& c : conds)
	{
		if (pddl::Function::getFunction(c) != pddl::Builtin::equals())
		{
			conditions.push_back(pddl::Fact::fromLiteral(c));
		}
	}

	double costs = action->getTotalCost();

	std::vector<std::shared_ptr<pddl::SimpleEffect>> effects;
	action->m_effect->collectEffects(effects);
	for (auto& eff : effects)
	{
		auto e = pddl::Fact::fromLiteral(eff);
		ops.push_back(std::shared_ptr<UnaryOp>(new UnaryOp(action, args, conditions, e, costs)));
	}
}

void UnaryOp::reset()
{
	m_numConditions = m_conditions.size();
	m_hAddCosts = m_opCosts;
}

std::string UnaryOp::str() const
{
	std::string args;
	for (auto& it : m_args)
	{
		args += it->getName() + " ";
	}

	if (!m_args.empty())
	{
		args.pop_back();
	}

	return m_action->m_name + " " + args + ": cost " + std::to_string(m_hAddCosts) + ", " + std::to_string(m_numConditions) + " remaining";
}

} /* namespace goal_planner_gui */
