/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 19, 2017
 *      Author: kuhnerd
 * 	  Filename: proposition.cpp
 */

#include <goal_planner_gui/relaxed_exploration/proposition.h>

namespace goal_planner_gui
{

Proposition::Proposition(const pddl::FactPtr& fact,
		const std::vector<UnaryOpPtr>& children) :
				m_fact(fact),
				m_children(children)
{
	reset();
}

Proposition::~Proposition()
{
}

std::string Proposition::str()
{
	return std::to_string(m_costs) + ": " + m_fact->str();
}

void Proposition::reset()
{
	m_costs = -1;
	m_hasFired = false;
	m_reachedBy.reset();
}

void Proposition::fire(std::vector<UnaryOpPtr>& ops)
{
	for (auto& op : m_children)
	{
		op->m_hAddCosts += m_costs;
		--op->m_numConditions;
		ASSERT(op->m_numConditions >= 0);
		if (op->m_numConditions == 0)
		{
			ops.push_back(op);
		}
	}
	m_hasFired = true;

}

} /* namespace goal_planner_gui */

