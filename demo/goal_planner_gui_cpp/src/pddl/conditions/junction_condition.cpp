/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: junction_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/junction_condition.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/parser.h>

namespace pddl
{

JunctionCondition::JunctionCondition(const std::vector<std::shared_ptr<Condition> >& parts,
		std::shared_ptr<Scope> scope) :
				m_parts(parts)
{
	m_scope = scope;
}

JunctionCondition::~JunctionCondition()
{
}

void JunctionCondition::parse(const std::string& tag,
		std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope,
		std::vector<std::shared_ptr<Condition>>& parts)
{
	auto first = it->get(tag);
	for (auto it2 = it->current(); it2 != it->end(); ++it2)
	{
		auto part = *it2;
		if (part->isTerminal())
		{
			throw UnexpectedTokenError(part->token, "condition");
		}
		parts.push_back(Condition::parse(part, scope));
	}
}

void JunctionCondition::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	children.reserve(m_parts.size());
	for (auto& p : m_parts)
	{
		children.push_back(p);
	}
}

const std::vector<std::shared_ptr<Condition> >& JunctionCondition::getParts() const
{
	return m_parts;
}

std::string JunctionCondition::str() const
{
	return "JunctionCondition";
}

} /* namespace pddl */

