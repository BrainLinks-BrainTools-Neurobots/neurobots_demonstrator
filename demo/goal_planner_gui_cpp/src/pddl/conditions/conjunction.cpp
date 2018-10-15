/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: conjunction.cpp
 */

#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

Conjunction::Conjunction(const std::vector<std::shared_ptr<Condition>>& parts,
		std::shared_ptr<Scope> scope) :
				JunctionCondition(parts, scope)
{
}

Conjunction::~Conjunction()
{
}

std::shared_ptr<Condition> Conjunction::negated()
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Condition> Conjunction::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> action)
{
	ConditionVector parts;
	JunctionCondition::parse("and", it, action, parts);
	return std::shared_ptr<Condition>(new Conjunction(parts));
}

std::string Conjunction::str() const
{
	std::string text;
	for (auto& c : m_parts)
	{
		text += "(" + c->str() + ")\n and ";
	}
	for (int i = 0; i < 5; ++i)
	{
		text.pop_back();
	}

	return "Conjunction: " + text;
}

std::shared_ptr<BaseElement> Conjunction::copy()
{
	std::vector<std::shared_ptr<Condition> > parts;
	for (auto& it : m_parts)
	{
		parts.push_back(it->getCopy<Condition>());
	}

	std::shared_ptr<Conjunction> cond(new Conjunction(parts, m_scope));

	return cond;
}

} /* namespace pddl */

