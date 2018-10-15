/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: disjunction.cpp
 */

#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

Disjunction::Disjunction(const std::vector<std::shared_ptr<Condition> >& parts,
		std::shared_ptr<Scope> scope) :
				JunctionCondition(parts, scope)
{
}

Disjunction::~Disjunction()
{
}

std::shared_ptr<Condition> Disjunction::negated()
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Condition> Disjunction::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> action)
{
	ConditionVector parts;
	JunctionCondition::parse("or", it, action, parts);
	return std::shared_ptr<Condition>(new Disjunction(parts));
}

std::string Disjunction::str() const
{
	std::string text;
	for (auto& c : m_parts)
	{
		text += "(" + c->str() + ")\n or ";
	}
	for (int i = 0; i < 5; ++i)
	{
		text.pop_back();
	}

	return "Disjunction: " + text;
}

std::shared_ptr<BaseElement> Disjunction::copy()
{
	std::vector<std::shared_ptr<Condition> > parts;
	for (auto& it : m_parts)
	{
		parts.push_back(it->getCopy<Condition>());
	}

	std::shared_ptr<Disjunction> cond(new Disjunction(parts, m_scope));

	return cond;
}

} /* namespace pddl */

