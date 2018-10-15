/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: quantified_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/quantified_condition.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

QuantifiedCondition::QuantifiedCondition()
{
}

QuantifiedCondition::~QuantifiedCondition()
{
}

std::shared_ptr<Condition> QuantifiedCondition::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> action)
{
	throw NotImplementedException(FILE_AND_LINE);
}

void QuantifiedCondition::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

void QuantifiedCondition::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

std::string QuantifiedCondition::str() const
{
	return "QuantifiedCondition";
}

std::shared_ptr<BaseElement> QuantifiedCondition::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}
} /* namespace pddl */

