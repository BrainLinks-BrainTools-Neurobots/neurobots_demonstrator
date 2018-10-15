/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: preference_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/preference_condition.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

PreferenceCondition::PreferenceCondition()
{
}

PreferenceCondition::~PreferenceCondition()
{
}

std::shared_ptr<Condition> PreferenceCondition::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> action)
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Condition> PreferenceCondition::negated()
{
	throw NotImplementedException(FILE_AND_LINE);
}

void PreferenceCondition::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	throw NotImplementedException(FILE_AND_LINE);
}

std::string PreferenceCondition::str() const
{
	return "PreferenceCondition";
}

std::shared_ptr<BaseElement> PreferenceCondition::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

} /* namespace pddl */
