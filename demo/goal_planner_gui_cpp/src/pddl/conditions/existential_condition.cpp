/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: existential_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/existential_condition.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

ExistentialCondition::ExistentialCondition()
{
}

ExistentialCondition::~ExistentialCondition()
{
}

std::shared_ptr<Condition> ExistentialCondition::negated()
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Condition> ExistentialCondition::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> action)
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::string ExistentialCondition::str() const
{
	return "ExistentialCondition";
}

} /* namespace pddl */
