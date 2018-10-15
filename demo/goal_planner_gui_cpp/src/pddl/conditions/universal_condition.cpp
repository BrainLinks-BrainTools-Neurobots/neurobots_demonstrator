/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: universal_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/universal_condition.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

UniversalCondition::UniversalCondition()
{
}

UniversalCondition::~UniversalCondition()
{
}

std::shared_ptr<Condition> UniversalCondition::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> action)
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Condition> UniversalCondition::negated()
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::string UniversalCondition::str() const
{
	return "UniversalCondition";
}

} /* namespace pddl */

