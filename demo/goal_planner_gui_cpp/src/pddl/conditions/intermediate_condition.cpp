/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: intermediate_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/intermediate_condition.h>
#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

IntermediateCondition::IntermediateCondition()
{
}

IntermediateCondition::~IntermediateCondition()
{
}

std::shared_ptr<Condition> IntermediateCondition::negated()
{
	throw NotImplementedException(FILE_AND_LINE);
}

void IntermediateCondition::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	throw NotImplementedException(FILE_AND_LINE);
}

size_t IntermediateCondition::hash() const
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

std::string IntermediateCondition::str() const
{
	return "IntermediateCondition";
}

std::shared_ptr<BaseElement> IntermediateCondition::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

} /* namespace pddl */
