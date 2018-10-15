/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: conditional_effect.cpp
 */
#include <goal_planner_gui/pddl/effects/conditional_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>

namespace pddl
{
ConditionalEffect::ConditionalEffect()
{
}

std::shared_ptr<Effect> ConditionalEffect::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	throw NotImplementedException(FILE_AND_LINE);
}

void ConditionalEffect::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	throw NotImplementedException(FILE_AND_LINE);
}

void ConditionalEffect::collectFreeVars(std::vector<std::shared_ptr<Parameter> >& res)
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<BaseElement> ConditionalEffect::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

}

