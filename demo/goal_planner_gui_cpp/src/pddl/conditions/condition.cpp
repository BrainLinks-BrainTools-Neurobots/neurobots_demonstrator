/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/condition.h>
#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/conditions/existential_condition.h>
#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/conditions/universal_condition.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>

namespace pddl
{

Condition::Condition()
{
}

Condition::~Condition()
{
}

std::shared_ptr<Condition> Condition::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	auto first = it->peek(Element::ExpectedTypeNone, "condition");
	auto& tag = first->token.string;

	std::shared_ptr<Condition> cond;

	if (tag == "and")
	{
		cond = Conjunction::parse(it, scope);
	}
	else if (tag == "and")
	{
		cond = Disjunction::parse(it, scope);
	}
	else if (tag == "exists")
	{
		cond = ExistentialCondition::parse(it, scope);
	}
	else if (tag == "forall")
	{
		cond = UniversalCondition::parse(it, scope);
	}

	if (cond)
	{
		return cond;
	}

	if (tag == "not")
	{
		first = it->get();
		cond = parse(it->get(Element::ExpectedTypeList, "condition"), scope);
		return cond->negated();
	}
	else if (tag == "imply")
	{
		throw NotImplementedException(FILE_AND_LINE);
	}
	else if (tag == "intermediate" || tag == "sometime")
	{
		throw NotImplementedException(FILE_AND_LINE);
	}
	else
	{
		return LiteralCondition::parse(it, scope);
	}

	throw NotImplementedException(FILE_AND_LINE);

}

void Condition::collectConditions(std::vector<std::shared_ptr<LiteralCondition> >& res)
{
	BaseElementVector children;
	getChildren(children);
	for (auto& it : children)
	{
		if (isInstanceSharedCast(it, c, LiteralCondition))
		{
			c->collectConditions(res);
		}
	}
}

void Condition::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	ConditionVector children;
	getChildrenAs<Condition>(children);
	for (auto& it : children)
	{
		it->collectFreeVars(res);
	}
}

std::string Condition::str() const
{
	return "Condition";
}

}
/* namespace pddl */

