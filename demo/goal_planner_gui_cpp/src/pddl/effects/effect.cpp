/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: effect.cpp
 */
#include <goal_planner_gui/pddl/effects/conditional_effect.h>
#include <goal_planner_gui/pddl/effects/conjunctive_effect.h>
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/effects/universal_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/utils.h>

namespace pddl
{

Effect::Effect()
{
}

Effect::~Effect()
{
}

std::shared_ptr<Effect> Effect::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	auto first = it->peek(Element::ExpectedTypeNone, "effect");
	auto& tag = first->token.string;

	std::shared_ptr<Effect> eff;
	if (tag == "and")
	{
		eff = ConjunctiveEffect::parse(it, scope);
	}
	else if (tag == "when")
	{
		eff = ConditionalEffect::parse(it, scope);
	}
	else if (tag == "forall")
	{
		eff = UniversalEffect::parse(it, scope);
	}

	if (eff)
	{
		return eff;
	}

	return SimpleEffect::parse(it, scope);
}

void Effect::collectEffects(std::vector<std::shared_ptr<SimpleEffect> >& res)
{
	EffectVector children;
	getChildrenAs<Effect>(children);
	for (auto& it : children)
	{
		it->collectEffects(res);
	}
}

void Effect::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	EffectVector children;
	getChildrenAs<Effect>(children);
	for (auto& it : children)
	{
		it->collectFreeVars(res);
	}
}

std::string Effect::str() const
{
	return "Effect";
}

}

