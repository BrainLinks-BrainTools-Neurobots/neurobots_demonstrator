/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: universial_effect.cpp
 */
#include <goal_planner_gui/pddl/effects/universal_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/types.h>

using pddl::TypedObject;

namespace pddl
{
UniversalEffect::UniversalEffect(const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& args,
		std::shared_ptr<Effect> effect,
		std::shared_ptr<Scope> parentScope) :
				Scope(std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>(args.begin(), args.end()), parentScope),
				m_args(args),
				m_effect(effect)
{
}

std::shared_ptr<Effect> UniversalEffect::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	throw NotImplementedException(FILE_AND_LINE);
}

const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& UniversalEffect::getArgs() const
{
	return m_args;
}

const std::shared_ptr<Effect>& UniversalEffect::getEffect() const
{
	return m_effect;
}

void UniversalEffect::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher> res2;
	for (auto& p : res)
	{
		if (CONTAINS_NOT(p, m_args))
		{
			res2.insert(p);
		}
	}
	res = res2;
}

void UniversalEffect::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	children.push_back(m_effect);
}

std::shared_ptr<BaseElement> UniversalEffect::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

}

