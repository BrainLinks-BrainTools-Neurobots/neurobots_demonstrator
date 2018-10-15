/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: conjunctive_effect.cpp
 */
#include <goal_planner_gui/pddl/base_element.h>
#include <goal_planner_gui/pddl/effects/conjunctive_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/parser.h>

namespace pddl
{
ConjunctiveEffect::ConjunctiveEffect(const std::vector<std::shared_ptr<Effect>>& effects,
		const std::shared_ptr<Scope>& scope) :
				m_effects(effects)
{
	m_scope = scope;
}

void ConjunctiveEffect::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	for (auto& it : m_effects)
		children.push_back(it);
}

std::shared_ptr<Effect> ConjunctiveEffect::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	it->get("and");

	EffectVector parts;
	for (auto it2 = it->current(); it2 != it->end(); ++it2)
	{
		auto& part = *it2;
		if (part->isTerminal())
			throw UnexpectedTokenError(part->token, "effects");
		parts.push_back(Effect::parse(part, scope));
	}

	return std::shared_ptr<Effect>(new ConjunctiveEffect(parts));
}

std::shared_ptr<BaseElement> ConjunctiveEffect::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

const std::vector<std::shared_ptr<Effect> >& ConjunctiveEffect::getEffects() const
{
	return m_effects;
}

}
