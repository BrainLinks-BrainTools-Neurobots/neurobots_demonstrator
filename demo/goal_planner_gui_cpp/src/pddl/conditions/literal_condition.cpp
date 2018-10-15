/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: literal_condition.cpp
 */

#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/terms.h>

namespace pddl
{

LiteralCondition::LiteralCondition(const std::shared_ptr<Predicate> predicate,
		const std::vector<std::shared_ptr<Term>>& args,
		const std::shared_ptr<Scope>& scope,
		bool negated) :
				Literal(predicate, args, scope, negated)
{
	BaseElement::m_scope = scope;
}

LiteralCondition::LiteralCondition(const LiteralCondition& other) :
				Literal(other)
{
	//Literal::m_scope is set by copy constructor
	BaseElement::m_scope = Literal::m_scope;
}

LiteralCondition::~LiteralCondition()
{
}

std::shared_ptr<Condition> LiteralCondition::negated()
{
	return std::shared_ptr<Condition>(new LiteralCondition(m_predicate, m_args, std::shared_ptr<Scope>(), !m_negated));
}

std::shared_ptr<Condition> LiteralCondition::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	auto literal = Literal::parse(it, scope, false, 999, SCOPE_CONDITION);
	return std::shared_ptr<Condition>(new LiteralCondition(literal->m_predicate, literal->m_args, scope, literal->m_negated));
}

void LiteralCondition::collectConditions(std::vector<std::shared_ptr<LiteralCondition> >& res)
{
	Condition::collectConditions(res);
	res.push_back(std::static_pointer_cast<LiteralCondition>(shared_from_this()));
}

void LiteralCondition::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	Literal::collectFreeVars(res);
}

std::shared_ptr<BaseElement> LiteralCondition::copy()
{
	return std::shared_ptr<LiteralCondition>(new LiteralCondition(*this));
}

std::string LiteralCondition::str() const
{
	return "LiteralCondition: " + Literal::str();
}

void LiteralCondition::setScope(const std::shared_ptr<Scope>& scope)
{
	BaseElement::setScope(scope);
	Literal::setScope(scope);
}

size_t LiteralCondition::hash() const
{
	std::size_t h = 0;
	hash_combine(h, std::string("LiteralCondition"), Literal::hash());

	if (m_hash != 0 && m_hash != h)
	{
		LOG_ERROR("Hash has changed!");
	}

	m_hash = h;

	return h;
}

size_t LiteralCondition::hashWithInstance() const
{
	std::size_t h = 0;
	hash_combine(h, std::string("LiteralCondition"), Literal::hashWithInstance());
	return h;
}

bool LiteralCondition::operator ==(const BaseElement& other) const
		{
	if (!dynamic_cast<const LiteralCondition*>(&other))
	{
		return false;
	}
	return BaseElement::operator ==(other);
}

bool LiteralCondition::operator ==(const Literal& other) const
		{
	if (!dynamic_cast<const LiteralCondition*>(&other))
	{
		return false;
	}
	return Literal::operator ==(other);
}

} /* namespace pddl */

