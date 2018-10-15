/*
 * effects.cpp
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/modules.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>

namespace pddl
{

SimpleEffect::SimpleEffect(std::shared_ptr<Predicate> predicate,
		const std::vector<std::shared_ptr<Term> >& args,
		std::shared_ptr<Scope> scope,
		bool negated) :
				Literal(predicate, args, scope, negated)
{
	BaseElement::m_scope = scope;
}

SimpleEffect::~SimpleEffect()
{
}

size_t SimpleEffect::hash() const
{
	return Literal::hash();
}

bool SimpleEffect::operator ==(const Effect& rhs) const
		{
	return hash() == rhs.hash();
}

bool SimpleEffect::operator ==(const Literal& rhs) const
		{
	return hash() == rhs.hash();
}

std::shared_ptr<Effect> SimpleEffect::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	auto first = it->get(Element::ExpectedTypeNone, "effect specification")->token;
	it->reset();
	auto literal = Literal::parse(it, scope, false, 999, SCOPE_EFFECT);

	if ((CONTAINS(literal->m_predicate, Builtin::assignmentOps()) || CONTAINS(literal->m_predicate, Builtin::numericOps())) && literal->m_negated)
	{
		throw ParseError(first, "Can't negate fluent assignment");
	}

	if (literal->m_predicate == Builtin::equals())
	{
		throw ParseError(first, "Can't use '=' in effects, please use 'assign' instead.");
	}

	return std::shared_ptr<Effect>(new SimpleEffect(literal->m_predicate, literal->m_args, scope, literal->m_negated));
}

std::string SimpleEffect::str() const
{
	return "SimpleEffect: " + Literal::str();
}

void SimpleEffect::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	Literal::collectFreeVars(res);
}

void SimpleEffect::collectEffects(std::vector<std::shared_ptr<SimpleEffect> >& res)
{
	res.push_back(std::static_pointer_cast<SimpleEffect>(shared_from_this()));
}

void SimpleEffect::setScope(const std::shared_ptr<Scope>& scope)
{
	BaseElement::setScope(scope);
	Literal::setScope(scope);
}

std::shared_ptr<BaseElement> SimpleEffect::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

} /* namespace pddl */

