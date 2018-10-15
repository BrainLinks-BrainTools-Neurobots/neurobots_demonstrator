/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: facts.cpp
 */

#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <goal_planner_gui/pddl/state/helpers.h>
#include <goal_planner_gui/pddl/state/state.h>
#include <goal_planner_gui/pddl/state/state_variable.h>
#include <goal_planner_gui/pddl/terms.h>

namespace pddl
{

Fact::Fact(std::shared_ptr<StateVariable> svar,
		std::shared_ptr<TypedObject> value) :
				m_svar(svar),
				m_value(value)
{
}

Fact::~Fact()
{
}

bool Fact::negated() const
{
	return false;
}

std::string Fact::str() const
{
	return m_svar->str() + " = " + m_value->str();
}

bool Fact::operator ==(const Fact& rhs) const
		{
	return hash() == rhs.hash();
}

bool Fact::operator !=(const Fact& rhs) const
		{
	return !(*this == rhs);
}

size_t Fact::hash() const
{
	size_t h = 0;
	pddl::hash_combine(h, m_svar->hash());
	if (m_value)
	{
		pddl::hash_combine(h, m_value->hash(true));
	}
	else
	{
		pddl::hash_combine(h, 0);
	}
	return h;
}

const std::shared_ptr<StateVariable>& Fact::getSvar() const
{
	return m_svar;
}

const std::shared_ptr<TypedObject>& Fact::getValue() const
{
	return m_value;
}

const std::vector<std::shared_ptr<TypedObject>>& Fact::allArgs() const
{
	if (m_allArgs.empty())
	{
		m_allArgs = m_svar->allArgs();
		m_allArgs.push_back(m_value);
	}
	return m_allArgs;
}

NegatedFact::NegatedFact(std::shared_ptr<StateVariable> svar,
		std::shared_ptr<TypedObject> value) :
				Fact(svar, value)
{
}

bool NegatedFact::negated() const
{
	return true;
}

std::vector<std::shared_ptr<Fact> > Fact::fromState(const std::shared_ptr<State>& state)
{
	std::vector<std::shared_ptr<Fact> > facts;
	facts.reserve(state->size());
	for (auto& it : *state)
	{
		facts.push_back(std::shared_ptr<Fact>(new Fact(it.first, it.second)));
	}
	return facts;
}

std::shared_ptr<Condition> Fact::toCondition() const
{
	dynamicSharedPointerCast(m_svar->getFunction(), p, Predicate);
	std::shared_ptr<LiteralCondition> l;
	if (p || m_svar->getModality())
	{
		l = m_svar->asLiteral<LiteralCondition>();
		if (m_value == DefaultTypedObjects::falseObject())
		{
			l = std::static_pointer_cast<LiteralCondition>(l->negated());
		}
	}
	else
	{
		TermVector args;
		for (auto& a : m_svar->getArgs())
		{
			args.push_back(pddl::TermPtr(new ConstantTerm(a)));
		}
		if (m_value->isInstanceOf(DefaultTypes::numberType()))
		{
			l.reset(new LiteralCondition(Builtin::eq(), {
					std::shared_ptr<Term>(new FunctionTerm(m_svar->getFunction(), args)),
					std::shared_ptr<Term>(new ConstantTerm(m_value))
			}));
		}
		else
		{
			l.reset(new LiteralCondition(Builtin::equals(), {
					std::shared_ptr<Term>(new FunctionTerm(m_svar->getFunction(), args)),
					std::shared_ptr<Term>(new ConstantTerm(m_value))
			}));
		}
	}

	return l;
}

std::shared_ptr<Fact> Fact::fromLiteral(const std::shared_ptr<Literal>& literal,
		const std::shared_ptr<State>& state)
{
	pddl::TypedObjectPtr value;
	if (CONTAINS(literal->m_predicate, pddl::Builtin::assignmentOps()) || literal->m_predicate == pddl::Builtin::eq()
			|| literal->m_predicate == pddl::Builtin::equals())
	{
		pddl::TypedObjectVector result;
		instantiateArgs( { literal->m_args.back() }, result, state);
		value = result[0];
		if (literal->m_negated)
		{
			return std::shared_ptr<Fact>(new NegatedFact(StateVariable::fromLiteral(literal, state), value));
		}
	}
	else
	{
		if (literal->m_negated)
		{
			value = DefaultTypedObjects::falseObject();
		}
		else
		{
			value = DefaultTypedObjects::trueObject();
		}
	}

	return std::shared_ptr<Fact>(new Fact(StateVariable::fromLiteral(literal, state), value));
}

std::shared_ptr<InitLiteral> Fact::toInit() const
{
	return asLiteral<InitLiteral>(true);
}

}
/* namespace pddl */

