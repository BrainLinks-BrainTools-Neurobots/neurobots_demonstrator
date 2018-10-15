/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: facts.h
 */

#ifndef HEAFB9C9B_146E_4624_A8B0_81466C3A6ABA
#define HEAFB9C9B_146E_4624_A8B0_81466C3A6ABA
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/state/state_variable.h>
#include <goal_planner_gui/pddl/utils.h>

namespace pddl
{

class StateVariable;
class TypedObject;
class State;
class Literal;
class Condition;
class InitLiteral;

class Fact
{
public:
	Fact(std::shared_ptr<StateVariable> svar,
			std::shared_ptr<TypedObject> value);
	virtual ~Fact();

	virtual bool negated() const;
	virtual std::string str() const;
	virtual bool operator==(const Fact& rhs) const;
	virtual bool operator!=(const Fact& rhs) const;

	virtual size_t hash() const;

	const std::shared_ptr<StateVariable>& getSvar() const;
	const std::shared_ptr<TypedObject>& getValue() const;

	const std::vector<std::shared_ptr<TypedObject>>& allArgs() const;

	std::shared_ptr<Condition> toCondition() const;
	std::shared_ptr<InitLiteral> toInit() const;

	static std::vector<std::shared_ptr<Fact>> fromState(const std::shared_ptr<State>& state);
	static std::shared_ptr<Fact> fromLiteral(const std::shared_ptr<Literal>& literal,
			const std::shared_ptr<State>& state = std::shared_ptr<State>());

	template<class T = Literal>
	std::shared_ptr<T> asLiteral(bool useEqual = false) const
			{
		std::shared_ptr<T> l;

		if (isInstance(m_svar->getFunction(), Predicate)||m_svar->getModality())
		{
			l = m_svar->asLiteral<T>();
			if (m_value == DefaultTypedObjects::falseObject())
			{
				return l->template negate<T>();
			}
			return l;
		}
		else
		{
			std::shared_ptr<Predicate> op;
			if (useEqual)
			{
				if (m_value->isInstanceOf(DefaultTypes::numberType()))
				{
					op = Builtin::numEqualAssign();
				}
				else
				{
					op = Builtin::equalAssign();
				}
			}
			else
			{
				if (m_value->isInstanceOf(DefaultTypes::numberType()))
				{
					op = Builtin::numAssign();
				}
				else
				{
					op = Builtin::assign();
				}
			}

			TermVector args;
			for (auto& a: m_svar->getArgs())
			{
				args.push_back(std::shared_ptr<Term>(new ConstantTerm(a)));
			}

			return std::shared_ptr<T>(new T(op,
					{
						std::shared_ptr<Term>(new FunctionTerm(m_svar->getFunction(), args)),
						std::shared_ptr<Term>(new ConstantTerm(m_value))
					}));
		}
	}

private:
	std::shared_ptr<StateVariable> m_svar;
	std::shared_ptr<TypedObject> m_value;
	mutable std::vector<std::shared_ptr<TypedObject>> m_allArgs;
};

class NegatedFact: public Fact
{
public:
	NegatedFact(std::shared_ptr<StateVariable> svar,
			std::shared_ptr<TypedObject> value);

	virtual bool negated() const;
};

HASH_AND_COMPARISON_OPERATOR(Fact);

//typedef std::unordered_set<std::shared_ptr<Fact>> FactUnorderedSet;
STREAM_OPERATOR(Fact);
//POINTER_DEF(Fact);
//VECTOR_DEF(Fact);

POINTER_DEF(NegatedFact);

} /* namespace pddl */

#endif /* HEAFB9C9B_146E_4624_A8B0_81466C3A6ABA */
