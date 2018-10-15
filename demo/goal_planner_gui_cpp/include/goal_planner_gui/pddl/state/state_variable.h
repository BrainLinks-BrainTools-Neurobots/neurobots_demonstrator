/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: state_variable.h
 */

#ifndef H54F50E32_7C8C_43C2_A4EE_4C5D9EA391F8
#define H54F50E32_7C8C_43C2_A4EE_4C5D9EA391F8

#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/utils.h>

namespace pddl
{

class Function;
class TypedObject;
class Predicate;
class Literal;
class Term;
class State;

class StateVariable
{
public:
	StateVariable(std::shared_ptr<Function> function,
			std::vector<std::shared_ptr<TypedObject>> args,
			std::shared_ptr<Predicate> modality = std::shared_ptr<Predicate>(),
			std::vector<std::shared_ptr<TypedObject>> modalArgs = std::vector<std::shared_ptr<TypedObject>>());
	virtual ~StateVariable();

	bool operator==(const StateVariable& rhs) const;
	bool operator!=(const StateVariable& rhs) const;

	/**
	 * Return a copy of this StateVariable with a modified modality.

	 Arguments:
	 modality -- The Predicate defining the new modality
	 modal_args -- List of TypedObjects depending on the modality
	 */
	std::shared_ptr<StateVariable> asModality(std::shared_ptr<Predicate> modality,
			const std::vector<std::shared_ptr<TypedObject>>& modal_args = std::vector<std::shared_ptr<TypedObject>>());

	virtual size_t hash() const;
	virtual std::string str() const;
	const std::vector<std::shared_ptr<TypedObject> >& getArgs() const;
	const std::shared_ptr<Function>& getFunction() const;
	const std::vector<std::shared_ptr<TypedObject> >& getModalArgs() const;
	const std::shared_ptr<Predicate>& getModality() const;

	const std::vector<std::shared_ptr<TypedObject>>& allArgs() const;
	static void svarArgsFromLiteral(const std::shared_ptr<Literal>& literal,
			std::shared_ptr<Function>& function,
			std::vector<std::shared_ptr<Term>>& litargs,
			std::shared_ptr<Predicate>& modality,
			std::vector<std::shared_ptr<Term>>& modalargs,
			std::shared_ptr<Term>& value);
	static std::shared_ptr<StateVariable> fromLiteral(const std::shared_ptr<Literal>& literal,
			const std::shared_ptr<State>& state = std::shared_ptr<State>());

	template<class T = Literal>
	std::shared_ptr<T> asLiteral() const
	{
		std::shared_ptr<T> lit;
		std::vector<std::shared_ptr<Term>> constArgs;
		for (auto& a : m_args)
		{
			constArgs.push_back(std::shared_ptr<Term>(new ConstantTerm(a)));
		}

		if (!m_modality)
		{
			ASSERT(isInstance(m_function, Predicate));
			lit.reset(new T(std::static_pointer_cast<Predicate>(m_function), constArgs));
		}
		else
		{
			std::shared_ptr<Term> fterm(new FunctionTerm(m_function, constArgs));
			auto it = m_modalArgs.begin();
			std::vector<std::shared_ptr<Term>> args;
			for (auto& parg : m_modality->getArgs())
			{
				if (isInstance(parg->getType(), FunctionType))
				{
					args.push_back(fterm);
				}
				else
				{
					args.push_back(std::shared_ptr<Term>(new ConstantTerm(*it)));
					++it;
				}
			}
			lit.reset(new T(m_modality, args));
		}

		return lit;
	}

protected:
	std::shared_ptr<Function> m_function;
	std::vector<std::shared_ptr<TypedObject> > m_args;
	std::shared_ptr<Predicate> m_modality;
	std::vector<std::shared_ptr<TypedObject> > m_modalArgs;

	mutable std::size_t m_hash;
	mutable std::vector<std::shared_ptr<TypedObject>> m_allArgs;
};

HASH_AND_COMPARISON_OPERATOR(StateVariable);

} /* namespace pddl */

#endif /* H54F50E32_7C8C_43C2_A4EE_4C5D9EA391F8 */
