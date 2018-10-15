/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: state_variable.cpp
 */

#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/state/helpers.h>
#include <goal_planner_gui/pddl/state/state.h>
#include <goal_planner_gui/pddl/state/state_variable.h>
#include <goal_planner_gui/pddl/terms.h>
#include <memory>

namespace pddl
{

StateVariable::StateVariable(std::shared_ptr<Function> function,
		std::vector<std::shared_ptr<TypedObject> > args,
		std::shared_ptr<Predicate> modality,
		std::vector<std::shared_ptr<TypedObject> > modalArgs) :
				m_function(function),
				m_args(args),
				m_modality(modality),
				m_modalArgs(modalArgs),
				m_hash(0)
{
}

StateVariable::~StateVariable()
{
}

bool StateVariable::operator ==(const StateVariable& rhs) const
		{
	return hash() == rhs.hash();
}

bool StateVariable::operator !=(const StateVariable& rhs) const
		{
	return !(*this == rhs);
}

size_t StateVariable::hash() const
{
	if (m_hash == 0)
	{
		hash_combine(m_hash, m_function->hash(), m_modality ? m_modality->hash() : 0);
		for (auto& it : m_args)
			hash_combine(m_hash, it->hash(true));
		for (auto& it : m_modalArgs)
			hash_combine(m_hash, it->hash(true));
	}
	return m_hash;
}

std::string StateVariable::str() const
{
	std::string args;
	for (auto& it : m_args)
		args += it->str() + ", ";

	if (!args.empty())
	{
		args.pop_back();
		args.pop_back();
	}

	std::string s = m_function->getName() + "(" + args + ")";

	if (m_modality)
	{
		std::string margs;
		for (auto& it : m_modalArgs)
			args += it->str() + ", ";
		s = m_modality->getName() + "(" + s + ", " + margs + ")";
	}

	return s;
}

const std::vector<std::shared_ptr<TypedObject> >& StateVariable::getArgs() const
{
	return m_args;
}

const std::shared_ptr<Function>& StateVariable::getFunction() const
{
	return m_function;
}

const std::vector<std::shared_ptr<TypedObject> >& StateVariable::getModalArgs() const
{
	return m_modalArgs;
}

const std::shared_ptr<Predicate>& StateVariable::getModality() const
{
	return m_modality;
}

std::shared_ptr<StateVariable> StateVariable::asModality(std::shared_ptr<Predicate> modality,
		const std::vector<std::shared_ptr<TypedObject> >& modal_args)
{
//	std::vector<std::shared_ptr<Parameter>> functionArgs;
//	for (auto& a: modality->getArgs())
//		if (a->getType()->t() == Types::FunctionType)
//			functionArgs.push_back(a);

	return std::shared_ptr<StateVariable>(new StateVariable(m_function, m_args, modality, modal_args));
}

const std::vector<std::shared_ptr<TypedObject>>& StateVariable::allArgs() const
{
	if (m_allArgs.empty())
	{
		m_allArgs.insert(m_allArgs.end(), m_args.begin(), m_args.end());
		m_allArgs.insert(m_allArgs.end(), m_modalArgs.begin(), m_modalArgs.end());
	}
	return m_allArgs;
}

void StateVariable::svarArgsFromLiteral(const std::shared_ptr<Literal>& literal,
		std::shared_ptr<Function>& function,
		std::vector<std::shared_ptr<Term>>& litargs,
		std::shared_ptr<Predicate>& modality,
		std::vector<std::shared_ptr<Term>>& modalargs,
		std::shared_ptr<Term>& value)
{
	litargs.clear();
	modalargs.clear();
	static const auto aOps = Builtin::assignmentOps();
	static const auto nComp = Builtin::numericComparators();
	if (CONTAINS(literal->m_predicate, aOps) || CONTAINS(literal->m_predicate, nComp) || literal->m_predicate == Builtin::equals())
	{
		dynamicSharedPointerCast(literal->m_args[0], fterm, FunctionTerm);
		ASSERT(fterm);
		function = fterm->getFunction();
		litargs = fterm->getArgs();
		value = literal->m_args[1];
	}
	else
	{
		auto pargs = literal->m_predicate->getArgs();
		size_t s = std::min(literal->m_args.size(), pargs.size());
		for (size_t i = 0; i < s; ++i)
		{
			auto& arg = literal->m_args[i];
			auto& parg = pargs[i];

			if (isInstanceSharedCast(parg, ft, FunctionTerm))
			{
				if (!function)
				{
					function = ft->getFunction();
					litargs = ft->getArgs();
				}
				else
				{
					throw Exception("A modal svar can only contain one function");
				}
			}
			else
			{
				modalargs.push_back(arg);
			}
		}

		if (!function)
		{
			function = literal->m_predicate;
			litargs = literal->m_args;
			modalargs.clear();
		}
		else
		{
			modality = literal->m_predicate;
		}

		value.reset(new ConstantTerm(literal->m_negated ? DefaultTypedObjects::falseObject() : DefaultTypedObjects::trueObject()));
	}
}

std::shared_ptr<StateVariable> StateVariable::fromLiteral(const std::shared_ptr<Literal>& literal,
		const std::shared_ptr<State>& state)
{
	std::shared_ptr<Function> function;
	std::vector<std::shared_ptr<Term>> litargs;
	std::shared_ptr<Predicate> modality;
	std::vector<std::shared_ptr<Term>> modalargs;
	std::shared_ptr<Term> value;
	svarArgsFromLiteral(literal, function, litargs, modality, modalargs, value);

	TypedObjectVector args;
	instantiateArgs(litargs, args, state);

	if (modality)
	{
		TypedObjectVector modalArgsRes;
		instantiateArgs(modalargs, modalArgsRes, state);
		return std::shared_ptr<StateVariable>(new StateVariable(function, args, literal->m_predicate, modalArgsRes));
	}

	return std::shared_ptr<StateVariable>(new StateVariable(function, args));
}

} /* namespace pddl */

