/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: state.cpp
 */

#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/base_element.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/conditions/intermediate_condition.h>
#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/conditions/preference_condition.h>
#include <goal_planner_gui/pddl/conditions/quantified_condition.h>
#include <goal_planner_gui/pddl/effects/conditional_effect.h>
#include <goal_planner_gui/pddl/effects/conjunctive_effect.h>
#include <goal_planner_gui/pddl/effects/universal_effect.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <goal_planner_gui/pddl/state/state.h>
#include <memory>

namespace pddl
{

State::State(const std::vector<std::shared_ptr<Fact> >& facts,
		std::shared_ptr<Problem> prop) :
				m_facts(facts),
				m_problem(prop)
{
}

State::~State()
{
}

void State::applyEffect(std::shared_ptr<BaseElement> effect,
		bool traceVars)
{
	std::unordered_map<std::shared_ptr<StateVariable>, std::shared_ptr<TypedObject>, StateVariableHasher> facts;
	getEffectFacts(effect, facts, traceVars);
	for (auto& it : facts)
	{
		auto& svar = it.first;
		auto& val = it.second;
		if (traceVars)
			m_writtenSvars.insert(svar);
//		LOG_INFO("Apply Effect: " << svar->str() << " --- " << val->getName());
		m_dict[svar] = val;
	}
}

template<typename T>
inline bool any(std::vector<T>& list,
		std::function<bool(T)> condition)
{
	for (auto& t : list)
		if (condition(t))
			return true;
	return false;
}

void State::getEffectFacts(std::shared_ptr<BaseElement> effect,
		std::unordered_map<std::shared_ptr<StateVariable>, std::shared_ptr<TypedObject>, StateVariableHasher>& facts,
		bool traceVars,
		std::function<bool(std::shared_ptr<BaseElement>&)> filterFn)
{
	if (!filterFn(effect))
	{
		facts.clear();
		return;
	}

	if (isInstanceSharedCast(effect, unEff, UniversalEffect))
	{
		std::vector<std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>> allObjects;
		for (auto& a: unEff->getArgs())
		{
			std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> res;
			m_problem->getAllObjects(a->getType(), res);
			allObjects.push_back(res);
		}
		throw NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(effect, condEff, ConditionalEffect))
	{
		throw NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(effect, conjEff, ConjunctiveEffect))
	{
		for (auto& eff: conjEff->getEffects())
		{
			getEffectFacts(eff, facts, traceVars);
		}
	}
	else if (isInstanceSharedCast(effect, literal, InitLiteral))
	{
		auto fact = getLiteralEffect(literal, traceVars);
		facts[fact->getSvar()] = fact->getValue();
	}
}

std::shared_ptr<Fact> State::getLiteralEffect(std::shared_ptr<InitLiteral> literal,
		bool traceVars)
{
	std::shared_ptr<StateVariable> effSvar;
	std::shared_ptr<TypedObject> effValue;

	auto& pArgs = literal->m_predicate->getArgs();
	if (literal->m_args.size() != pArgs.size())
		throw Exception("args.size() != pargs.size()");

	std::vector<std::shared_ptr<StateVariable>> svars;
	std::vector<std::shared_ptr<TypedObject>> values;

	for (size_t i = 0; i < pArgs.size(); ++i)
	{
		auto& arg = literal->m_args[i];
		auto& parg = pArgs[i];
		if (parg->getType()->t() == Types::FunctionType)
			svars.push_back(svarFromTerm(arg, traceVars));
		else
			values.push_back(evaluateTerm(arg, traceVars));
	}

	auto& pred = literal->m_predicate;
	std::shared_ptr<TypedObject> previous = DefaultTypedObjects::unknownObject();
	std::shared_ptr<TypedObject> val;

	std::shared_ptr<StateVariable> svar;
	if (CONTAINS(pred, Builtin::assignmentOps()) || CONTAINS(pred, Builtin::numericOps()))
	{
		svar = svars[0];

		//hack
		if (svar->getFunction() == Builtin::totalCost() && CONTAINS_NOT(svar, m_dict))
			m_dict[svar] = std::shared_ptr<TypedNumber>(new TypedNumber(0.0));

		previous = m_dict[svar];
		val = values[0];
		effSvar = svar;
	}

	static std::unordered_set<std::shared_ptr<Predicate>> numPreds { Builtin::scaleUp(), Builtin::scaleDown(), Builtin::increase(),
			Builtin::decrease() };

//	LOG_INFO(pred->str());
//	for (auto& a : Builtin::assignmentOps())
//	{
//		LOG_INFO(a->str());
//	}
//	for (auto& a : numPreds)
//	{
//		LOG_INFO(a->str());
//	}

	if (CONTAINS(pred, Builtin::assignmentOps()))
	{
		effValue = val;
	}
	else if (CONTAINS(pred, numPreds))
	{
		const auto& numPrev = std::static_pointer_cast<TypedNumber>(previous);
		const auto& numVal = std::static_pointer_cast<TypedNumber>(val);
		if (previous == DefaultTypedObjects::unknownObject())
			effValue = DefaultTypedObjects::unknownObject();
		else if (pred == Builtin::increase())
			effValue.reset(new TypedNumber(numPrev->getNumber() + numVal->getNumber()));
		else if (pred == Builtin::decrease())
			effValue.reset(new TypedNumber(numPrev->getNumber() - numVal->getNumber()));
		else if (pred == Builtin::scaleUp())
			effValue.reset(new TypedNumber(numPrev->getNumber() * numVal->getNumber()));
		else if (pred == Builtin::scaleDown())
			effValue.reset(new TypedNumber(numPrev->getNumber() / numVal->getNumber()));
	}
	else
	{
		if (svars.empty())
			svar.reset(new StateVariable(pred, values));
		else if (svars.size() == 1)
			svar = svars[0]->asModality(pred, values);
		else
			throw Exception("A modal svar can only contain one function");

		effSvar = svar;
		if (literal->m_negated)
			effValue = DefaultTypedObjects::falseObject();
		else
			effValue = DefaultTypedObjects::trueObject();
	}
//
//	LOG_INFO(effSvar->getArgs().size());
//	for (auto& it : effSvar->getArgs())
//	{
//		LOG_INFO(it);
//	}

	return std::shared_ptr<Fact>(new Fact(effSvar, effValue));
}

std::shared_ptr<StateVariable> State::svarFromTerm(std::shared_ptr<Term> term,
		bool traceVars)
{
	if (term->t() != TermTypes::FunctionTerm)
		throw Exception(term->str() + " is not a function term");

	const auto& fterm = std::static_pointer_cast<FunctionTerm>(term);
	if (CONTAINS(fterm->getFunction(), Builtin::numericFunctions()))
	{
		throw Exception("can't create state variable form built-in function " + fterm->getFunction()->getName());
	}

	std::vector<std::shared_ptr<TypedObject>> values;
	for (auto& arg : fterm->getArgs())
		values.push_back(evaluateTerm(arg, traceVars));

	return std::shared_ptr<StateVariable>(new StateVariable(fterm->getFunction(), values));
}

std::shared_ptr<TypedObject> State::evaluateTerm(std::shared_ptr<Term> term,
		bool traceVars)
{
	if (term->t() == TermTypes::ConstantTerm)
	{
		return std::static_pointer_cast<ConstantTerm>(term)->getObj();
	}

	else if (term->t() == TermTypes::VariableTerm)
	{
		auto t = std::static_pointer_cast<VariableTerm>(term);
		ASSERT(t->isInstantiated());
		return t->getInstance();
	}

	else if (term->t() == TermTypes::FunctionVariableTerm)
	{
		throw NotImplementedException(FILE_AND_LINE);
	}

	if (term->t() != TermTypes::FunctionTerm)
	{
		throw Exception("You need to provide a function term in the following (check py)!");
	}

	std::vector<std::shared_ptr<TypedObject>> values;
	const auto& fterm = std::static_pointer_cast<FunctionTerm>(term);
	for (auto& arg : fterm->getArgs())
		values.push_back(evaluateTerm(arg, traceVars));

	const auto& f = fterm->getFunction();

	auto compare = [](std::shared_ptr<TypedObject> object) -> bool
	{	return object == DefaultTypedObjects::unknownObject();};

	if (any<std::shared_ptr<TypedObject>>(values, compare))
		return DefaultTypedObjects::unknownObject();

	if (CONTAINS(f, Builtin::numericFunctions()))
	{
		double val1 = std::static_pointer_cast<TypedNumber>(values[0])->getNumber();
		double val2 = std::static_pointer_cast<TypedNumber>(values[1])->getNumber();
		if (f == Builtin::plus())
			return std::shared_ptr<TypedNumber>(new TypedNumber(val1 + val2));
		else if (f == Builtin::minus())
			return std::shared_ptr<TypedNumber>(new TypedNumber(val1 - val2));
		else if (f == Builtin::mult())
			return std::shared_ptr<TypedNumber>(new TypedNumber(val1 * val2));
		else if (f == Builtin::div())
			return std::shared_ptr<TypedNumber>(new TypedNumber(val1 / val2));
		else if (f == Builtin::neg())
			return std::shared_ptr<TypedNumber>(new TypedNumber(-val1));
	}

	std::shared_ptr<StateVariable> svar(new StateVariable(f, values));
	if (traceVars)
		m_readSvars.insert(svar);

	return m_dict[svar];
}

std::shared_ptr<State> State::fromProblem(std::shared_ptr<Problem> problem)
{
	std::shared_ptr<State> state(new State( { }, problem));

	for (auto& i : problem->m_init)
		state->applyEffect(i);

	return state;
}

bool State::contains(const std::shared_ptr<Fact>& key)
{
	auto& svar = key->getSvar();
	if (CONTAINS_NOT(svar, m_dict))
	{
		if (isInstanceSharedCast(svar->getFunction(), p, Predicate))
		{
			return (key->getValue() == DefaultTypedObjects::falseObject() ^ key->negated());
		}
		return (key->getValue() == DefaultTypedObjects::unknownObject() ^ key->negated());
	}
	return (m_dict[svar] == key->getValue()) ^ key->negated();
}

bool State::contains(const std::shared_ptr<StateVariable>& key)
{
	return m_dict.find(key) != m_dict.end();
}

State::StateMap::iterator State::begin()
{
	return m_dict.begin();
}

State::StateMap::const_iterator State::begin() const
{
	return m_dict.begin();
}

State::StateMap::iterator State::end()
{
	return m_dict.end();
}

State::StateMap::const_iterator State::end() const
{
	return m_dict.end();
}

std::string State::str() const
{
	std::string res;
	for (auto& it : *this)
	{
		res += it.first->str() + " = " + it.second->getName() + "\n";
	}
	return res;
}

const std::shared_ptr<TypedObject>& State::get(const std::shared_ptr<StateVariable>& key)
{
	return m_dict[key];
}

const std::shared_ptr<Problem>& State::getProblem() const
{
	return m_problem;
}

std::vector<std::shared_ptr<Fact>> State::iterfacts()
{
	return Fact::fromState(shared_from_this());
}

size_t State::size() const
{
	return m_dict.size();
}

bool State::isExecutable(const std::shared_ptr<Action>& action)
{
	//not required because we have no axioms
	//extstate = self.get_extended_state(self.get_relevant_vars(action.precondition))
	return isSatisfied(action->m_precondition);
}

bool State::evaluateLiteral(const std::shared_ptr<Literal>& literal,
		bool traceVars)
{
	ASSERT(literal->m_args.size() == literal->m_predicate->getArgs().size());

//	LOG_INFO(literal->str());

	std::vector<std::shared_ptr<StateVariable>> svars;
	std::vector<std::shared_ptr<TypedObject>> values;
	for (size_t i = 0; i < literal->m_args.size(); ++i)
	{
		auto& arg = literal->m_args[i];
		auto& parg = literal->m_predicate->getArgs()[i];

//		LOG_INFO(literal->m_predicate->str());

		if (isInstance(parg->getType(), FunctionType))
		{
			svars.push_back(svarFromTerm(arg, traceVars));
		}
		else
		{
//			LOG_INFO(parg->str())
//			LOG_INFO(arg->str() << " -> " << evaluateTerm(arg))
			values.push_back(evaluateTerm(arg, traceVars));
		}
	}

	bool found = false;
	for (auto& v : values)
	{
		if (v == DefaultTypedObjects::unknownObject())
		{
			found = true;
			break;
		}
	}

	auto& pred = literal->m_predicate;

	if (found)
	{
		if (pred == Builtin::equals() || pred == Builtin::eq())
		{
			return (values[0] == values[1]) ^ literal->m_negated;
		}
		return literal->m_negated;
	}

	if (pred == Builtin::equals() || pred == Builtin::eq())
	{
//		LOG_INFO(values[0]->getName());
//		LOG_INFO(values[1]->getName());
//		LOG_INFO(literal->m_negated)
		return (values[0]->getName() == values[1]->getName()) ^ literal->m_negated;
	}
	else if (pred == Builtin::gt())
	{
		return (values[0]->getName() > values[1]->getName()) ^ literal->m_negated;
	}
	else if (pred == Builtin::lt())
	{
		return (values[0]->getName() < values[1]->getName()) ^ literal->m_negated;
	}
	else if (pred == Builtin::ge())
	{
		return (values[0]->getName() >= values[1]->getName()) ^ literal->m_negated;
	}
	else if (pred == Builtin::le())
	{
		return (values[0]->getName() <= values[1]->getName()) ^ literal->m_negated;
	}
	else
	{
		StateVariablePtr svar;
		if (svars.empty())
		{
			svar.reset(new StateVariable(pred, values));
		}
		else if (svars.size() == 1)
		{
			svar = svars[0]->asModality(pred, values);
		}
		else
		{
			throw pddl::Exception("A modal svar can only contain one function");
		}

		if (traceVars)
		{
			m_readSvars.insert(svar);
		}

//		if self.auto_axiom_evaluation and self.problem and svar.function in self.problem.domain.get_derived():
//		                st = self.get_extended_state([svar])

		return get(svar) == DefaultTypedObjects::trueObject() ^ literal->m_negated;
	}
}

bool State::isSatisfied(const std::shared_ptr<Condition>& cond)
{
	if (isInstance(cond, PreferenceCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstance(cond, IntermediateCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(cond, c, LiteralCondition))
	{
		//TODO: missing parts
		return evaluateLiteral(c);
	}
	else if (isInstanceSharedCast(cond, c, Conjunction))
	{
		if (c->getParts().empty())
		{
			return true;
		}

		for (auto& it: c->getParts())
		{
			if (!isSatisfied(it))
			{
				return false;
			}
		}
		return true;
	}
	else if (isInstanceSharedCast(cond, c, Disjunction))
	{
		if (c->getParts().empty())
		{
			return true;
		}

		for (auto& it: c->getParts())
		{
			if (isSatisfied(it))
			{
				return true;
			}
		}
		return false;
	}
	else if (isInstance(cond, QuantifiedCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else
	{
		return true;
	}
}

void State::set(const std::shared_ptr<StateVariable>& key,
		const std::shared_ptr<TypedObject>& value)
{
	m_dict[key] = value;
}

std::shared_ptr<State> State::copy()
{
	std::shared_ptr<State> s(new State( { }, m_problem));
	for (auto& it : m_dict)
	{
		s->set(it.first, it.second);
	}
	return s;
}

}

/* namespace pddl */

