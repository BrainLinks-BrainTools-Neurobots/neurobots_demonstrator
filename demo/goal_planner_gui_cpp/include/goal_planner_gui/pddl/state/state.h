/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: state.h
 */

#ifndef H8F2AC77C_17E7_4E69_B7E6_A4D66A8E7A8D
#define H8F2AC77C_17E7_4E69_B7E6_A4D66A8E7A8D
#include <goal_planner_gui/pddl/state/state_variable.h>
#include <memory>
#include <unordered_set>

namespace pddl
{

class Problem;
class Fact;
class InitLiteral;
class Term;
class Effect;
class BaseElement;
class Action;
class Condition;

/**
 * This class represents a complete planning state. It is a
 dictionary of StateVariable to TypedObjects.

 It has additional features to compute the truth value of
 conditions and the results of effect statements, as well as the
 evaluation of axioms.
 */
class State: public std::enable_shared_from_this<State>
{
public:
	typedef std::unordered_map<std::shared_ptr<StateVariable>, std::shared_ptr<TypedObject>, StateVariableHasher> StateMap;

	State(const std::vector<std::shared_ptr<Fact>>& facts = std::vector<std::shared_ptr<Fact>>(),
			std::shared_ptr<Problem> prop = std::shared_ptr<Problem>());
	virtual ~State();

	void applyEffect(std::shared_ptr<BaseElement> effect,
			bool traceVars = false);

	/**
	 * Return a Fact that describes the effect of applying a
	 Literal to this state. Supported are adding/deleting atoms,
	 fluent assignments and numeric operations. All Parameters to
	 the literal (and nested terms) must be instantiated, otherwise
	 an Exception is raised.

	 Arguments:
	 literal -- Literal object to apply.
	 trace_vars -- if True, all StateVariables required to resolve
	 the literal's arguments are written to the read_svars member
	 variable.
	 */
	std::shared_ptr<Fact> getLiteralEffect(std::shared_ptr<InitLiteral> literal,
			bool traceVars = false);

	/**
	 * Return list of Facts that describe the effect of applying
	 an Effect to this state. Supported are adding/deleting atoms,
	 fluent assignments and numeric operations. All Parameters to
	 the Effect (and nested elements) must be instantiated,
	 otherwise an Exception is raised.

	 Arguments:
	 effect -- Effect object to apply.
	 trace_vars -- if True, all StateVariables required to resolve
	 the effects's arguments are written to the read_svars member
	 variable.
	 */
	void getEffectFacts(std::shared_ptr<BaseElement> effect,
			std::unordered_map<std::shared_ptr<StateVariable>, std::shared_ptr<TypedObject>, StateVariableHasher>& facts,
			bool traceVars = false,
			std::function<bool(std::shared_ptr<BaseElement>&)> filterFn = [](std::shared_ptr<BaseElement>& eff)
			{	return true;});

	/**
	 * Create a StateVariable from a (potentially nested)
	 FunctionTerm. All Parameters in the term (and nested terms)
	 must be instantiated, otherwise an Exception is raised.

	 Arguments:
	 term -- Term object to evaluate
	 trace_vars -- if True, all StateVariables required to resolve
	 the term are written to the read_svars member variable.
	 */
	std::shared_ptr<StateVariable> svarFromTerm(std::shared_ptr<Term> term,
			bool traceVars = false);

	/**
	 * Evaluate a Term and return its value in the current
	 state. All Parameters in the term (and nested terms) must be
	 instantiated, otherwise an Exception is raised.

	 Arguments:
	 term -- Term object to evaluate
	 trace_vars -- if True, all StateVariables required to resolve
	 the term are written to the read_svars member variable.
	 */
	std::shared_ptr<TypedObject> evaluateTerm(std::shared_ptr<Term> term,
			bool traceVars = false);
	bool evaluateLiteral(const std::shared_ptr<Literal>& literal,
			bool traceVars = false);

	static std::shared_ptr<State> fromProblem(std::shared_ptr<Problem> problem);

	bool contains(const std::shared_ptr<Fact>& key);
	bool contains(const std::shared_ptr<StateVariable>& key);
	const std::shared_ptr<TypedObject>& get(const std::shared_ptr<StateVariable>& key);
	void set(const std::shared_ptr<StateVariable>& key, const std::shared_ptr<TypedObject>& value);

	StateMap::iterator begin();
	StateMap::const_iterator begin() const;
	StateMap::iterator end();
	StateMap::const_iterator end() const;

	size_t size() const;

	virtual std::string str() const;

	std::vector<std::shared_ptr<Fact>> iterfacts();
	const std::shared_ptr<Problem>& getProblem() const;

	bool isExecutable(const std::shared_ptr<Action>& action);
	bool isSatisfied(const std::shared_ptr<Condition>& cond);

	std::shared_ptr<State> copy();

private:
	std::vector<std::shared_ptr<Fact>> m_facts;
	std::shared_ptr<Problem> m_problem;
	std::unordered_set<std::shared_ptr<StateVariable>, StateVariableHasher> m_writtenSvars;
	std::unordered_set<std::shared_ptr<StateVariable>, StateVariableHasher> m_readSvars;
	StateMap m_dict;
};

POINTER_DEF(State);

} /* namespace pddl */

#endif /* H8F2AC77C_17E7_4E69_B7E6_A4D66A8E7A8D */
