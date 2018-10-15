/*
 * actions.h
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_ACTIONS_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_ACTIONS_H_
#include <goal_planner_gui/pddl/scope.h>

namespace pddl
{

class Parameter;
class Condition;
class Effect;

class Action: public Scope
{
public:
	Action(const std::string& name,
			const std::vector<std::shared_ptr<Parameter>>& args,
			std::shared_ptr<Condition> precondition,
			std::shared_ptr<Effect> effect,
			std::shared_ptr<Scope> domain);
	virtual ~Action();

	virtual bool operator==(const Action& rhs) const;
	virtual bool operator!=(const Action& rhs) const;

	virtual size_t hash() const;
	virtual std::string str() const;

	double getTotalCost();
	void getEffects(const std::shared_ptr<Term>& term,
			std::unordered_set<std::shared_ptr<Term>, pddl::TermHasher>& result);

	static std::shared_ptr<Action> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	std::string m_name;
	std::vector<std::shared_ptr<Parameter>> m_args;
	std::shared_ptr<Condition> m_precondition;
	std::shared_ptr<Effect> m_effect;
	mutable size_t m_hash;
};

HASH_AND_COMPARISON_OPERATOR(Action);
STREAM_OPERATOR(Action)

} /* namespace pddl */

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_ACTIONS_H_ */
