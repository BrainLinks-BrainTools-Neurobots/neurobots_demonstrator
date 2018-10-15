/*
 * problem.h
 *
 *  Created on: Oct 2, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_PROBLEM_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_PROBLEM_H_
#include <goal_planner_gui/pddl/base_element.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/utils.h>
#include <string>
#include <unordered_set>

namespace pddl
{

class InitLiteral: public BaseElement, public Literal
{
public:
	InitLiteral(std::shared_ptr<Predicate> predicate,
			const std::vector<std::shared_ptr<Term>>& args,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>(),
			bool negated = false);

	virtual std::string str() const;
	virtual size_t hash() const;
	virtual bool operator==(const InitLiteral& other) const;
	virtual bool operator!=(const InitLiteral& other) const;

	static std::shared_ptr<InitLiteral> parse(std::shared_ptr<Element> elem,
			std::shared_ptr<Scope> scope);

	virtual void setScope(const std::shared_ptr<Scope>& scope);

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	virtual std::shared_ptr<BaseElement> copy();
};

HASH_AND_COMPARISON_OPERATOR(InitLiteral);

class Domain;
class State;

class Problem: public Scope
{
public:
	/**
	 * Used in neurobots, init is filled later
	 */
	Problem(const std::string& name,
			const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& objects,
			const std::shared_ptr<Domain> domain);
	Problem(const std::string& name,
			const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& objects,
			const std::shared_ptr<Domain> domain,
			const std::unordered_set<std::shared_ptr<pddl::InitLiteral>, InitLiteralHasher>& init);
	virtual ~Problem();

	std::shared_ptr<Domain> getDomain();

	virtual void getAllObjects(const std::shared_ptr<Type> type,
			std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& result);

	std::shared_ptr<Problem> copy();

	std::string m_name;
	std::unordered_set<std::shared_ptr<pddl::InitLiteral>, InitLiteralHasher> m_init;
	std::unordered_set<std::shared_ptr<pddl::TypedObject>, TypedObjectHasher> m_objects;
	std::unordered_map<std::shared_ptr<Type>, std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>, TypeHasher> m_objectsByType;
};

POINTER_DEF(Problem);

} /* namespace pddl */

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_PROBLEM_H_ */
