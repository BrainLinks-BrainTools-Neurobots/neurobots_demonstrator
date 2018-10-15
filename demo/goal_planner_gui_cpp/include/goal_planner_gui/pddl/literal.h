/*
 * literal.h
 *
 *  Created on: Oct 3, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_LITERAL_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_LITERAL_H_
#include <goal_planner_gui/pddl/constants.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <memory>
#include <vector>

namespace pddl
{

class Predicate;
class Term;
class Scope;
class Element;
class Function;

/**
 * This class represents a PDDL literal.

 The arguments of the literal can be functions, variables and
 constants and are stored as Term objects in the "args" member
 variable.
 */
class Literal
{
protected:
	Literal(const Literal& other);

public:
	Literal(std::shared_ptr<Predicate> predicate,
			const std::vector<std::shared_ptr<Term>>& args,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>(),
			bool negated = false);
	virtual ~Literal();

	virtual std::string str() const;
	virtual size_t hash() const;
	virtual size_t hashWithInstance() const;
	virtual bool operator==(const Literal& other) const;
	virtual bool operator!=(const Literal& other) const;

	static std::shared_ptr<Literal> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope,
			bool negate = false,
			int maxNesting = 999,
			int functionScope = SCOPE_ALL);

	static bool isFunctional(const std::shared_ptr<Literal>& lit);
	static std::shared_ptr<Function> getFunction(const std::shared_ptr<Literal>& lit);

	virtual void setScope(const std::shared_ptr<Scope>& scope);

	void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	static void getLiteralElements(const std::shared_ptr<Literal>& lit,
			std::shared_ptr<Function>& function,
			std::vector<std::shared_ptr<Term>>& litargs,
			std::shared_ptr<Function>& modality,
			std::vector<std::shared_ptr<Term>>& modalArgs,
			std::shared_ptr<Term>& value,
			bool& negated);

	template<class T = Literal>
	std::shared_ptr<T> negate() const
	{
		return std::shared_ptr<T>(new T(m_predicate, m_args, std::shared_ptr<Scope>(), !m_negated));
	}

	std::shared_ptr<Predicate> m_predicate;
	std::vector<std::shared_ptr<Term>> m_args;
	bool m_negated;

protected:
	std::shared_ptr<Scope> m_scope;
};

HASH_AND_COMPARISON_OPERATOR(Literal)
STREAM_OPERATOR(Literal)

} /* namespace pddl */

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_LITERAL_H_ */
