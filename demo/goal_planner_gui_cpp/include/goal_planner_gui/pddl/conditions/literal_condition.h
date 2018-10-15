/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: literal_condition.h
 */

#ifndef H772AF250_7425_4E23_A281_FA651666A8BC
#define H772AF250_7425_4E23_A281_FA651666A8BC

#include <goal_planner_gui/pddl/conditions/condition.h>
#include <goal_planner_gui/pddl/literal.h>

namespace pddl
{

class Predicate;
class Term;

class LiteralCondition: public Literal, public Condition
{
protected:
	LiteralCondition(const LiteralCondition& other);

public:
	LiteralCondition(const std::shared_ptr<Predicate> predicate,
			const std::vector<std::shared_ptr<Term>>& args,
			const std::shared_ptr<Scope>& scope = std::shared_ptr<Scope>(),
			bool negated = false);
	virtual ~LiteralCondition();

	virtual std::shared_ptr<Condition> negated();

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	virtual void setScope(const std::shared_ptr<Scope>& scope);

	virtual void collectConditions(std::vector<std::shared_ptr<LiteralCondition>>& res);

	void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Condition");
		Condition::getParents(parents);
	}

	virtual size_t hash() const;
	virtual size_t hashWithInstance() const;
	virtual bool operator==(const BaseElement& other) const;
	virtual bool operator==(const Literal& other) const;

	virtual std::shared_ptr<BaseElement> copy();

	virtual std::string str() const;
};

} /* namespace pddl */

#endif /* H772AF250_7425_4E23_A281_FA651666A8BC */
