/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: quantified_condition.h
 */

#ifndef H7B3CFF80_806E_440A_9468_36CF16C80C68
#define H7B3CFF80_806E_440A_9468_36CF16C80C68

#include <goal_planner_gui/pddl/conditions/condition.h>

namespace pddl
{

class QuantifiedCondition: public Condition
{
public:
	QuantifiedCondition();
	virtual ~QuantifiedCondition();

	virtual std::shared_ptr<Condition> negated() = 0;

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Condition");
		Condition::getParents(parents);
	}

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	virtual std::string str() const;

	virtual std::shared_ptr<BaseElement> copy();
};

} /* namespace pddl */

#endif /* H7B3CFF80_806E_440A_9468_36CF16C80C68 */
