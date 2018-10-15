/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: intermediate_condition.h
 */

#ifndef HA9E72090_CA55_47AB_B142_B5C348ADE96C
#define HA9E72090_CA55_47AB_B142_B5C348ADE96C

#include <goal_planner_gui/pddl/conditions/condition.h>

namespace pddl
{

class IntermediateCondition: public Condition
{
public:
	IntermediateCondition();
	virtual ~IntermediateCondition();

	virtual std::shared_ptr<Condition> negated();

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Condition");
		Condition::getParents(parents);
	}

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;

	virtual size_t hash() const;
	virtual std::string str() const;
	virtual std::shared_ptr<BaseElement> copy();
};

} /* namespace pddl */

#endif /* HA9E72090_CA55_47AB_B142_B5C348ADE96C */
