/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: universal_condition.h
 */

#ifndef H7CD8FD18_F68E_4B6D_95ED_2817D34F96AD
#define H7CD8FD18_F68E_4B6D_95ED_2817D34F96AD

#include <goal_planner_gui/pddl/conditions/quantified_condition.h>

namespace pddl
{

class UniversalCondition: public QuantifiedCondition
{
public:
	UniversalCondition();
	virtual ~UniversalCondition();

	virtual std::shared_ptr<Condition> negated();

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "forall";
	}

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("QuantifiedCondition");
		QuantifiedCondition::getParents(parents);
	}

	virtual std::string str() const;
};

} /* namespace pddl */

#endif /* H7CD8FD18_F68E_4B6D_95ED_2817D34F96AD */
