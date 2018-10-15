/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: preference_condition.h
 */

#ifndef HC143DD20_B51B_4B0A_98B5_C65C86D25A4B
#define HC143DD20_B51B_4B0A_98B5_C65C86D25A4B

#include <goal_planner_gui/pddl/conditions/condition.h>

namespace pddl
{

class PreferenceCondition: public Condition
{
public:
	PreferenceCondition();
	virtual ~PreferenceCondition();

	virtual std::shared_ptr<Condition> negated();

	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "preference";
	}

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Condition");
		Condition::getParents(parents);
	}

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;
	virtual std::string str() const;

	virtual std::shared_ptr<BaseElement> copy();
};

} /* namespace pddl */

#endif /* HC143DD20_B51B_4B0A_98B5_C65C86D25A4B */
