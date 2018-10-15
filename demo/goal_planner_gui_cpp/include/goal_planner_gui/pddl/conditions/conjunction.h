/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: conjunction.h
 */

#ifndef H7983152F_8957_4C8E_83AD_73D0B90B5AC7
#define H7983152F_8957_4C8E_83AD_73D0B90B5AC7

#include <goal_planner_gui/pddl/conditions/junction_condition.h>

namespace pddl
{

class Conjunction: public JunctionCondition
{
public:
	Conjunction(const std::vector<std::shared_ptr<Condition>>& parts,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>());
	virtual ~Conjunction();

	virtual std::shared_ptr<Condition> negated();

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "and";
	}

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("JunctionCondition");
		JunctionCondition::getParents(parents);
	}

	virtual std::string str() const;

	virtual std::shared_ptr<BaseElement> copy();
};

} /* namespace pddl */

#endif /* H7983152F_8957_4C8E_83AD_73D0B90B5AC7 */
