/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: disjunction.h
 */

#ifndef H2A6D4F1A_B998_4986_97C9_DE2AFAC705A5
#define H2A6D4F1A_B998_4986_97C9_DE2AFAC705A5

#include <goal_planner_gui/pddl/conditions/junction_condition.h>

namespace pddl
{

class Disjunction: public JunctionCondition
{
public:
	Disjunction(const std::vector<std::shared_ptr<Condition>>& parts,
				std::shared_ptr<Scope> scope = std::shared_ptr<Scope>());
	virtual ~Disjunction();

	virtual std::shared_ptr<Condition> negated();

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "or";
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

#endif /* H2A6D4F1A_B998_4986_97C9_DE2AFAC705A5 */
