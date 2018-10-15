/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: existential_condition.h
 */

#ifndef HD33964D1_F922_4BE7_9CF4_F74BC306F515
#define HD33964D1_F922_4BE7_9CF4_F74BC306F515

#include <goal_planner_gui/pddl/conditions/quantified_condition.h>

namespace pddl
{

class ExistentialCondition: public QuantifiedCondition
{
public:
	ExistentialCondition();
	virtual ~ExistentialCondition();

	virtual std::shared_ptr<Condition> negated();

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "exists";
	}

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("QuantifiedCondition");
		QuantifiedCondition::getParents(parents);
	}

	virtual std::string str() const;
};

} /* namespace pddl */

#endif /* HD33964D1_F922_4BE7_9CF4_F74BC306F515 */
