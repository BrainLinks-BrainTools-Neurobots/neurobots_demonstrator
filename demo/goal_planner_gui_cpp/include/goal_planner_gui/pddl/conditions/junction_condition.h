/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: junction_condition.h
 */

#ifndef H20FD7365_37EF_4C3E_B58C_914FEB1093CD
#define H20FD7365_37EF_4C3E_B58C_914FEB1093CD

#include <goal_planner_gui/pddl/conditions/condition.h>

namespace pddl
{

class JunctionCondition: public Condition
{
public:
	JunctionCondition(const std::vector<std::shared_ptr<Condition>>& parts,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>());
	virtual ~JunctionCondition();

	virtual std::shared_ptr<Condition> negated() = 0;

	static void parse(const std::string& tag,
			std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope,
			std::vector<std::shared_ptr<Condition>>& res);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Condition");
		Condition::getParents(parents);
	}

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;
	const std::vector<std::shared_ptr<Condition> >& getParts() const;
	virtual std::string str() const;

protected:
	std::vector<std::shared_ptr<Condition>> m_parts;
};

} /* namespace pddl */

#endif /* H20FD7365_37EF_4C3E_B58C_914FEB1093CD */
