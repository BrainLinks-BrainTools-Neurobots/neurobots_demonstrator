/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 29, 2017
 *      Author: kuhnerd
 * 	  Filename: predicates.h
 */

#ifndef H977A3FB5_1308_4128_94DB_F5012253BF3C
#define H977A3FB5_1308_4128_94DB_F5012253BF3C
#include <goal_planner_gui/pddl/constants.h>
#include <goal_planner_gui/pddl/functions.h>
#include <unordered_map>

namespace pddl
{

class Element;

class Predicate: public Function
{
public:
	Predicate(const std::string& name,
			const std::vector<std::shared_ptr<Parameter>>& args,
			bool builtin = false,
			int functionScope = SCOPE_ALL);
	virtual ~Predicate();

	virtual std::string str() const;

	virtual std::shared_ptr<Function> copy();

	static std::shared_ptr<Predicate> parse(std::shared_ptr<Element> it,
			std::unordered_map<std::string, std::shared_ptr<Type>>& types);
};

} /* namespace pddl */

#endif /* H977A3FB5_1308_4128_94DB_F5012253BF3C */
