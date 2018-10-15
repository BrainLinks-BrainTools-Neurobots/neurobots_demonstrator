/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 28, 2017
 *      Author: kuhnerd
 * 	  Filename: visitors.cpp
 */

#include <boost/algorithm/string/trim.hpp>
#include <boost/any.hpp>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/visitors.h>

namespace pddl
{

TermPrintVisitor::TermPrintVisitor(bool instantiated) :
				m_instantiated(instantiated)
{
}

TermPrintVisitor::~TermPrintVisitor()
{
}

boost::any pddl::TermPrintVisitor::operator ()(boost::any value,
		const std::vector<boost::any>& result)
{
	std::shared_ptr<Term> term = boost::any_cast<std::shared_ptr<Term>>(value);
	if (isInstanceSharedCast(term, varTerm, VariableTerm))
	{
		if (m_instantiated && varTerm->isInstantiated())
		{
			return boost::any(varTerm->getInstance()->getName());
		}
	}
	else if (isInstanceSharedCast(term, funcTerm, FunctionTerm))
	{
		std::string r;
		for (auto& it: result)
		r += boost::any_cast<std::string>(it) + " ";
		boost::algorithm::trim(r);
		return boost::any(std::string("(") + funcTerm->getFunction()->getName() + " " + r + ")");
	}

	return boost::any(std::string("[Error]"));
}

} /* namespace pddl */

