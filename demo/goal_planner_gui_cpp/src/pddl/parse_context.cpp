/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 9, 2017
 *      Author: kuhnerd
 * 	  Filename: parse_context.cpp
 */

#include <goal_planner_gui/pddl/modules.h>
#include <goal_planner_gui/pddl/parse_context.h>
#include <goal_planner_gui/pddl/utils.h>

namespace pddl
{

ParseContext::ParseContext()
{
}

ParseContext::~ParseContext()
{
}

void ParseContext::addClass(const std::string& name,
		const std::string& c)
{
	m_classes[name].push_back(c);
}

void ParseContext::getHandlers(const std::string& name,
		const std::string& tag)
{
	for (auto& it : m_classes)
	{
		LOG_INFO("class: " << it.first);
		for (auto& it2 : it.second)
		{
			LOG_INFO("\t- " <<it2);
		}
	}
}

} /* namespace pddl */

