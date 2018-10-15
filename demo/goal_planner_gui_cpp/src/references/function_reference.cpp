/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: function_reference.cpp
 */

#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/references/function_reference.h>

namespace goal_planner_gui
{

FunctionReference::FunctionReference(const std::shared_ptr<pddl::Function>& function,
		const std::vector<std::shared_ptr<PartitionEntry>>& args,
		const std::shared_ptr<ReferenceList>& refContext) :
				RelationalReference(function, args, std::shared_ptr<PartitionEntry>(), refContext)
{
}

FunctionReference::~FunctionReference()
{
}

std::shared_ptr<RelationalReference> FunctionReference::clone(const std::vector<std::shared_ptr<PartitionEntry> >& args,
		const std::shared_ptr<PartitionEntry>& value)
{
	std::shared_ptr<RelationalReference> res(new FunctionReference(m_function, args, m_context));
	res->setValue(value);
	return res;
}

} /* namespace goal_planner_gui */

