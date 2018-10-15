/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: feature_reference.cpp
 */

#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/references/feature_reference.h>

namespace goal_planner_gui
{
FeatureReference::FeatureReference(const std::shared_ptr<pddl::Function>& function,
		const std::shared_ptr<PartitionEntry>& value,
		const std::shared_ptr<ReferenceList>& refContext) :
				RelationalReference(function, { std::shared_ptr<PartitionEntry>() }, value, refContext)
{
}

FeatureReference::~FeatureReference()
{
}

std::shared_ptr<RelationalReference> FeatureReference::clone(const std::vector<std::shared_ptr<PartitionEntry> >& args,
		const std::shared_ptr<PartitionEntry>& value)
{
	std::shared_ptr<RelationalReference> res(new FeatureReference(m_function, value, m_context));
	res->setArgs(args);
	return res;
}

} /* namespace goal_planner_gui */

