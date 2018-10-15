/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 14, 2017
 *      Author: kuhnerd
 * 	  Filename: existential_partition_entry.cpp
 */
#include <goal_planner_gui/existential_partition_entry.h>

namespace goal_planner_gui
{

ExistentialPartitionEntry::ExistentialPartitionEntry(const std::shared_ptr<Reference>& ref,
		const std::shared_ptr<Partition>& partition) :
				PartitionEntry(ref, partition)
{
}

ExistentialPartitionEntry::~ExistentialPartitionEntry()
{
}

bool ExistentialPartitionEntry::isQuantified()
{
	return true;
}

std::string ExistentialPartitionEntry::str() const
{
	std::string res;
	for (const auto& r : getReferences())
	{
		res += r->str() + ", ";
	}
	if (!res.empty())
	{
		res.pop_back();
		res.pop_back();
	}
	return "ExistentialPartitionEntry [" + res + "]";
}

std::shared_ptr<PartitionEntry> ExistentialPartitionEntry::clone()
{
	std::shared_ptr<PartitionEntry> entry(new ExistentialPartitionEntry(m_ref, m_partition));
	entry->setReachable(m_reachableObjects);
	entry->setAllRefs(m_allRefs);
	return entry;
}

std::string ExistentialPartitionEntry::getSpecialTypeQualifier()
{
	return "an arbitrary";
}

} /* namespace goal_planner_gui */
