/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 14, 2017
 *      Author: kuhnerd
 * 	  Filename: universial_partition_entry.cpp
 */

#include <goal_planner_gui/universal_partition_entry.h>

namespace goal_planner_gui
{

UniversalPartitionEntry::UniversalPartitionEntry(const std::shared_ptr<Reference>& ref,
		const std::shared_ptr<Partition>& partition) :
				PartitionEntry(ref, partition)
{

}

UniversalPartitionEntry::~UniversalPartitionEntry()
{
}

bool UniversalPartitionEntry::isQuantified()
{
	return true;
}

std::string UniversalPartitionEntry::str() const
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
	return "UniversalPartitionEntry [" + res + "]";
}

std::shared_ptr<PartitionEntry> UniversalPartitionEntry::clone()
{
	std::shared_ptr<PartitionEntry> entry(new UniversalPartitionEntry(m_ref, m_partition));
	entry->setReachable(m_reachableObjects);
	entry->setAllRefs(m_allRefs);
	return entry;
}

std::string UniversalPartitionEntry::getSpecialTypeQualifier()
{
	return "all";
}

} /* namespace goal_planner_gui */

