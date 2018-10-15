/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 12, 2017
 *      Author: kuhnerd
 * 	  Filename: alternative_partition_entry.cpp
 */

#include <goal_planner_gui/alternative_partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/exception.h>

namespace goal_planner_gui
{

AlternativePartitionEntry::AlternativePartitionEntry(const std::shared_ptr<Reference>& ref,
		const std::shared_ptr<Partition>& partition,
		const std::shared_ptr<PartitionEntry>& alternativeParent,
		const std::shared_ptr<Partition>& forbidPartition) :
				PartitionEntry(ref, partition),
				m_forbidPartition(forbidPartition),
				m_alternativeParent(alternativeParent)
{
}

AlternativePartitionEntry::~AlternativePartitionEntry()
{
}

void AlternativePartitionEntry::getAlternativeParents(std::unordered_set<std::shared_ptr<AlternativePartitionEntry>, PartitionEntryHasher>& alternatives)
{
	if (!m_alternativeParent)
		return;
	else
	{
		m_alternativeParent->getAlternativeParents(alternatives);
		alternatives.insert(std::static_pointer_cast<AlternativePartitionEntry>(m_alternativeParent));
	}
}

bool AlternativePartitionEntry::isQuantified()
{
	return false;
}

std::string AlternativePartitionEntry::str() const
{
	return "AlternativePartitionEntry: " + PartitionEntry::str();
}

std::shared_ptr<PartitionEntry> AlternativePartitionEntry::clone()
{
	std::shared_ptr<PartitionEntry> entry(new AlternativePartitionEntry(m_ref, m_partition, m_alternativeParent, m_forbidPartition));
	entry->setReachable(m_reachableObjects);
	entry->setAllRefs(m_allRefs);
	return entry;
}

void AlternativePartitionEntry::__determineFobiddenPartitionsAndRefs(std::vector<std::shared_ptr<Partition> >& forbiddenPartitions,
		std::vector<std::shared_ptr<Reference> >& forbiddenRefs)
{
	std::unordered_set<std::shared_ptr<AlternativePartitionEntry>, PartitionEntryHasher> alternativeParents;
	getAlternativeParents(alternativeParents);
	for (auto& p : alternativeParents)
	{
		auto& forbidPart = p->getForbidPartition();
		if (forbidPart)
		{
			forbiddenPartitions.push_back(forbidPart);
			forbiddenRefs.push_back(forbidPart->getExpandParent()->getRef());
		}
	}

	forbiddenRefs.push_back(m_forbidPartition->getExpandParent()->getRef());
	forbiddenPartitions.push_back(m_forbidPartition);
}

const std::shared_ptr<Partition>& AlternativePartitionEntry::getForbidPartition() const
{
	return m_forbidPartition;
}

void AlternativePartitionEntry::setForbidPartition(const std::shared_ptr<Partition>& forbidPartition)
{
	m_forbidPartition = forbidPartition;
}

std::shared_ptr<PartitionEntry> AlternativePartitionEntry::makeAlternative(const std::shared_ptr<Partition>& newPartition)
{
	//TODO: potential problem? newPartition is changed
	if (!newPartition->getExpandParent())
	{
		newPartition->setExpandParent(newPartition->getChildren()[0]);
	}

	//TODO: getReferences().begin() != python
	return std::shared_ptr<PartitionEntry>(new AlternativePartitionEntry(*(getReferences().begin()), newPartition, shared_from_this(), newPartition));
}

std::string AlternativePartitionEntry::getSpecialTypeQualifier()
{
	return "some";
}

} /* namespace goal_planner_gui */

