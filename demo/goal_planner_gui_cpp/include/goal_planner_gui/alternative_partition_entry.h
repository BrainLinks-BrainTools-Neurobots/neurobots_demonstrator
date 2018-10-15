/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 12, 2017
 *      Author: kuhnerd
 * 	  Filename: alternative_partition_entry.h
 */

#ifndef H4FC5BA6F_3F5C_4A7B_B83A_9B4E7E1A4375
#define H4FC5BA6F_3F5C_4A7B_B83A_9B4E7E1A4375

#include <goal_planner_gui/partition_entry.h>

namespace goal_planner_gui
{

class AlternativePartitionEntry: public PartitionEntry
{
public:
	AlternativePartitionEntry(const std::shared_ptr<Reference>& ref,
			const std::shared_ptr<Partition>& partition,
			const std::shared_ptr<PartitionEntry>& alternativeParent,
			const std::shared_ptr<Partition>& forbidPartition);
	virtual ~AlternativePartitionEntry();

	virtual void getAlternativeParents(std::unordered_set<std::shared_ptr<AlternativePartitionEntry>, PartitionEntryHasher>& alternatives);
	const std::shared_ptr<Partition>& getForbidPartition() const;
	void setForbidPartition(const std::shared_ptr<Partition>& forbidPartition);

	virtual bool isQuantified();
	virtual std::string str() const;
	virtual std::shared_ptr<PartitionEntry> makeAlternative(const std::shared_ptr<Partition>& newPartition);
	virtual std::shared_ptr<PartitionEntry> clone();

protected:
	virtual void __determineFobiddenPartitionsAndRefs(std::vector<std::shared_ptr<Partition>>& forbiddenPartitions,
			std::vector<std::shared_ptr<Reference>>& forbiddenRefs);
	virtual std::string getSpecialTypeQualifier();

private:
	std::shared_ptr<Partition> m_forbidPartition;
	std::shared_ptr<PartitionEntry> m_alternativeParent;

};

} /* namespace goal_planner_gui */

#endif /* H4FC5BA6F_3F5C_4A7B_B83A_9B4E7E1A4375 */
