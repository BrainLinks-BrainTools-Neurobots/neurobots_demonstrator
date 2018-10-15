/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 14, 2017
 *      Author: kuhnerd
 * 	  Filename: universial_partition_entry.h
 */

#ifndef HA14C0F19_0EF1_4B2E_B852_90791B969165
#define HA14C0F19_0EF1_4B2E_B852_90791B969165

#include <goal_planner_gui/partition_entry.h>

namespace goal_planner_gui
{

class UniversalPartitionEntry: public PartitionEntry
{
public:
	UniversalPartitionEntry(const std::shared_ptr<Reference>& ref,
			const std::shared_ptr<Partition>& partition);
	virtual ~UniversalPartitionEntry();

	virtual bool isQuantified();
	virtual std::string str() const;

	virtual std::shared_ptr<PartitionEntry> clone();

protected:
	virtual std::string getSpecialTypeQualifier();
};

} /* namespace goal_planner_gui */

#endif /* HA14C0F19_0EF1_4B2E_B852_90791B969165 */
