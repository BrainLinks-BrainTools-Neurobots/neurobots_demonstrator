/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 14, 2017
 *      Author: kuhnerd
 * 	  Filename: existential_partition_entry.h
 */

#ifndef H3DD6DE96_084C_42B0_982A_AD817F4952F1
#define H3DD6DE96_084C_42B0_982A_AD817F4952F1

#include <goal_planner_gui/partition_entry.h>

namespace goal_planner_gui
{

class ExistentialPartitionEntry: public PartitionEntry
{
public:
	ExistentialPartitionEntry(const std::shared_ptr<Reference>& ref,
			const std::shared_ptr<Partition>& partition);
	virtual ~ExistentialPartitionEntry();

	virtual bool isQuantified();
	virtual std::string str() const;

	virtual std::shared_ptr<PartitionEntry> clone();

protected:
	virtual std::string getSpecialTypeQualifier();
};

} /* namespace goal_planner_gui */

#endif /* H3DD6DE96_084C_42B0_982A_AD817F4952F1 */
