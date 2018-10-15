/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: feature_reference.h
 */

#ifndef H27C39D50_0221_4F5B_B653_F3A290E75221
#define H27C39D50_0221_4F5B_B653_F3A290E75221

#include <goal_planner_gui/references/relational_reference.h>
#include <goal_planner_gui/pddl/functions.h>

namespace goal_planner_gui
{

class PartitionEntry;
class ReferenceList;

class FeatureReference: public goal_planner_gui::RelationalReference
{
public:
	FeatureReference(const std::shared_ptr<pddl::Function>& function,
			const std::shared_ptr<PartitionEntry>& value,
			const std::shared_ptr<ReferenceList>& refContext);
	virtual ~FeatureReference();

	virtual inline Reference::Type getReferenceType() const
	{
		return TypeFeatureReference;
	}

	virtual std::shared_ptr<RelationalReference> clone(const std::vector<std::shared_ptr<PartitionEntry>>& args,
	const std::shared_ptr<PartitionEntry>& value);
};

VECTOR_DEF(FeatureReference);

} /* namespace goal_planner_gui2 */

#endif /* H27C39D50_0221_4F5B_B653_F3A290E75221 */
