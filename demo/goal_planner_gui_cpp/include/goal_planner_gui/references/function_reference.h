/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: function_reference.h
 */

#ifndef H19B682E0_C290_4E5F_BA43_538CB270561A
#define H19B682E0_C290_4E5F_BA43_538CB270561A

#include <goal_planner_gui/references/relational_reference.h>
#include <goal_planner_gui/pddl/functions.h>

namespace goal_planner_gui
{

class ReferenceList;
class PartitionEntry;

class FunctionReference: public goal_planner_gui::RelationalReference
{
public:
	FunctionReference(const std::shared_ptr<pddl::Function>& function,
			const std::vector<std::shared_ptr<PartitionEntry>>& args,
			const std::shared_ptr<ReferenceList>& refContext);
	virtual ~FunctionReference();

	virtual inline Reference::Type getReferenceType() const
	{
		return TypeFunctionReference;
	}

	virtual std::shared_ptr<RelationalReference> clone(const std::vector<std::shared_ptr<PartitionEntry>>& args,
		const std::shared_ptr<PartitionEntry>& value);
};

VECTOR_DEF(FunctionReference);

} /* namespace goal_planner_gui2 */

#endif /* H19B682E0_C290_4E5F_BA43_538CB270561A */
