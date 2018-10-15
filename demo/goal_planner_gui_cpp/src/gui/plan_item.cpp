/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: plan_item.cpp
 */

#include <goal_planner_gui/goals/goal_context.h>
#include <goal_planner_gui/gui/image_text_item.h>
#include <goal_planner_gui/gui/plan_item.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/planning/plan_node.h>
#include <goal_planner_gui/reference_list.h>

namespace goal_planner_gui
{

PlanItem::PlanItem()
{
}

PlanItem::PlanItem(const std::shared_ptr<PlanNode>& node,
		const std::shared_ptr<GoalContextResult>& context) :
				m_node(node),
				m_context(context)
{
	computeArgs();

	m_nameItem = new ImageTextItem(context->goalContext->m_refs->getName(node->getAction()->m_name),
			context->goalContext->m_refs->getImage(node->getAction()->m_name));
}

PlanItem::~PlanItem()
{
}

QString PlanItem::getActionStatus()
{
	return QString::fromStdString(m_node->getStatus());
}

void PlanItem::computeArgs()
{
	auto& action = m_node->getAction();
	std::unordered_set<std::shared_ptr<pddl::Parameter>, pddl::TypedObjectHasher> relevantArgs;
	action->m_effect->collectFreeVars(relevantArgs);

	ASSERT(m_node->getArgs().size() == action->m_args.size())

	LOG_INFO("Compute Args");

	for (size_t i = 0; i < m_node->getArgs().size(); ++i)
	{
		auto& o = m_node->getArgs()[i];
		auto& a = action->m_args[i];

		LOG_INFO(o << " " << a->str());

		if (CONTAINS_NOT(a, relevantArgs))
		{
			continue;
		}

		PartitionPtr p = Partition::getInitialPartition(o->getType(), m_context->goalContext->m_refs);
		PartitionEntryPtr refEntry = p->getChildForObject(o);
		PartitionEntryPtr finalEntry;
		bool checkInformation = false;

		while (refEntry && refEntry != finalEntry)
		{
			finalEntry = refEntry;
			refEntry = refEntry->expandSingle(o, checkInformation);
			checkInformation = true;
		}

		std::vector<PartitionEntry::DescriptionEntry> desc;
		finalEntry->description(desc);
		for (auto& it : desc)
		{
			m_args.push_back(QVariant::fromValue(new ImageTextItem(it.text, it.image)));
		}
	}
}

} /* namespace goal_planner_gui */

