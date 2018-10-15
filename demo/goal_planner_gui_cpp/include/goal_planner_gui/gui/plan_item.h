/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: plan_item.h
 */

#ifndef H4847CEB9_49D9_4964_92B1_F58311F583E1
#define H4847CEB9_49D9_4964_92B1_F58311F583E1

#include <goal_planner_gui/gui/ref_item.h>
#include <memory>

namespace goal_planner_gui
{

class PlanNode;
struct GoalContextResult;

class PlanItem: public RefItem
{
	Q_OBJECT

public:
	PlanItem();
	PlanItem(const std::shared_ptr<PlanNode>& node,
			const std::shared_ptr<GoalContextResult>& context);
	virtual ~PlanItem();

	QString getActionStatus();

	Q_PROPERTY(QString actionStatus READ getActionStatus CONSTANT);

private:
	void computeArgs();

private:
	std::shared_ptr<PlanNode> m_node;
	std::shared_ptr<GoalContextResult> m_context;
};

} /* namespace goal_planner_gui */

Q_DECLARE_METATYPE(goal_planner_gui::PlanItem);

#endif /* H4847CEB9_49D9_4964_92B1_F58311F583E1 */
