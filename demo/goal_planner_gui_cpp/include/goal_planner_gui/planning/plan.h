/*
 * plan.h
 *
 *  Created on: Feb 17, 2018
 *      Author: kuhnerd
 */

#ifndef HC41FB4C2_9502_424B_83D2_DC2D382FFAA8
#define HC41FB4C2_9502_424B_83D2_DC2D382FFAA8

#include <goal_planner_gui/planning/plan_node.h>
#include <goal_planner_gui/pddl/problem.h>
#include <memory>

namespace goal_planner_gui
{

class Plan
{
public:
	Plan(const std::vector<std::shared_ptr<PlanNode>>& plan);
	virtual ~Plan();

	static std::shared_ptr<Plan> parse(const std::string& file,
			const pddl::ProblemPtr& problem);

	std::vector<std::shared_ptr<PlanNode>>::iterator begin();
	std::vector<std::shared_ptr<PlanNode>>::const_iterator begin() const;

	std::vector<std::shared_ptr<PlanNode>>::iterator end();
	std::vector<std::shared_ptr<PlanNode>>::const_iterator end() const;

	std::shared_ptr<PlanNode>& operator[](const int index);

	size_t size() const;

private:
	std::vector<std::shared_ptr<PlanNode>> m_plan;
};

} /* namespace goal_planner_gui */

#endif /* HC41FB4C2_9502_424B_83D2_DC2D382FFAA8 */
