/*
 * plan_node.h
 *
 *  Created on: Feb 17, 2018
 *      Author: kuhnerd
 */

#ifndef H493DB886_44A2_4992_AF20_7D67DB460F0C
#define H493DB886_44A2_4992_AF20_7D67DB460F0C
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/types.h>

#define ACTION_STATUS_EXECUTABLE "EXECUTABLE"
#define ACTION_STATUS_IN_PROGRESS "IN_PROGRESS"
#define ACTION_STATUS_EXECUTED "EXECUTED"
#define ACTION_STATUS_FAILED "FAILED"
#define ACTION_STATUS_UNSUCCESSFUL "UNSUCCESSFUL"

namespace goal_planner_gui
{

class PlanNode
{
public:
	PlanNode(const pddl::ActionPtr& action,
			const pddl::TypedObjectVector& args);
	virtual ~PlanNode();

	static std::shared_ptr<PlanNode> parse(const std::string& line,
			const pddl::ProblemPtr& problem);
	const pddl::ActionPtr& getAction() const;
	const pddl::TypedObjectVector& getArgs() const;
	const std::string& getStatus() const;
	void setStatus(const std::string& status);
	virtual std::string str();

private:
	pddl::ActionPtr m_action;
	pddl::TypedObjectVector m_args;
	std::string m_status;
};

} /* namespace goal_planner_gui */

#endif /* H493DB886_44A2_4992_AF20_7D67DB460F0C */
