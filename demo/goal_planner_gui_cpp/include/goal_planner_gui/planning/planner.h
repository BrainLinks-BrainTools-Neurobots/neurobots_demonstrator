/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Feb 15, 2018
 *      Author: kuhnerd
 * 	  Filename: planner.h
 */

#ifndef H81AF2EC3_CEEC_4D91_884A_499B67E4A0E5
#define H81AF2EC3_CEEC_4D91_884A_499B67E4A0E5

#include <goal_planner_gui/planning/plan.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/state/state.h>

namespace goal_planner_gui
{

class GoalSpec;
class Writer;

class Planner
{
public:
	Planner(const pddl::StatePtr& state,
			const std::string& path);
	virtual ~Planner();

	bool plan(const std::shared_ptr<GoalSpec>& goal);
	const std::shared_ptr<Plan>& getPlan() const;
	const pddl::ProblemPtr& getProblem() const;
	const std::string& getProblemPath() const;
	bool executeAction(const std::shared_ptr<PlanNode>& planNode);

private:
	void setupFastDownward();

private:
	pddl::StatePtr m_state;
	pddl::ProblemPtr m_problem;
	std::shared_ptr<Writer> m_writer;
	std::shared_ptr<Plan> m_plan;
	const std::string m_packagePath;
	const std::string m_tmpPath;
	const std::string m_domainPath;
	const std::string m_problemPath;
	const std::string m_planPath;
	std::string m_fastDownwardBuild;
	const std::string m_translateBin;
	const std::string m_preprocessBin;
	const std::string m_searchBin;
	const std::string m_searchArgs;
};

} /* namespace goal_planner_gui */

#endif /* H81AF2EC3_CEEC_4D91_884A_499B67E4A0E5 */
