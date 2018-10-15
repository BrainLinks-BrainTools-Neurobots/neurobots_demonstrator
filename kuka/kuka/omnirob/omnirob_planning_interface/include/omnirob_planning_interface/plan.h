/*
 * plan.h
 *
 *  Created on: Oct 12, 2015
 *      Author: kuhnerd
 */

#ifndef OMNIROB_PLANNING_INTERFACE_PLAN_H_
#define OMNIROB_PLANNING_INTERFACE_PLAN_H_

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/RobotState.h>

namespace omnirob_planning_interface
{

class Plan
{
public:
	Plan();
	virtual ~Plan();

	void appendPlan(Plan& other);
	bool getLastWaypoint(moveit_msgs::RobotState& state) const;
	bool getInterpolatedWaypoint(const ros::Time& now,
			moveit_msgs::RobotState& state,
			bool& finished,
			double& t) const;

	moveit_msgs::MotionPlanResponse m_plan;
};

} /* namespace omnirob_planning_interface */

#endif /* OMNIROB_PLANNING_INTERFACE_PLAN_H_ */
