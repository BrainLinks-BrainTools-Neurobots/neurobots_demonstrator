/*
 * plan.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: kuhnerd
 */

#include <omnirob_planning_interface/plan.h>
#include <ros/ros.h>

namespace omnirob_planning_interface
{

Plan::Plan()
{
}

Plan::~Plan()
{
}

void Plan::appendPlan(Plan& other)
{
	assert(m_plan.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
	assert(other.m_plan.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
	assert(m_plan.group_name == other.m_plan.group_name);

	m_plan.planning_time += other.m_plan.planning_time;
	auto& otherPoints = other.m_plan.trajectory.joint_trajectory.points;
	auto& thisPoints = m_plan.trajectory.joint_trajectory.points;
	ros::Duration end = thisPoints.back().time_from_start;
	for (auto& it : otherPoints)
	{
		it.time_from_start += end;
	}

	thisPoints.insert(thisPoints.end(), otherPoints.begin(), otherPoints.end());
}

bool Plan::getLastWaypoint(moveit_msgs::RobotState& state) const
		{
	if (m_plan.trajectory.joint_trajectory.points.empty())
	{
		return false;
	}

	state.joint_state.name = m_plan.trajectory.joint_trajectory.joint_names;
	state.joint_state.position = m_plan.trajectory.joint_trajectory.points.back().positions;

	return true;
}

bool Plan::getInterpolatedWaypoint(const ros::Time& now,
		moveit_msgs::RobotState& state,
		bool& finished,
		double& t) const
		{
	if (m_plan.trajectory.joint_trajectory.points.empty())
	{
		finished = false;
		t = 0.0;
		return false;
	}

	double nowS = now.toSec();

	if (nowS > m_plan.trajectory.joint_trajectory.points.back().time_from_start.toSec())
	{
		finished = true;
		t = 1.0;
		return getLastWaypoint(state);
	}

	//find current goal point
	int i = 1;
	while (i < m_plan.trajectory.joint_trajectory.points.size())
	{
		if (nowS < m_plan.trajectory.joint_trajectory.points[i].time_from_start.toSec())
		{
			break;
		}
		++i;
	}

	//interpolate
	const trajectory_msgs::JointTrajectoryPoint& pNew = m_plan.trajectory.joint_trajectory.points[i];
	const trajectory_msgs::JointTrajectoryPoint& pOld = m_plan.trajectory.joint_trajectory.points[i - 1];
	const std::vector<double>& posNew = pNew.positions;
	const std::vector<double>& posOld = pOld.positions;
	double timeNew = pNew.time_from_start.toSec();
	double timeOld = pOld.time_from_start.toSec();

	t = (nowS - timeOld) / (timeNew - timeOld);

	state.joint_state.name = m_plan.trajectory.joint_trajectory.joint_names;
	state.joint_state.position.resize(posNew.size());

	for (size_t j = 0; j < posNew.size(); ++j)
	{
		state.joint_state.position[j] = ((1.0 - t) * posOld[j] + t * posNew[j]);
	}

	finished = false;

	return true;
}

} /* namespace omnirob_planning_interface */

