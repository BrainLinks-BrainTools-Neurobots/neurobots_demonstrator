/*
 * omnirob_planning_interface.h
 *
 *  Created on: Oct 12, 2015
 *      Author: kuhnerd
 */

#ifndef OMNIROB_PLANNING_INTERFACE_H_
#define OMNIROB_PLANNING_INTERFACE_H_

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <omnirob_planning_interface/plan.h>

namespace omnirob_planning_interface
{

enum Planner
{
	RRTConnect,
	RRT,
	RRTStar
};

class OmnirobPlanningInterface
{
public:
	struct Joint {
		std::string name;
		double value;
	};

	//planningFrame is the base frame (e.g., lbr_0_link)
	//endEffectorFrame is the name of the link, that
	//should be moved to the goal poses which are provided
	//to the plan methods (e.g., sdh2_push_link)
	OmnirobPlanningInterface(const std::string& planningFrame,
			const std::string& endEffectorFrame,
			const std::string& planningGroup,
			const double maxPlanningTime,
			const int maxPlanningAttempts);
	virtual ~OmnirobPlanningInterface();

	//try to find a plan
	virtual bool plan(const Eigen::Affine3d& goal,
			bool allowCollisions, //between hand and environment
			Plan& result);
	virtual bool plan(const Eigen::Affine3d& goal,
			std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
			bool allowCollisions, //between hand and environment
			Plan& result);
	virtual bool plan(const moveit_msgs::RobotState& start,
			const Eigen::Affine3d& goal,
			bool allowCollisions, //between hand and environment
			Plan& result);
	virtual bool plan(const moveit_msgs::RobotState& start,
			const Eigen::Affine3d& goal,
			std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
			bool allowCollisions, //between hand and environment
			Plan& result);

	//takes the last waypoint of other to start with planning
	//and directly combines both plans (if no errors occurred)
	virtual bool plan(const Plan& other,
			const Eigen::Affine3d& goal,
			bool allowCollisions, //between hand and environment
			Plan& result);
	virtual bool plan(const Plan& other,
			const Eigen::Affine3d& goal,
			std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
			bool allowCollisions, //between hand and environment
			Plan& result);

	//executes a plan
	virtual bool execute(const Plan& plan);

	//move directly, i.e., plan and execute
	virtual bool move(const Eigen::Affine3d& goal,
			bool allowCollisions); //between hand and environment
	virtual bool move(const moveit_msgs::RobotState& start,
			const Eigen::Affine3d& goal,
			bool allowCollisions); //between hand and environment

	virtual bool isStateInCollision(const moveit_msgs::RobotState& state);

	virtual void visualizePlan(const Plan& plan);
	Planner getPlanner() const;
	void setPlanner(Planner planner);

protected:
	bool updatePlanningScene();
	void setCollisionsAllowed();
	void setCollisionsDisallowed();

private:
	bool planTrajectory(const moveit_msgs::RobotState* start,
			const Eigen::Affine3d& goal,
			std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
			bool allowCollisions, //between hand and environment
			Plan& result);
	std::string getPlannerString();

protected:
	ros::NodeHandle m_privateNodeHandle;
	ros::Publisher m_pubTrajectory;
	ros::Publisher m_pubPlanningScene;
	robot_model_loader::RobotModelLoader m_robotModelLoader;
	ros::ServiceClient m_serviceGetPlanningScene;
	ros::ServiceClient m_serviceExecuteTrajectory;
	robot_model::RobotModelPtr m_robotModel;
	planning_scene::PlanningScenePtr m_planningScene;
	planning_pipeline::PlanningPipeline* m_planningPipeline;
	Planner m_planner;
	boost::mutex m_deleteMutex;

	boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> m_controller;
	bool m_controllerConnected;

	const std::string m_planningFrame;
	const std::string m_endEffectorFrame;
	const std::string m_planningGroup;
	const double m_maxPlanningTime;
	const double m_maxPlanningAttempts;
};

} /* namespace omnirob_planning_interface */

#endif /* OMNIROB_PLANNING_INTERFACE_H_ */
