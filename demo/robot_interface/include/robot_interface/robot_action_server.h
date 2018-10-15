/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: robot_action_server.h
 */

#ifndef ROBOT_ACTION_SERVER_H_
#define ROBOT_ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <database_conversions/knowledge_base.h>

#ifdef ROADMAP_PLANNER_FOUND
#include <prm_planner_msgs/GoalAction.h>
#endif

#ifdef ROBOT_INTERFACE_FOUND
#include <robot_interface_definition/robot_interface.h>
#endif

//Actions from "robot_interface_msgs" package
#include <robot_interface_msgs/moveToAction.h>
#include <robot_interface_msgs/drinkSubjectAction.h>
#include <robot_interface_msgs/dropObjectAction.h>
#include <robot_interface_msgs/graspObjectAction.h>
#include <robot_interface_msgs/pourLiquidAction.h>

#ifdef ROADMAP_PLANNER_FOUND
#include <prm_planner_msgs/SetState.h>
#include <std_srvs/Empty.h>
#endif

namespace robot_interface
{

class RobotInterfaceServer
{
public:

	RobotInterfaceServer();
	~RobotInterfaceServer();

	bool initSystem();

	//MoveTo Action Callback
	void execute_moveTo(const robot_interface_msgs::moveToGoalConstPtr &goal);
	//DrinkSubject Action Callback
	void execute_drinkSubject(const robot_interface_msgs::drinkSubjectGoalConstPtr &goal);
	//Drop Object Action Callback
	void execute_dropObject(const robot_interface_msgs::dropObjectGoalConstPtr &goal);
	//Grasp Object Action Callback
	void execute_graspObject(const robot_interface_msgs::graspObjectGoalConstPtr &goal);
	//Pour Liquid Action Callback
	void execute_pourLiquid(const robot_interface_msgs::pourLiquidGoalConstPtr &goal);

private:
	bool getDataFromDatabase(const std::string& name,
			Object& goalObject);
	bool toggleImageStream(bool start);
	bool showImageInGui(const std::string& topic);

private:
	//Node Handle
	ros::NodeHandle m_nh;

	//MoveTo Action Server
	actionlib::SimpleActionServer<robot_interface_msgs::moveToAction> m_as_moveTo;
	actionlib::SimpleActionServer<robot_interface_msgs::drinkSubjectAction> m_as_drinkSubject;
	actionlib::SimpleActionServer<robot_interface_msgs::dropObjectAction> m_as_dropObject;
	actionlib::SimpleActionServer<robot_interface_msgs::graspObjectAction> m_as_graspObject;
	actionlib::SimpleActionServer<robot_interface_msgs::pourLiquidAction> m_as_pourLiquid;

	//Service clients for RRT* planning and execution
	ros::ServiceClient m_client_set_planning_scene_info;
	ros::ServiceClient m_client_set_edge_cost_weights;
	ros::ServiceClient m_client_run_planner_goal_pose;
	ros::ServiceClient m_client_execute_trajectory_robot;

	//Service calls
	ros::ServiceClient m_clientGetObject;
	ros::ServiceClient m_clientUpdateObject;
	ros::ServiceClient m_clientOmnirobArm;
	ros::ServiceClient m_clientGetOmnirobImages;
	ros::ServiceClient m_clientCameraStreamInGUI;

	Eigen::Affine3d m_transportEEFPose;
	Eigen::Affine3d m_facePrePosition;

	//BiRRT planner
#ifdef ROBOT_INTERFACE_FOUND
	ros::ServiceClient m_clientBirrtPlanner;
	#endif

	//PRM planner
#ifdef ROADMAP_PLANNER_FOUND
//	robot_interface_definition::RobotInterface* m_roadmapPlannerGraspAndDropOmnirob;
//	robot_interface_definition::RobotInterface* m_roadmapPlannerOmnirobPouring;
//	robot_interface_definition::RobotInterface* m_roadmapPlannerOmnirobDrinking;

	typedef actionlib::SimpleActionClient<prm_planner_msgs::GoalAction> PRMActionClient;

	PRMActionClient* m_actionClientRoadmapPlannerDefault; //for drop planning
	PRMActionClient* m_actionClientRoadmapPlannerDefaultYDown; //for eef planning
	PRMActionClient* m_actionClientRoadmapPlannerPouring;
	PRMActionClient* m_actionClientRoadmapPlannerDrinking;

	ros::ServiceClient m_clientGetDataOmnirobDefault;
	ros::ServiceClient m_clientGetDataOmnirobDefaultYDown;
	ros::ServiceClient m_clientGetDataOmnirobPouring;
	ros::ServiceClient m_clientGetDataOmnirobDrinking;
	ros::ServiceClient m_clientSetObjectPoseTypeOmnirobDrinking;

	ros::ServiceServer m_serverStopRoadmapPlanner;

	boost::atomic_bool m_stopRoadmapPlanner;

	bool prmPlannerSendGoal(const Eigen::Affine3d& goal,
			bool yDown,
			bool reactToStop = true);
	bool prmPlannerSendGoal(const std::vector<std::string>& params,
			PRMActionClient* client,
			bool reactToStop = true);
	bool prmPlannerSendDrop(const std::string& object,
			const Eigen::Affine3d& goal);
	bool prmPlannerSendGrasp(const std::string& object);

	bool stopRoadmapPlanner(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &res);
#endif

};

}

#endif /* ROBOT_ACTION_SERVER_H_ */
