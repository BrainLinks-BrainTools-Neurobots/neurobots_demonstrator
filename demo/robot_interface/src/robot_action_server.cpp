/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: robot_action_server.cpp
 */

#include <database_conversions/database_conversions.h>
#include <database_msgs/get_object.h>
#include <database_msgs/update_object.h>
#include <robot_interface/robot_action_server.h>
#include <std_srvs/SetBool.h>
#include <eigen_conversions/eigen_msg.h>
#include <planner_control_msgs/ShowTopic.h>

#ifdef ROBOT_INTERFACE_FOUND
#	include <kuka_manager/JointState.h>

#	ifdef ROADMAP_PLANNER_FOUND
#	include <prm_planner_msgs/GetImage.h>
#	include <prm_planner_msgs/SetState.h>
#	include <prm_planner_msgs/GoalAction.h>
#	include <prm_planner_msgs/SetObjectPoseType.h>
#	endif

#	ifdef BIRRT_PLANNER_FOUND
#	include <planner_msgs/planning_scene_info.h>
#	include <planner_msgs/set_edge_costs.h>
#	include <planner_msgs/run_planner_map_goal_pose.h>
#	endif

#	ifdef BIRRT_TRAJECTORY_EXECUTION_FOUND
#	include <execution_msgs/execute_trajectory_robot.h>
#	endif
#endif

#define OMNIROB_CAMERA "/omnirob_camera_bridge/rgb/image_rect_color"
#define SHELF_LEFT_CAMERA "/shelf_left_camera/rgb/image_raw"
#define SHELF_RIGHT_CAMERA "/shelf_right_camera/rgb/image_raw"
#define FACE_CAMERA "/face_camera/rgb/image_raw"
#define CONTROL_VIEW "control_view"

namespace robot_interface
{

const std::vector<std::string> omnirobNames = { "lbr_1_joint", "lbr_2_joint", "lbr_3_joint", "lbr_4_joint", "lbr_5_joint", "lbr_6_joint",
		"lbr_7_joint" };

RobotInterfaceServer::RobotInterfaceServer() :
				m_as_moveTo(m_nh, "neurobots_demo/move_to", boost::bind(&RobotInterfaceServer::execute_moveTo, this, _1), false),
				m_as_drinkSubject(m_nh, "neurobots_demo/drink_subject", boost::bind(&RobotInterfaceServer::execute_drinkSubject, this, _1), false),
				m_as_dropObject(m_nh, "neurobots_demo/drop_object", boost::bind(&RobotInterfaceServer::execute_dropObject, this, _1), false),
				m_as_graspObject(m_nh, "neurobots_demo/grasp_object", boost::bind(&RobotInterfaceServer::execute_graspObject, this, _1), false),
				m_as_pourLiquid(m_nh, "neurobots_demo/pour_liquid", boost::bind(&RobotInterfaceServer::execute_pourLiquid, this, _1), false)
{
	ros::NodeHandle nh("neurobots_database");
	ros::NodeHandle nhGlobal;

	//Neurobots Database Service Clients
	m_clientGetObject = nh.serviceClient<database_msgs::get_object>("get_object");
	m_clientUpdateObject = nh.serviceClient<database_msgs::update_object>("update_object");
	m_clientGetOmnirobImages = nh.serviceClient<std_srvs::SetBool>("/omnirob_camera_bridge/send_images");
	m_clientCameraStreamInGUI = nh.serviceClient<planner_control_msgs::ShowTopic>("/goal_planner_gui/show_by_topic");

	toggleImageStream(true);

//	m_clientGetOmnirobImages.waitForExistence();

#ifdef ROBOT_INTERFACE_FOUND
//	m_birrtPlanner = NULL;
	m_clientOmnirobArm = nh.serviceClient<kuka_manager::JointState>("/omnirob_lbr/omnirob/cmd_joint_state");
#endif

	ROS_INFO("Init system...");

	if (!initSystem())
	{
		ROS_FATAL("Could not initialize RobotActionServer components!");
	}

#ifdef ROADMAP_PLANNER_FOUND
	ROS_INFO("Waiting for PRM planners...");

	{
		//default planner
		m_clientGetDataOmnirobDefault = nh.serviceClient<prm_planner_msgs::GetImage>("/prm_planner/default_omnirob/get_image");
		m_actionClientRoadmapPlannerDefault = new actionlib::SimpleActionClient<prm_planner_msgs::GoalAction>("/prm_planner/default_omnirob/goals",
				true);

		//check if connection is available
		if (!m_actionClientRoadmapPlannerDefault->waitForServer(ros::Duration(3)))
			ROS_ERROR("Something is wrong: Is the PRM Planner in the namespace /prm_planner/default_omnirob running?");
		else
			ROS_INFO("Connected to PRM Planner /prm_planner/default_omnirob");
	}

	{
		//default planner
		m_clientGetDataOmnirobDefaultYDown = nh.serviceClient<prm_planner_msgs::GetImage>("/prm_planner/default_omnirob_y_down/get_image");
		m_actionClientRoadmapPlannerDefaultYDown = new actionlib::SimpleActionClient<prm_planner_msgs::GoalAction>(
				"/prm_planner/default_omnirob_y_down/goals", true);

		//check if connection is available
		if (!m_actionClientRoadmapPlannerDefaultYDown->waitForServer(ros::Duration(3)))
			ROS_ERROR("Something is wrong: Is the PRM Planner in the namespace /prm_planner/default_omnirob_y_down running?");
		else
			ROS_INFO("Connected to PRM Planner /prm_planner/default_omnirob_y_down");
	}

	//pouring planner
	m_clientGetDataOmnirobPouring = nh.serviceClient<prm_planner_msgs::GetImage>("/prm_planner/omnirob_pouring/get_image");
	m_actionClientRoadmapPlannerPouring = new actionlib::SimpleActionClient<prm_planner_msgs::GoalAction>("/prm_planner/omnirob_pouring/goals", true);

	//check if connection is available
	if (!m_actionClientRoadmapPlannerPouring->waitForServer(ros::Duration(3)))
		ROS_ERROR("Something is wrong: Is the PRM Planner in the namespace /prm_planner/omnirob_pouring running?");
	else
		ROS_INFO("Connected to PRM Planner /prm_planner/omnirob_pouring");

	//drinking planner
	m_clientGetDataOmnirobDrinking = nh.serviceClient<prm_planner_msgs::GetImage>("/prm_planner/omnirob_drinking/get_image");
	m_clientSetObjectPoseTypeOmnirobDrinking = nh.serviceClient<prm_planner_msgs::SetObjectPoseType>(
			"/prm_planner/omnirob_drinking/set_object_pose_type");
	m_actionClientRoadmapPlannerDrinking = new actionlib::SimpleActionClient<prm_planner_msgs::GoalAction>("/prm_planner/omnirob_drinking/goals",
			true);

	//check if connection is available
	if (!m_actionClientRoadmapPlannerDrinking->waitForServer(ros::Duration(3)))
		ROS_ERROR("Something is wrong: Is the PRM Planner in the namespace /prm_planner/omnirob_drinking running?");
	else
		ROS_INFO("Connected to PRM Planner /prm_planner/omnirob_drinking");

	//stop planner
	m_stopRoadmapPlanner = false;
	m_serverStopRoadmapPlanner = nhGlobal.advertiseService("stop_roadmap_planner", &RobotInterfaceServer::stopRoadmapPlanner, this);
#endif

#ifdef BIRRT_PLANNER_FOUND
	m_client_set_planning_scene_info = nh.serviceClient<planner_msgs::planning_scene_info>("/planning_server/set_planning_scene_info");
	m_client_set_edge_cost_weights = nh.serviceClient<planner_msgs::set_edge_costs>("/planning_server/set_planner_edge_costs");
	m_client_run_planner_goal_pose = nh.serviceClient<planner_msgs::run_planner_map_goal_pose>("/planning_server/run_planner_map_goal_pose");

	//check if connection is available
	if (!m_client_set_planning_scene_info.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("m_client_set_planning_scene_info not available");
	}

	if (!m_client_set_edge_cost_weights.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("m_client_set_edge_cost_weights not available");
	}

	if (!m_client_run_planner_goal_pose.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("m_client_run_planner_goal_pose not available");
	}

	//Set planning scene info and edge cost weights for RRT* planner
	//Set planning scene info
	planner_msgs::planning_scene_info planning_info;
	planning_info.request.env_size_x.push_back(-4.0);
	planning_info.request.env_size_x.push_back(1.0);
	planning_info.request.env_size_y.push_back(-1.0);
	planning_info.request.env_size_y.push_back(4.0);
	planning_info.request.show_environment_borders = false;
	m_client_set_planning_scene_info.call(planning_info);
	//Set edge cost weights
	planner_msgs::set_edge_costs edge_costs;
	edge_costs.request.edge_costs.push_back(1.0);	//x
	edge_costs.request.edge_costs.push_back(1.0);	//y
	edge_costs.request.edge_costs.push_back(3.0);	//theta
	m_client_set_edge_cost_weights.call(edge_costs);

	ROS_INFO_STREAM("Found planner: birrt_star_motion_planning::BiRRTstarPlanner");
#else
	ROS_INFO_STREAM("Planner not found: birrt_star_motion_planning::BiRRTstarPlanner");
#endif

#ifdef BIRRT_TRAJECTORY_EXECUTION_FOUND
	//Set up execution service clients
	m_client_execute_trajectory_robot = nh.serviceClient<execution_msgs::execute_trajectory_robot>("/execution_server/execute_trajectory_robot");
	ROS_INFO("Found planner: trajectory_execution::MotionCommanderRobot");
#endif

	//Start action servers for Roadmap planner
	m_as_moveTo.start();
	m_as_drinkSubject.start();
	m_as_dropObject.start();
	m_as_graspObject.start();
	m_as_pourLiquid.start();

}

RobotInterfaceServer::~RobotInterfaceServer()
{
#ifdef ROADMAP_PLANNER_FOUND
	if (m_actionClientRoadmapPlannerDefault != NULL)
	{
		delete m_actionClientRoadmapPlannerDefault;
		m_actionClientRoadmapPlannerDefault = NULL;
	}

	if (m_actionClientRoadmapPlannerDefaultYDown != NULL)
	{
		delete m_actionClientRoadmapPlannerDefaultYDown;
		m_actionClientRoadmapPlannerDefaultYDown = NULL;
	}

	if (m_actionClientRoadmapPlannerPouring != NULL)
	{
		delete m_actionClientRoadmapPlannerPouring;
		m_actionClientRoadmapPlannerPouring = NULL;
	}

	if (m_actionClientRoadmapPlannerDrinking != NULL)
	{
		delete m_actionClientRoadmapPlannerDrinking;
		m_actionClientRoadmapPlannerDrinking = NULL;
	}
#endif
}

bool RobotInterfaceServer::initSystem()
{
	Object baseInitObject, armInitObject, transportEEFObject, facePrePosition;
	ros::Rate r(2);

	ROS_INFO("Get data from database...");

	while (!getDataFromDatabase("baseinit", baseInitObject))
	{
		r.sleep();
		ROS_WARN("Robot Action Server: Waiting to get baseinit parameter!");
	}

	while (!getDataFromDatabase("arminit", armInitObject))
	{
		r.sleep();
		ROS_WARN("Robot Action Server: Waiting to get arminit parameter!");
	}

	while (!getDataFromDatabase("poseeeftransport", transportEEFObject))
	{
		r.sleep();
		ROS_WARN("Robot Action Server: Waiting to get poseeeftransport parameter!");
	}

	while (!getDataFromDatabase("facepreposition", facePrePosition))
	{
		r.sleep();
		ROS_WARN("Robot Action Server: Waiting to get facepreposition parameter!");
	}

	m_transportEEFPose = CONVERT_TO_AFFINE(transportEEFObject["pose"]);
	m_facePrePosition = CONVERT_TO_AFFINE(facePrePosition["pose"]);

	ROS_INFO("Done.");

	//init arm
#ifdef ROBOT_INTERFACE_FOUND
	ROS_INFO("Init arm...");
	kuka_manager::JointState js;
	js.request.header.stamp = ros::Time::now();
	js.request.jointState.name = omnirobNames;
	js.request.jointState.position = CONVERT_TO_DOUBLE_STD_VECTOR(armInitObject["jointstate"]);
	if (!m_clientOmnirobArm.call(js))
	{
		ROS_INFO("An error occured while running the trajectory");
		return false;
	}
	ROS_INFO("Done.");
#endif

	return true;
}

void RobotInterfaceServer::execute_moveTo(const robot_interface_msgs::moveToGoalConstPtr &goal)
{
	// create messages that are used to published feedback/result
	robot_interface_msgs::moveToFeedback feedback;
	robot_interface_msgs::moveToResult result;
	bool success = false;

	ROS_INFO("Robot Interface: MoveTo received");

	feedback.state = "collect_data";
	m_as_moveTo.publishFeedback(feedback);

	if (goal->target_object == "shelfleft")
	{
		showImageInGui(SHELF_RIGHT_CAMERA);
	}
	else if (goal->target_object == "shelfright")
	{
		showImageInGui(SHELF_LEFT_CAMERA);
	}
	else
	{
		showImageInGui(FACE_CAMERA);
	}

	toggleImageStream(false);

	Object goalObject;
	bool data = getDataFromDatabase(goal->target_object, goalObject);

	Object omnirob;
	bool data2 = getDataFromDatabase("omnirob", omnirob);

	if (data && data2)
	{
		Eigen::Affine3d goalPose = CONVERT_TO_AFFINE(goalObject["pose"]);

		//ROS_INFO_STREAM(goalPose.matrix());

		feedback.state = "plan_motion";
		m_as_moveTo.publishFeedback(feedback);

		//Call Bidirectional RRT* Planner (Maintainer: Felix Burget)
		ROS_INFO_STREAM(goalPose.matrix());
#		ifdef BIRRT_PLANNER_FOUND
		ROS_INFO_STREAM("BiRRT Planner found...");
		planner_msgs::run_planner_map_goal_pose srv;
		//Set planner config
		srv.request.planner_type = "bi";
		srv.request.run_id = 0;
		//Set max planning time
		srv.request.flag_iter_or_time = 1;
		srv.request.iter_or_time = 30;	//in seconds
		//Set planning features
		srv.request.informed_sampling = true;
		srv.request.tree_optimization = true;
		//Show tree in RViz
		srv.request.show_tree = true;
		//Set goal Pose
		tf::poseEigenToMsg(goalPose, srv.request.goal_pose);

		//Set robot location in database
		Object object2;
		database_msgs::update_object srv3;
		object2["at"].push_back(std::string("nowhere"));
		database_conversions::convertObjectToMsg(object2, srv3.request.object);
		srv3.request.name = goal->robot;
		srv3.request.send_changes = true;
		m_clientUpdateObject.call(srv3);

		success = m_client_run_planner_goal_pose.call(srv) && srv.response.success;

		if (success)
		{
			execution_msgs::execute_trajectory_robot execute_trajectory;
			execute_trajectory.request.planner_type = "bi_informed_rrt_star";
			execute_trajectory.request.run_id = "0";
			//Execute trajectory
			success = m_client_execute_trajectory_robot.call(execute_trajectory) && execute_trajectory.response.execution_success;

			if (success)
			{
				//Set robot location in database
				Object object;
				database_msgs::update_object srv2;
				ROS_INFO_STREAM("Setting target pose to " << goal->target_object);
				object["at"].push_back(goal->target_object);
				database_conversions::convertObjectToMsg(object, srv2.request.object);
				srv2.request.name = goal->robot;
				srv2.request.send_changes = true;
				m_clientUpdateObject.call(srv2);
			}

		}
		//			else
		//			{
		//				//Set robot location in database
		//				database_msgs::update_object srv2;
		//				database_conversions::convertObjectToMsg(omnirob, srv2.request.object);
		//				srv2.request.name = goal->robot;
		//				srv2.request.send_changes = true;
		//				m_clientUpdateObject.call(srv2);
		//			}

		//			ROS_INFO_STREAM("BiRRT Planner quit...");
		//			if (success && srv.res.success)
		//			{
		//				feedback.state = "execute_motion";
		//				m_as_moveTo.publishFeedback(feedback);
		//
		////#				ifdef BIRRT_TRAJECTORY_EXECUTION_FOUND
		//				m_birrtPlanExecution->execute();
		//				feedback.state = "arrived at target location";
		//				m_as_moveTo.publishFeedback(feedback);
		//
		//				//Set robot location in database
		//				Object object;
		//				database_msgs::update_object srv;
		//				object["at"].push_back(goal->target_object);
		//				database_conversions::convertObjectToMsg(object, srv.request.object);
		//				srv.request.name = goal->robot;
		//				srv.request.send_changes = true;
		//				m_clientUpdateObject.call(srv);
		////
		////#				else
		////				feedback.state = "cannot execute motion, because BIRRT_TRAJECTORY is not available";
		////				m_as_moveTo.publishFeedback(feedback);
		////#				endif
		//			}
		//			else
		//			{
		//				feedback.state = "motion planning failed";
		//				m_as_moveTo.publishFeedback(feedback);
		//			}
#		else
		ROS_INFO_STREAM("BiRRT Planner not found!");
		success = false;
#		endif

	}
	else
	{
		feedback.state = "data for object not available";
		m_as_moveTo.publishFeedback(feedback);

		success = false;
	}

	showImageInGui(CONTROL_VIEW);

	result.arrived = success;

	// set the action state to succeeded
	m_as_moveTo.setSucceeded(result);
}

//DrinkSubject Action Callback
void RobotInterfaceServer::execute_drinkSubject(const robot_interface_msgs::drinkSubjectGoalConstPtr &goal)
{
	// create messages that are used to published feedback/result
	robot_interface_msgs::drinkSubjectFeedback feedback;
	robot_interface_msgs::drinkSubjectResult result;
	bool success = false;

	ROS_INFO("Robot Interface: DrinkSubject received");

	feedback.state = "collect_data";
	m_as_drinkSubject.publishFeedback(feedback);

	Object goalObject;
	bool data = getDataFromDatabase(goal->glass, goalObject);

	if (data)
	{
#ifdef ROADMAP_PLANNER_FOUND
		feedback.state = "plan_and_execute_pour";
		m_as_drinkSubject.publishFeedback(feedback);

		showImageInGui(OMNIROB_CAMERA);
		toggleImageStream(true);

		//Call PRM Planner (Maintainer: Daniel Kuhner)
		prm_planner_msgs::GetImage getImage;
		m_clientGetDataOmnirobDrinking.call(getImage);
		m_clientGetDataOmnirobDefaultYDown.call(getImage);

		//use the poses of the objects after next drinking
		prm_planner_msgs::SetObjectPoseType setObjectPoseType;
		setObjectPoseType.request.mode = prm_planner_msgs::SetObjectPoseType::Request::USE_CURRENT_POSE_WITHOUT_UPDATE;
		setObjectPoseType.request.neglectedObjects =
		{	goal->glass };

		success = m_clientSetObjectPoseTypeOmnirobDrinking.call(setObjectPoseType) && setObjectPoseType.response.success == 1;

		if (success)
		{
			success = prmPlannerSendGoal(m_facePrePosition, true);
			sleep(5);
		}

		if (success)
		{
			std::vector<std::string> parameters(3);

			success = false;
			ros::Rate r(0.5);
			int i = 0;
			while (!success && ros::ok() && i++ < 15)
			{
				//drink
				parameters[0] = "drink"; //command
				parameters[1] = goal->target_subject;
				parameters[2] = goal->glass;
				success = prmPlannerSendGoal(parameters, m_actionClientRoadmapPlannerDrinking);
				r.sleep();
			}

			if (success)
			{
				//wait
				feedback.state = "plan_and_execute_wait";
				m_as_drinkSubject.publishFeedback(feedback);
				sleep(2);

				//back
				parameters[0] = "back"; //command
				success = prmPlannerSendGoal(parameters, m_actionClientRoadmapPlannerDrinking);
				feedback.state = "plan_and_execute_back";
				m_as_drinkSubject.publishFeedback(feedback);

				if (success)
				{
					m_clientGetDataOmnirobDefault.call(getImage);
					success = prmPlannerSendGoal(m_transportEEFPose, true);
					feedback.state = "plan_and_execute_transport_position";
					m_as_drinkSubject.publishFeedback(feedback);

					if (success)
					{
						ROS_INFO_STREAM("called " << goal->glass);
						//Set 'contains' of cup in database
						Object object;
						database_msgs::update_object srv;
						object["contains"].push_back(std::string("empty"));
						database_conversions::convertObjectToMsg(object, srv.request.object);
						srv.request.name = goal->glass;
						srv.request.send_changes = true;
						m_clientUpdateObject.call(srv);
					}
				}
			}
			else
			{
				m_clientGetDataOmnirobDefault.call(getImage);
				prmPlannerSendGoal(m_transportEEFPose, true);
				feedback.state = "plan_and_execute_transport_position";
				m_as_drinkSubject.publishFeedback(feedback);
			}
		}

		setObjectPoseType.request.mode = prm_planner_msgs::SetObjectPoseType::Request::UPDATE_OBJECT_POSES;
		setObjectPoseType.request.neglectedObjects.clear();
		m_clientSetObjectPoseTypeOmnirobDrinking.call(setObjectPoseType);

		showImageInGui(CONTROL_VIEW);
		toggleImageStream(false);

#else
		feedback.state = "planner not available";
		m_as_drinkSubject.publishFeedback(feedback);
		result.arrived = success;
		m_as_drinkSubject.setSucceeded(result);
		toggleImageStream(false);
		return;
#endif
	}
	else
	{
		success = false;
	}

	result.arrived = success;

	// set the action state to succeeded
	m_as_drinkSubject.setSucceeded(result);
}

//Drop Object Action Callback
void RobotInterfaceServer::execute_dropObject(const robot_interface_msgs::dropObjectGoalConstPtr &goal)
{
	// create messages that are used to published feedback/result
	robot_interface_msgs::dropObjectFeedback feedback;
	robot_interface_msgs::dropObjectResult result;

	feedback.state = "collect_data";
	m_as_dropObject.publishFeedback(feedback);

	ROS_INFO("Robot Interface: DropObject received");

	Object goalObject;
	bool data = getDataFromDatabase(goal->target_location, goalObject);
	bool success = false;
	if (data)
	{
		//Call Elastic Roadmap Planner (Maintainer: Daniel Kuhner)
#ifdef ROADMAP_PLANNER_FOUND
		showImageInGui(OMNIROB_CAMERA);
		toggleImageStream(true);

		Eigen::Affine3d goalPose = CONVERT_TO_AFFINE(goalObject["pose"]);

		feedback.state = "plan_and_execute";
		m_as_dropObject.publishFeedback(feedback);

		goalPose.setIdentity();

		prm_planner_msgs::GetImage getImage;
		m_clientGetDataOmnirobDefault.call(getImage);
		success = prmPlannerSendDrop(goal->current_object, goalPose);

		//drive to transport pose
		if (success)
		{
			m_clientGetDataOmnirobDefaultYDown.call(getImage);
			success = prmPlannerSendGoal(m_transportEEFPose, true);
		}

		if (success)
		{
			//Set object location in database
			Object object;
			database_msgs::update_object srv;
			object["position"].push_back(goal->target_location);
			database_conversions::convertObjectToMsg(object, srv.request.object);
			srv.request.name = goal->current_object;
			srv.request.send_changes = true;
			m_clientUpdateObject.call(srv);

			//Set robot 'arm-empty' in database
			Object robot;
			database_msgs::update_object srv2;
			robot["arm-empty"].push_back(true);
			database_conversions::convertObjectToMsg(robot, srv2.request.object);
			srv2.request.name = goal->robot;
			srv2.request.send_changes = true;
			m_clientUpdateObject.call(srv2);
		}

		showImageInGui(CONTROL_VIEW);
		toggleImageStream(false);
#else
		feedback.state = "planner not available";
		m_as_dropObject.publishFeedback(feedback);
		result.placed = success;
		m_as_dropObject.setSucceeded(result);
		toggleImageStream(false);
		return;
#endif
	}

	result.placed = success;

	// set the action state to succeeded
	m_as_dropObject.setSucceeded(result);
}

//Grasp Object Action Callback
void RobotInterfaceServer::execute_graspObject(const robot_interface_msgs::graspObjectGoalConstPtr &goal)
{
	// create messages that are used to published feedback/result
	robot_interface_msgs::graspObjectFeedback feedback;
	robot_interface_msgs::graspObjectResult result;

	feedback.state = "collect_data";
	m_as_graspObject.publishFeedback(feedback);

	ROS_INFO("Robot Interface: GraspObject received");

	Object goalObject;
	bool data = getDataFromDatabase(goal->target_object, goalObject);
	bool success = false;
	if (data)
	{
		//Call Elastic Roadmap Planner (Maintainer: Daniel Kuhner)
#ifdef ROADMAP_PLANNER_FOUND
		Eigen::Affine3d goalPose = CONVERT_TO_AFFINE(goalObject["pose"]);

		showImageInGui(OMNIROB_CAMERA);
		toggleImageStream(true);

		feedback.state = "plan_and_execute";
		m_as_graspObject.publishFeedback(feedback);

		prm_planner_msgs::GetImage getImage;
		m_clientGetDataOmnirobDefaultYDown.call(getImage);
		success = prmPlannerSendGrasp(goal->target_object);

//		//drive to transport pose
//		if (success)
//		{
//			m_clientGetDataOmnirobDefault.call(getImage);
//			success = prmPlannerSendGoal(m_transportEEFPose);
//		}

		if (success)
		{
			//Set object location in database
			Object object;
			database_msgs::update_object srv;
			object["position"].push_back(goal->robot);
			database_conversions::convertObjectToMsg(object, srv.request.object);
			srv.request.name = goal->target_object;
			srv.request.send_changes = true;
			m_clientUpdateObject.call(srv);

			//Set robot 'arm-empty' in database
			Object robot;
			database_msgs::update_object srv2;
			robot["arm-empty"].push_back(false);
			database_conversions::convertObjectToMsg(robot, srv2.request.object);
			srv2.request.name = goal->robot;
			srv2.request.send_changes = true;
			m_clientUpdateObject.call(srv2);
		}

		showImageInGui(CONTROL_VIEW);
		toggleImageStream(false);

#else
		feedback.state = "planner not available";
		m_as_graspObject.publishFeedback(feedback);
		result.grasped = success;
		m_as_graspObject.setSucceeded(result);
		toggleImageStream(false);
		return;
#endif
	}

	result.grasped = success;

	// set the action state to succeeded
	m_as_graspObject.setSucceeded(result);
}

//Pour Liquid Action Callback
void RobotInterfaceServer::execute_pourLiquid(const robot_interface_msgs::pourLiquidGoalConstPtr &goal)
{
	// create messages that are used to published feedback/result
	robot_interface_msgs::pourLiquidFeedback feedback;
	robot_interface_msgs::pourLiquidResult result;
	bool success = false;

	ROS_INFO("Robot Interface: PourLiquid received");

	feedback.state = "collect_data";
	m_as_pourLiquid.publishFeedback(feedback);

	Object goalObject;
	bool data = getDataFromDatabase(goal->target_object, goalObject);

	if (data)
	{
#ifdef ROADMAP_PLANNER_FOUND
		feedback.state = "plan_and_execute_pour";
		m_as_pourLiquid.publishFeedback(feedback);

		showImageInGui(OMNIROB_CAMERA);
		toggleImageStream(true);

		//Call PRM Planner (Maintainer: Daniel Kuhner)
		prm_planner_msgs::GetImage getImage;
		m_clientGetDataOmnirobPouring.call(getImage);

		std::vector<std::string> parameters(3);

		//pour
		ROS_INFO_STREAM("sending pour signal");

		parameters[0] = "pour"; //command
		parameters[1] = goal->current_vessel;
		parameters[2] = goal->target_object;
		success = prmPlannerSendGoal(parameters, m_actionClientRoadmapPlannerPouring);

		ROS_INFO_STREAM("pour success: "<<success);

		if (success)
		{
			//wait
			feedback.state = "plan_and_execute_wait";
			m_as_pourLiquid.publishFeedback(feedback);
//			sleep(2);

			ROS_INFO_STREAM("sending back signal");

			//back
			parameters[0] = "back"; //command
			success = prmPlannerSendGoal(parameters, m_actionClientRoadmapPlannerPouring, false); //false -> do not react to stop
			feedback.state = "plan_and_execute_back";
			m_as_pourLiquid.publishFeedback(feedback);
		}

		if (success)
		{
			//Set 'contains' of cup in database
			Object object;
			database_msgs::update_object srv;
			object["contains"].push_back(goal->swapped_content);
			database_conversions::convertObjectToMsg(object, srv.request.object);
			srv.request.name = goal->target_object;
			srv.request.send_changes = true;
			m_clientUpdateObject.call(srv);

			//Set 'contains' of bottle in database
			Object robot;
			database_msgs::update_object srv2;
			robot["contains"].push_back(std::string("empty"));
			database_conversions::convertObjectToMsg(robot, srv2.request.object);
			srv2.request.name = goal->current_vessel;
			srv2.request.send_changes = true;
			m_clientUpdateObject.call(srv2);
		}

		showImageInGui(CONTROL_VIEW);
		toggleImageStream(false);

#else
		feedback.state = "planner not available";
		m_as_pourLiquid.publishFeedback(feedback);
		result.poured = success;
		m_as_pourLiquid.setSucceeded(result);
		toggleImageStream(false);
		return;
#endif
	}
	else
	{
		success = false;
	}

	result.poured = success;

	// set the action state to succeeded
	m_as_pourLiquid.setSucceeded(result);
}

//Get Object Data from the database server
bool RobotInterfaceServer::getDataFromDatabase(const std::string& name,
		Object& goalObject)
{
	//object
	database_msgs::get_object srvObject;
	srvObject.request.name = name;
	if (!m_clientGetObject.call(srvObject) || !srvObject.response.found)
	{
		ROS_WARN("There is no object %s", name.c_str());
		return false;
	}

	database_conversions::convertMsgToObject(srvObject.response.object, goalObject);

	return true;

}

bool RobotInterfaceServer::toggleImageStream(bool start)
{
	std_srvs::SetBool srv;
	srv.request.data = start;
	bool success = m_clientGetOmnirobImages.call(srv) && srv.response.success;
	if (start)
		sleep(3);
	return success;
}

bool RobotInterfaceServer::showImageInGui(const std::string& topic)
{
//	planner_control_msgs::ShowTopic srv;
//	srv.request.Topic = topic;
//	bool result = m_clientCameraStreamInGUI.call(srv);
//	ROS_INFO_STREAM("Result :" <<srv.response.Result);
//	return result && srv.response.Result == "Success";
	return true;
}

#ifdef ROADMAP_PLANNER_FOUND
bool RobotInterfaceServer::prmPlannerSendGoal(const Eigen::Affine3d& goal,
		bool yDown,
		bool reactToStop)
{
	m_stopRoadmapPlanner = false;
	prm_planner_msgs::GoalGoal goalMsg;
	tf::poseEigenToMsg(goal, goalMsg.goal);
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_MOVE;

	if (yDown)
	{
		m_actionClientRoadmapPlannerDefaultYDown->sendGoal(goalMsg);
		while (!m_actionClientRoadmapPlannerDefaultYDown->waitForResult(ros::Duration(0.1)))
		{
			if (reactToStop && m_stopRoadmapPlanner)
			{
				m_actionClientRoadmapPlannerDefaultYDown->cancelGoal();
				m_stopRoadmapPlanner = true;
			}
		}
		return m_actionClientRoadmapPlannerDefaultYDown->getResult()->success;
	}
	else
	{
		m_actionClientRoadmapPlannerDefault->sendGoal(goalMsg);
		while (!m_actionClientRoadmapPlannerDefault->waitForResult(ros::Duration(0.1)))
		{
			if (reactToStop && m_stopRoadmapPlanner)
			{
				m_actionClientRoadmapPlannerDefault->cancelGoal();
				m_stopRoadmapPlanner = true;
			}
		}
		return m_actionClientRoadmapPlannerDefault->getResult()->success;
	}
}

bool RobotInterfaceServer::prmPlannerSendGoal(const std::vector<std::string>& params,
		PRMActionClient* client,
		bool reactToStop)
{
	m_stopRoadmapPlanner = false;
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_CUSTOM;
	goalMsg.str = params;
	client->sendGoal(goalMsg);
	while (!client->waitForResult(ros::Duration(0.1)))
	{
		if (reactToStop && m_stopRoadmapPlanner)
		{
			ROS_INFO("Stop!");
			client->cancelGoal();
			m_stopRoadmapPlanner = true;
		}
	}
	return client->getResult()->success;
}

bool RobotInterfaceServer::prmPlannerSendDrop(const std::string& object,
		const Eigen::Affine3d& goal)
{
	m_stopRoadmapPlanner = false;
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.object_name = object;
	tf::poseEigenToMsg(goal, goalMsg.goal);
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_DROP;
	m_actionClientRoadmapPlannerDefault->sendGoal(goalMsg);
	while (!m_actionClientRoadmapPlannerDefault->waitForResult(ros::Duration(0.1)))
	{
		if (m_stopRoadmapPlanner)
		{
			m_actionClientRoadmapPlannerDefault->cancelGoal();
			m_stopRoadmapPlanner = true;
		}
	}
	return m_actionClientRoadmapPlannerDefault->getResult()->success;
}

bool RobotInterfaceServer::prmPlannerSendGrasp(const std::string& object)
{
	m_stopRoadmapPlanner = false;
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.object_name = object;
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_GRASP;
	m_actionClientRoadmapPlannerDefaultYDown->sendGoal(goalMsg);
	while (!m_actionClientRoadmapPlannerDefaultYDown->waitForResult(ros::Duration(0.1)))
	{
		if (m_stopRoadmapPlanner)
		{
			m_actionClientRoadmapPlannerDefaultYDown->cancelGoal();
			m_stopRoadmapPlanner = true;
		}
	}
	return m_actionClientRoadmapPlannerDefaultYDown->getResult()->success;
}

bool RobotInterfaceServer::stopRoadmapPlanner(std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res)
{
	ROS_INFO("Stop received!");
	m_stopRoadmapPlanner = true;
	return true;
}
#endif

} //end of namespace

