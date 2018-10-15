///*
// * demo.cpp
// *
// *  Created on: Oct 2, 2016
// *      Author: kuhnerd
// */
//
//#ifdef ROBOT_INTERFACE_FOUND
//#include <ros/ros.h>
//#include <robot_interface/robot_action_server.h>
//
//#include <database_conversions/database_conversions.h>
//#include <database_msgs/get_object.h>
//#include <database_msgs/update_object.h>
//#include <robot_interface/robot_action_server.h>
//#include <kuka_manager/JointState.h>
//#include <tf/transform_listener.h>
//#include <tf_conversions/tf_eigen.h>
//
//#ifdef ROADMAP_PLANNER_FOUND
//#include <prm_planner_interface/prm_planner_interface.h>
//#endif
//
//#ifdef BIRRT_PLANNER_FOUND
//#include <birrt_star_algorithm/birrt_star.h>
//#endif
//
//#ifdef BIRRT_TRAJECTORY_EXECUTION_FOUND
//#include <motion_trajectory_execution/trajectory_execution_robot.h>
//#endif
//
//tf::TransformListener* listener;
//
//robot_interface_definition::RobotInterface* m_roadmapPlannerGraspAndDropOmnirob;
//robot_interface_definition::RobotInterface* m_roadmapPlannerOmnirobPouring;
//
//robot_interface_definition::RobotInterface* m_birrtPlanner;
//robot_interface_definition::RobotInterface* m_birrtPlanExecution;
//
//Eigen::Affine3d getTf(const std::string frame1,
//		const std::string frame2)
//{
//	tf::StampedTransform transform;
//	try
//	{
//		listener->lookupTransform(frame1, frame2,
//				ros::Time(0), transform);
//	}
//	catch (tf::TransformException& ex)
//	{
//		ROS_ERROR("%s", ex.what());
//		ros::Duration(1.0).sleep();
//	}
//	Eigen::Affine3d pose;
//	tf::transformTFToEigen(transform, pose);
//	return pose;
//}
//
//Eigen::Affine3d getMat(std::vector<double> vec)
//{
//	Eigen::Affine3d mat;
//	ROS_INFO_STREAM(vec.size());
//	for (int i = 0; i < vec.size(); ++i)
//	{
//		mat.data()[i] = vec[i];
//		//ROS_INFO_STREAM(vec.data()[i]);
//	}
//
//	return mat;
//}
//
//int main(int argc,
//		char** argv)
//{
//	ros::init(argc, argv, "demo");
//	ros::NodeHandle n;
//
//	int start = 0;
//
//	if (argc <= 1)
//		start = 0;
//	else
//		start = atoi(argv[1]);
//
//	listener = new tf::TransformListener;
//
//	ros::AsyncSpinner spinner(0);
//	spinner.start();
//
//	//get omnirob joint state service
//	ros::ServiceClient omnirobSC = n.serviceClient<kuka_manager::JointState>("/omnirob_lbr/omnirob/cmd_joint_state");
//	omnirobSC.waitForExistence();
//
//	Eigen::Affine3d eef;
//
//	std::vector<double> initOmnirobPose;
//	n.getParam("/setup/init_base", initOmnirobPose);
//
//	std::vector<double> shelfRightOmnirobPose;
//	n.getParam("/setup/base_shelf_right", shelfRightOmnirobPose);
//
//	std::vector<double> shelfLeftOmnirobPose;
//	n.getParam("/setup/base_shelf_left", shelfLeftOmnirobPose);
//
//	std::vector<double> tableOmnirobPose;
//	n.getParam("/setup/base_table", tableOmnirobPose);
//
//	std::vector<double> initOmnirobArmPose;
//	n.getParam("setup/init_arm", initOmnirobArmPose);
//
//	std::vector<double> poseEefTransport;
//	n.getParam("setup/pose_eef_transport", poseEefTransport);
//
//	std::vector<std::string> omnirobName = { "lbr_1_link", "lbr_2_link", "lbr_3_link", "lbr_4_link", "lbr_5_link", "lbr_6_link", "lbr_7_link" };
//
////#ifdef ROADMAP_PLANNER_FOUND
////	m_roadmapPlannerGraspAndDropIiwa = new prm_planner::PRMPlannerInterface("/prm_planner/iiwa_default");
////	m_roadmapPlannerGraspAndDropOmnirob = new prm_planner::PRMPlannerInterface("/prm_planner/default_omnirob");
////	m_roadmapPlannerOmnirobPouring = new prm_planner::PRMPlannerInterface("/prm_planner/omnirob_pouring");
//////	m_roadmapPlannerIiwaDrinking = new prm_planner::PRMPlannerInterface("/prm_planner/iiwa_drinking");
////
////	ROS_INFO("Found planner: prm_planner::RoadmapPlanner");
////#endif
//
//#ifdef BIRRT_PLANNER_FOUND
//	m_birrtPlanner = new birrt_star_motion_planning::BiRRTstarPlanner("omnirob_base");
//	ROS_INFO("Found planner: birrt_star_motion_planning::BiRRTstarPlanner");
//#endif
//
//#ifdef BIRRT_TRAJECTORY_EXECUTION_FOUND
//	m_birrtPlanExecution = new trajectory_execution::MotionCommanderRobot("omnirob_base");
//	ROS_INFO("Found planner: trajectory_execution::MotionCommanderRobot");
//#endif
//
////	Eigen::AngleAxisd rot(M_PI / 2, Eigen::Vector3d::UnitX());
////
////	Eigen::Quaterniond q(rot);
////	ROS_INFO(" %f %f %f %f", q.x(), q.y(), q.z(), q.w());
////	exit(0);
//
//////	init omnirob arm
////	{
////		ROS_INFO("init arm");
////		kuka_manager::JointState js;
////		//		js.request.jointState.header.stamp = ros::Time::now();
////		//		js.request.jointState.name = omnirobName;
////		//		//js.request.jointState.position = initOmnirobArmPose;
////		//		js.request.jointState.position.swap(initOmnirobArmPose);
////		//		js.request.jointState.velocity = {0,0,0,0,0,0,0};
////		//		ROS_INFO_STREAM(js.request.jointState.position[0] << " " << js.request.jointState.position[1] << " " << js.request.jointState.position[2]);
////		//		ROS_INFO_STREAM(js.request.jointState.name[0] << " " << js.request.jointState.name[1] << " " << js.request.jointState.name[2]);
////
////		js.request.header.stamp = ros::Time::now();
////		js.request.jointState.name =
////		{	"iiwa_1_joint", "iiwa_2_joint", "iiwa_3_joint", "iiwa_4_joint", "iiwa_5_joint", "iiwa_6_joint", "iiwa_7_joint"};
////		js.request.jointState.position =
////		{	initIiwaArmPose[0], initIiwaArmPose[1], initIiwaArmPose[2], initIiwaArmPose[3], initIiwaArmPose[4], initIiwaArmPose[5], initIiwaArmPose[6]};
////
////		if (iiwaSC.call(js))
////		{
////			ROS_INFO("Trajectory executed: ");
////		}
////		else
////		{
////			ROS_INFO("An error occured while running the trajectory");
////		}
////	}
////	return 0;
//
//	if (start == 0)
//	{
//		ROS_INFO("init arm");
//		kuka_manager::JointState js;
//
//		js.request.header.stamp = ros::Time::now();
//		js.request.jointState.name =
//		{	"lbr_1_joint", "lbr_2_joint", "lbr_3_joint", "lbr_4_joint", "lbr_5_joint", "lbr_6_joint", "lbr_7_joint"};
//		js.request.jointState.position =
//		{	initOmnirobArmPose[0], initOmnirobArmPose[1], initOmnirobArmPose[2], initOmnirobArmPose[3], initOmnirobArmPose[4], initOmnirobArmPose[5], initOmnirobArmPose[6]};
//
//		if (!omnirobSC.call(js))
//		{
//			ROS_INFO("An error occured while running the trajectory");
//			return -1;
//		}
//
//		++start;
//	}
//
//	//init omnirob
//	if (start == 1)
//	{
//		ROS_INFO("init");
//		Eigen::Affine3d pose = getMat(initOmnirobPose);
//		ROS_INFO_STREAM(pose.matrix());
//		if (!m_birrtPlanner->plan(pose))
//		{
//			ROS_ERROR("Cannot find a plan to the init pose");
//			return -1;
//		}
//		if (!m_birrtPlanExecution->execute())
//		{
//			ROS_ERROR("Cannot execute the plan to the init pose");
//			return -1;
//		}
//
//		++start;
//	}
//
//	//omnirob to shelf
//	if (start == 2)
//	{
//		ROS_INFO("omnirob to shelf left");
//		Eigen::Affine3d pose = getMat(shelfLeftOmnirobPose);
////		m_roadmapPlannerGraspAndDropOmnirob->activate(); //activate early
//		if (!m_birrtPlanner->plan(pose))
//		{
//			ROS_ERROR("Cannot find a plan to the shelf left pose");
//			return -1;
//		}
//		if (!m_birrtPlanExecution->execute())
//		{
//			ROS_ERROR("Cannot execute the plan to the shelf left pose");
//			return -1;
//		}
//
//		++start;
//	}
//
//	//omnirob to shelf
//	if (start == 3)
//	{
//		ROS_INFO("omnirob to shelf right");
//		Eigen::Affine3d pose = getMat(shelfRightOmnirobPose);
////		m_roadmapPlannerGraspAndDropOmnirob->activate(); //activate early
//		if (!m_birrtPlanner->plan(pose))
//		{
//			ROS_ERROR("Cannot find a plan to the shelf right pose");
//			return -1;
//		}
//		if (!m_birrtPlanExecution->execute())
//		{
//			ROS_ERROR("Cannot execute the plan to the shelf right pose");
//			return -1;
//		}
//
//		++start;
//	}
//
////	//grasp bottle
////	if (start == 3)
////	{
////		ROS_INFO("grasp bottle");
////		m_roadmapPlannerGraspAndDropOmnirob->activate();
////		sleep(4);
////		ROS_INFO("called1");
////		if (!m_roadmapPlannerGraspAndDropOmnirob->grasp("water_bottle"))
////		{
////			ROS_ERROR("Cannot grasp the bottle in the shelf");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//up
////	if (start == 4)
////	{
////		ROS_INFO("up");
////		m_roadmapPlannerGraspAndDropOmnirob->activate();
////		eef = getTf("/omnirob_lbr/lbr_0_link", "/omnirob_lbr/wsg_gripper_tip_frame");
////		eef.translation().z() += 0.05;
////		ROS_INFO_STREAM(eef.matrix());
////
////		if (!m_roadmapPlannerGraspAndDropOmnirob->plan(eef))
////		{
////			ROS_ERROR("Cannot find a plan to the up position");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//omnirob to shelf2
////	if (start == 5)
////	{
////		ROS_INFO("omnirob to shelf2");
////		m_roadmapPlannerGraspAndDropOmnirob->activate();
////		Eigen::Affine3d pose = getMat(shelf2OmnirobPose);
////		if (!m_birrtPlanner->plan(pose))
////		{
////			ROS_ERROR("Cannot find a plan to the shelf2 pose");
////			return -1;
////		}
////		if (!m_birrtPlanExecution->execute())
////		{
////			ROS_ERROR("Cannot execute the plan to the shelf2 pose");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//transport position
////	if (start == 6)
////	{
////		ROS_INFO("transport position");
////		m_roadmapPlannerGraspAndDropOmnirob->activate();
////		eef = getTf("/omnirob_lbr/lbr_0_link", "/omnirob_lbr/wsg_gripper_tip_frame");
////		Eigen::Affine3d pose = getMat(poseEefTransport);
////		//		pose.linear() = eef.linear();
////		pose.translation().z() -= 0.05;
////		ROS_INFO_STREAM(pose.matrix());
////		if (!m_roadmapPlannerGraspAndDropOmnirob->plan(pose))
////		{
////			ROS_ERROR("Cannot find a plan to the transport position");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//omnirob to table 1
////	if (start == 7)
////	{
////		ROS_INFO("omnirob to table 1");
////		Eigen::Affine3d pose = getMat(table1OmnirobPose);
////		if (!m_birrtPlanner->plan(pose))
////		{
////			ROS_ERROR("Cannot find a plan to the table 1 pose");
////			return -1;
////		}
////		if (!m_birrtPlanExecution->execute())
////		{
////			ROS_ERROR("Cannot execute the plan to the table 1 pose");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//table 1 to table
////	if (start == 8)
////	{
////		ROS_INFO("omnirob to table");
////		Eigen::Affine3d pose = getMat(tableOmnirobPose);
////		m_roadmapPlannerGraspAndDropOmnirob->deactivate();
//////		m_roadmapPlannerOmnirobPouring->activate(); //activate early
////		if (!m_birrtPlanner->plan(pose))
////		{
////			ROS_ERROR("Cannot find a plan to the table pose");
////			return -1;
////		}
////		if (!m_birrtPlanExecution->execute())
////		{
////			ROS_ERROR("Cannot execute the plan to the table pose");
////			return -1;
////		}
////
////		++start;
//////		return 0;
////	}
////
////	//pour
////	if (start == 9)
////	{
////		ROS_INFO("pour");
////		m_roadmapPlannerOmnirobPouring->activate();
////		ros::Rate r(10);
////		while (!m_roadmapPlannerOmnirobPouring->plan("pour", "") && ros::ok())
////		{
////			ROS_ERROR("Cannot pour. Try again...");
////			r.sleep();
////		}
////
////		++start;
////	}
////
////	//pour
////	if (start == 10)
////	{
////		sleep(5);
////		ROS_INFO("back");
////		m_roadmapPlannerOmnirobPouring->activate();
////		sleep(2);
////		if (!m_roadmapPlannerOmnirobPouring->plan("back", ""))
////		{
////			ROS_ERROR("Cannot back");
////			return -1;
////		}
////
////		++start;
////	}
////
////	if (start == 11)
////	{
////		ROS_INFO("omnirob to table 1");
////		m_roadmapPlannerOmnirobPouring->deactivate();
////		Eigen::Affine3d pose = getMat(table1OmnirobPose);
////		if (!m_birrtPlanner->plan(pose))
////		{
////			ROS_ERROR("Cannot find a plan to the table 1 pose");
////			return -1;
////		}
////		if (!m_birrtPlanExecution->execute())
////		{
////			ROS_ERROR("Cannot execute the plan to the table 1 pose");
////			return -1;
////		}
////
////		++start;
////	}
////
////	if (start == 12)
////	{
////		ROS_INFO("init iiwa");
////		kuka_manager::JointState js;
////
////		js.request.header.stamp = ros::Time::now();
////		js.request.jointState.name =
////		{	"iiwa_1_joint", "iiwa_2_joint", "iiwa_3_joint", "iiwa_4_joint", "iiwa_5_joint", "iiwa_6_joint", "iiwa_7_joint"};
////		js.request.jointState.position =
////		{	jsIiwa[0], jsIiwa[1], jsIiwa[2], jsIiwa[3], jsIiwa[4], jsIiwa[5], jsIiwa[6]};
////
////		if (!iiwaSC.call(js))
////		{
////			ROS_INFO("An error occured while running the trajectory");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//grasp cup
////	if (start == 13)
////	{
////		ROS_INFO("grasp cup");
////		m_roadmapPlannerGraspAndDropIiwa->activate();
////		sleep(4);
////		if (!m_roadmapPlannerGraspAndDropIiwa->grasp("cup_letters"))
////		{
////			ROS_ERROR("Cannot grasp the cup on the table");
////			return -1;
////		}
////
////		++start;
////	}
////
////	//up
////	if (start == 14)
////	{
////		ROS_INFO("up");
////		m_roadmapPlannerGraspAndDropIiwa->activate();
////		eef = getTf("/iiwa/iiwa_0_link", "/iiwa/sdh2_grasp_link");
////		eef.translation().z() += 0.05;
////		ROS_INFO_STREAM(eef.matrix());
////
////		if (!m_roadmapPlannerGraspAndDropIiwa->plan(eef))
////		{
////			ROS_ERROR("Cannot find a plan to the up position");
////			return -1;
////		}
////
////		++start;
////	}
////
////	if (start == 15)
////	{
////		ROS_INFO("up");
////		m_roadmapPlannerGraspAndDropIiwa->activate();
////		Eigen::Affine3d pose = getMat(iiwaInitDrinking);
////
////		if (!m_roadmapPlannerGraspAndDropIiwa->plan(pose))
////		{
////			ROS_ERROR("Cannot find a plan to the up position");
////			return -1;
////		}
////
////		++start;
////	}
//
////	m_roadmapPlannerGraspAndDropOmnirob->deactivate();
////
////	int a;
////	std::cin >> a;
////	//omnirob to table
////	{
////		ROS_INFO("omnirob to table");
////		Eigen::Affine3d pose = getMat(tableOmnirobPose);
////		m_birrtPlanner->plan(pose);
////		m_birrtPlanExecution->execute();
////	}
//
//	ros::waitForShutdown();
//
//	return 0;
//}
//#else
//int main(int argc, char** argv)
//{
//	return 0;
//}
//#endif
//
