/*
 * MoveItController.h
 *
 *  Created on: Nov 20, 2015
 *      Author: Ingo Killmann
 */

#ifndef MOVEITCONTROLLER_H_
#define MOVEITCONTROLLER_H_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <eigen_conversions/eigen_msg.h>
#include <sdh2_hand/SDHAction.h>
#include <wsg_gripper/GripperCommandGoal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map>

 #include <moveit/robot_model_loader/robot_model_loader.h>

enum HandType {NONE, SCHUNK_HAND, WSG_GRIPPER};

class MoveItController {
public:
	MoveItController(std::string robot_name, HandType hand=NONE);
	virtual ~MoveItController();

	void init(ros::NodeHandle& nh);
	bool goToPosition(Eigen::Vector3f& position, Eigen::Matrix3f& orientation);
	bool goToPosition(Eigen::Affine3d& position);
	bool goToPositions(EigenSTL::vector_Affine3d& positions);
	double goToPositionWithFlexAngle(Eigen::Vector3f& position, Eigen::Matrix3f& orientation,
			double lengthHand, double rad, double angle);
	void goHome();
	void openGripper(double speed=0.2);
	void closeGripper(double speed=0.2, double closing_mm=77);
	bool goStraight(Eigen::Vector3f& start, Eigen::Vector3f& target, Eigen::Matrix3f& orientation,
			int num_waypoints, Eigen::Vector3f& recover);
	bool goCircular(Eigen::Vector3f& position, double xOffset, double zOffset, double angleBetweenPoints,
			int num_waypoints, Eigen::Vector3f& recover, Eigen::Matrix3f& orientation, bool reverse=false);
	void saveLastPlan(std::string name);
	void concatPlans(std::string plan1, std::string plan2, std::string combined_plan);
	void reversePlan(std::string plan, std::string save_name);
	bool checkPlan(std::string plan);
	void executePlan(std::string plan, Eigen::Vector3f& recover, Eigen::Matrix3f& orientation);
	void deletePlan(std::string plan);
	void eraseAllPlans();
	Eigen::Vector3f getCurrentPosition();
	bool checkReachability(Eigen::Affine3d& position);
	bool cartesianToJointPosition(Eigen::Affine3d& robotPoses, sensor_msgs::JointState& jointPoses);
	void cartesianToJointPositions(std::vector<Eigen::Affine3d>& robotPoses,
			std::vector<sensor_msgs::JointState>& jointPoses);



	/**
	 * Getter for ROS NodeHandle
	 */
	ros::NodeHandle& getNodeHandle() {
		return nh;
	}

	moveit::planning_interface::MoveGroup& getGroup() {
		return group;
	}

private:
	//move_it
	moveit::planning_interface::MoveGroup group;
	std::string robot_name;
	std::string name_prefix;
	//ros
	ros::NodeHandle nh;
	ros::ServiceClient hand_client;
	ros::Publisher pub_finish;
	ros::Publisher wsg_pub;
	//hand
	HandType hand;

	//Constant variables
	static const int PARALLEL = 0;   //For Schunk hand
	static const int STOPFINGER = 2; //For Schunk hand
	static const int NOSTOP = 0;	  //For Schunk hand
	static const std::string FINISHED_ERROR;       //message, when movement did not work
	static const std::string FINISHED_SUCCESS;  //message, when movement was successfully

	//plans
	moveit::planning_interface::MoveGroup::Plan lastPlan; //The last plan, that was calculated, always gets saved
	std::map<std::string,moveit::planning_interface::MoveGroup::Plan> savedPlans; //Hashmap of all saved plans

	void spin();
	void pub_finished(std::string finish);
	void computeWaypointsStraight(Eigen::Vector3f& start, Eigen::Vector3f& target,
			Eigen::Matrix3f& orientation, int num_waypoints, std::vector<Eigen::Affine3d>& waypoints);

	// new demo 2016
	void computeWaypointCircular(Eigen::Vector3f centerOfCup, const double &cupRadius, 
 	        const double &cupHalfHeight, const double &angle,
 	        const int numOfWaypoints, std::vector<Eigen::Affine3d> &waypoints);
	
	void computeWaypointCircularReverse(Eigen::Vector3f& start, int num_waypoints,
			std::vector<Eigen::Affine3d>& waypoints);
	void createRotTransMatrix(Eigen::Vector3f& position, Eigen::Matrix3f& orientation,
			Eigen::Matrix4d& matrix);
	int calculatePlan(std::vector<Eigen::Affine3d>& waypoints,
			moveit::planning_interface::MoveGroup::Plan& plan);
	int executePlan(moveit::planning_interface::MoveGroup::Plan& plan);
	void makePlanOfTrajectory(robot_trajectory::RobotTrajectory& trajectory,
			moveit::planning_interface::MoveGroup::Plan& plan);
};

#endif /* MOVEITCONTROLLER_H_ */
