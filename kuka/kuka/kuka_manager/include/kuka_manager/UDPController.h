/*
 * UDPController.h
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */

#ifndef UDPCONTROLLER_H_
#define UDPCONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "kuka_manager/UDPHandle.h"
#include "kuka_manager/Speed.h"
#include "kuka_manager/JointState.h"
#include <kuka_manager/SpeedAcceleration.h>
#include "kuka_manager/kuka_robots.h"

#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <unordered_map>

class UDPController
{
public:
	UDPController();
	virtual ~UDPController();

	// Thread stuff
	void start(ros::NodeHandle& nh,
			const char* host,
			int port,
			KUKA::Robotname robot);
	void runThread();

	// Ros stuff
	void commandJointStateCallback(const sensor_msgs::JointState::ConstPtr &jointState);
	bool commandJointStateCallback(kuka_manager::JointState::Request &req,
			kuka_manager::JointState::Response &res);

	void commandJointStateAsyncCallback(const sensor_msgs::JointState::ConstPtr &jointState);
	bool commandJointStateAsyncCallback(kuka_manager::JointState::Request &req,
			kuka_manager::JointState::Response &res);

	void commandCartesianCallback(const geometry_msgs::Transform::ConstPtr &cartesian);
	bool commandCartesianCallback(kuka_manager::JointState::Request &req,
			kuka_manager::JointState::Response &res);
	void commandCartesianAsyncCallback(const geometry_msgs::Transform::ConstPtr &cartesian);
	void changeVelAccCallback(const kuka_manager::Speed::ConstPtr &speed);
	bool changeVelAccCallbackService(kuka_manager::SpeedAcceleration::Request &req,
			kuka_manager::SpeedAcceleration::Response &res);

	// Help functions
	bool moveRobotArmWithJointState(double timestamp,
			int counter,
			std::unordered_map<std::string, double> jointState);
	bool moveRobotArmWithJointStateAsync(double timestamp,
			int counter,
			std::unordered_map<std::string, double> jointState);
	bool moveRobotArmWithCartesian(double timestamp,
			int counter,
			double x,
			double y,
			double z,
			double alpha,
			double beta,
			double gamma);
	bool moveRobotArmWithCartesianAsync(double timestamp,
			int counter,
			double x,
			double y,
			double z,
			double alpha,
			double beta,
			double gamma);

	bool sendTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory,
			actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServer);

	bool sendTrajectoryAsync(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory,
			actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServer);

	void sendRealTimeTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory);

	void stopArm(const std_msgs::Bool::ConstPtr& tmp);
	void stopArm();

private:
	// Extracts the ids of the needed joints
	void getJointMap(const std::vector<std::string>& names,
			const std::vector<double>& joints,
			std::unordered_map<std::string, double>& ids);

private:
	KUKA::Robotname robot;

	// Parameter
	int jointCounter;
	int cartesianCounter;

	// UDP stuff
	UDPHandle udpSender;

	// Ros objects
	ros::NodeHandle nh;
	ros::Subscriber commandJointStateSub;
	ros::ServiceServer commandJointStateService;

	ros::Subscriber commandJointStateAsyncSub;
	ros::ServiceServer commandJointStateAsyncService;

	ros::Publisher commandJointStateFinishedPub;
	ros::Subscriber commandRealTimeTrajectorySub;

	ros::Subscriber commandCartesianSub;
	ros::ServiceServer commandCartesianService;
	ros::Subscriber commandCartesianAsyncSub;
	ros::Publisher commandCartesianFinishedPub;
	ros::Subscriber changeVelAccCommand;
	ros::ServiceServer changeVelAccService;

	ros::Subscriber commandStopArm;
};

#endif /* UDPCONTROLLER_H_ */
