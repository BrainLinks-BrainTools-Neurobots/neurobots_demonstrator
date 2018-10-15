/*
 * KukaManager.h
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */

#ifndef KUKAMANAGER_H_
#define KUKAMANAGER_H_

#include <ros/ros.h>
#include <boost/thread.hpp>

// Controller
#include "kuka_manager/UDPController.h"
#include "kuka_manager/UDPPlatformController.h"
#include "kuka_manager/OmnirobPublisher.h"
#include "kuka_manager/UDPPublisher.h"

#include "kuka_manager/kuka_robots.h"

// Movit
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class KukaManager {
public:
	KukaManager();
	virtual ~KukaManager();

	void init(const char* host, ros::NodeHandle &nh, KUKA::Robotname robot);
	void initFollowJointStateActionServer(std::string actionserverName, std::string actionserverName2);
	void startThreads();
	void spin();
	void readPortConfig(std::string filename);

private:
	KUKA::Robotname robot;

	std::unordered_map<std::string, int> portMap;

	// Ros stuff
	ros::NodeHandle nh;

	// Controller
	UDPController udpOmnirobController;
	UDPPublisher udpOmnirobPublisher;
	UDPController udpIIWAController;
	UDPPublisher udpIIWAPublisher;
	UDPPlatformController platformController;
	OmnirobPublisher omnirobPublisher;

	// Threads
	boost::thread* udpOmnirobControllerThread;
	boost::thread* udpOmnirobPublisherThread;
	boost::thread* udpIIWAControllerThread;
	boost::thread* udpIIWAPublisherThread;
	boost::thread* platformControllerThread;
	boost::thread* omnirobPublisherThread;

	// Movit actionserver
	std::string actionNameOmnirob;
	std::string actionNameIIWA;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServerOmnirob;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServerIIWA;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServerAsyncOmnirob;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServerAsyncIIWA;
	control_msgs::FollowJointTrajectoryFeedback _feedback;
	control_msgs::FollowJointTrajectoryResult _result;

	void callbackOmnirob(const control_msgs::FollowJointTrajectoryGoalConstPtr& jointState);
	void callbackOmnirobPreempt();
	void callbackOmnirobAsync(const control_msgs::FollowJointTrajectoryGoalConstPtr& jointState);
	void callbackIIWA(const control_msgs::FollowJointTrajectoryGoalConstPtr& jointState);
	void callbackIIWAAsync(const control_msgs::FollowJointTrajectoryGoalConstPtr& jointState);
};

#endif /* KUKAMANAGER_H_ */
