/*
 * KukaManager.cpp
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */

#include "kuka_manager/KukaManager.h"
#include "kuka_manager/UDPPublisher.h"
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>

KukaManager::KukaManager() :
				robot(KUKA::Robotname::None),
				udpOmnirobControllerThread(NULL),
				udpOmnirobPublisherThread(NULL),
				udpIIWAControllerThread(NULL),
				udpIIWAPublisherThread(NULL),
				platformControllerThread(NULL),
				omnirobPublisherThread(NULL)
{
	actionServerOmnirob = NULL;
	actionServerIIWA = NULL;
	actionServerAsyncOmnirob = NULL;
	actionServerAsyncIIWA = NULL;

	std::string filepath = ros::package::getPath("kuka_manager");
	filepath += "/config.txt";
	readPortConfig(filepath);
}

KukaManager::~KukaManager()
{
	if (actionServerOmnirob != NULL)
	{
		delete actionServerOmnirob;
	}
	if (actionServerIIWA != NULL)
	{
		delete actionServerIIWA;
	}
	if (actionServerAsyncOmnirob != NULL)
	{
		delete actionServerAsyncOmnirob;
	}
	if (actionServerAsyncIIWA != NULL)
	{
		delete actionServerAsyncIIWA;
	}

	// Kill all threads
	udpOmnirobControllerThread->interrupt();
	udpOmnirobPublisherThread->interrupt();
	udpIIWAControllerThread->interrupt();
	udpIIWAPublisherThread->interrupt();
	platformControllerThread->interrupt();
	omnirobPublisherThread->interrupt();
}

void KukaManager::init(const char* host,
		ros::NodeHandle &nh,
		KUKA::Robotname robot)
{
	this->robot = robot;

	// Ros
	this->nh = nh;

	if (robot == KUKA::Robotname::Omnirob) {
		udpOmnirobController.start(nh, host, portMap["OmnirobControllerPort"], robot);
		udpOmnirobPublisher.start(nh, host, portMap["OmnirobPublisherPort"],
				portMap["OmnirobReceiverPort"], robot);
		platformController.start(nh, host, portMap["OmnirobPlatformPublisherPort"]);
		omnirobPublisher.start(nh);
		initFollowJointStateActionServer("omnirob_lbr/follow_joint_trajectory", "");
	}
	else if (robot == KUKA::Robotname::IIWA) {
		udpIIWAController.start(nh, host, portMap["IIWAControllerPort"], robot);
		udpIIWAPublisher.start(nh, host, portMap["IIWAPublisherPort"],
				portMap["IIWAReceiverPort"], robot);
		initFollowJointStateActionServer("iiwa/follow_joint_trajectory", "");
	}
	else if (robot == KUKA::Robotname::Both) {
		udpOmnirobController.start(nh, host, portMap["OmnirobControllerPort"], KUKA::Robotname::Omnirob);
		udpOmnirobPublisher.start(nh, host, portMap["OmnirobPublisherPort"],
				portMap["OmnirobReceiverPort"], KUKA::Robotname::Omnirob);
		platformController.start(nh, host, portMap["OmnirobPlatformPublisherPort"]);
		omnirobPublisher.start(nh);

		udpIIWAController.start(nh, host, portMap["IIWAControllerPort"], KUKA::Robotname::IIWA);
		udpIIWAPublisher.start(nh, host, portMap["IIWAPublisherPort"],
				portMap["IIWAReceiverPort"], KUKA::Robotname::IIWA);
		initFollowJointStateActionServer("omnirob_lbr/follow_joint_trajectory", "iiwa/follow_joint_trajectory");
	}
}

void KukaManager::initFollowJointStateActionServer(std::string actionServerName, std::string actionServerName2)
{
	if (robot == KUKA::Robotname::Omnirob) {
		actionNameOmnirob = actionServerName;
		if (actionServerOmnirob != NULL)
		{
			delete actionServerOmnirob;
		}
		actionServerOmnirob = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameOmnirob, boost::bind(&KukaManager::callbackOmnirob, this, _1), true);
		actionServerOmnirob->registerPreemptCallback(boost::bind(&KukaManager::callbackOmnirobPreempt, this));
		if (actionServerAsyncOmnirob != NULL)
		{
			delete actionServerAsyncOmnirob;
		}
		std::stringstream str;
		str << actionNameOmnirob << "_async";
		std::string actionNameAsync = str.str();
		actionServerAsyncOmnirob = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameAsync, boost::bind(&KukaManager::callbackOmnirobAsync, this, _1), true);
		actionServerAsyncOmnirob->registerPreemptCallback(boost::bind(&KukaManager::callbackOmnirobPreempt, this));
	}
	else if (robot == KUKA::Robotname::IIWA) {
		actionNameIIWA = actionServerName;
		if (actionServerIIWA != NULL)
		{
			delete actionServerIIWA;
		}
		actionServerIIWA = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameIIWA, boost::bind(&KukaManager::callbackIIWA, this, _1), true);
		if (actionServerAsyncIIWA != NULL)
		{
			delete actionServerAsyncIIWA;
		}
		std::stringstream str;
		str << actionNameIIWA << "_async";
		std::string actionNameAsync = str.str();
		actionServerAsyncIIWA = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameAsync, boost::bind(&KukaManager::callbackIIWAAsync, this, _1), true);
	}
	else if (robot == KUKA::Robotname::Both) {
		actionNameOmnirob = actionServerName;
		if (actionServerOmnirob != NULL)
		{
			delete actionServerOmnirob;
		}
		actionServerOmnirob = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameOmnirob, boost::bind(&KukaManager::callbackOmnirob, this, _1), true);

		actionNameIIWA = actionServerName2;
		if (actionServerIIWA != NULL)
		{
			delete actionServerIIWA;
		}
		actionServerIIWA = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameIIWA, boost::bind(&KukaManager::callbackIIWA, this, _1), true);
		if (actionServerAsyncOmnirob != NULL)
		{
			delete actionServerAsyncOmnirob;
		}
		std::stringstream str;
		str << actionNameOmnirob << "_async";
		std::string actionNameAsync = str.str();
		actionServerAsyncOmnirob = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameAsync, boost::bind(&KukaManager::callbackOmnirobAsync, this, _1), true);
		if (actionServerAsyncIIWA != NULL)
		{
			delete actionServerAsyncIIWA;
		}
		std::stringstream strIIWA;
		strIIWA << actionNameIIWA << "_async";
		std::string actionNameAsyncIIWA = strIIWA.str();
		actionServerAsyncIIWA = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh,
				actionNameAsyncIIWA, boost::bind(&KukaManager::callbackIIWAAsync, this, _1), true);
	}
}

void KukaManager::startThreads()
{
	// Start controller as threads
	if (robot == KUKA::Robotname::Omnirob) {
		udpOmnirobControllerThread = new boost::thread(boost::bind(&UDPController::runThread,
				&udpOmnirobController));
		udpOmnirobPublisherThread = new boost::thread(boost::bind(&UDPPublisher::runThread,
				&udpOmnirobPublisher));
		platformControllerThread = new boost::thread(boost::bind(&UDPPlatformController::runThread,
				&platformController));
		omnirobPublisherThread = new boost::thread(boost::bind(&OmnirobPublisher::runThread,
				&omnirobPublisher));
	} else if (robot == KUKA::Robotname::IIWA) {
		udpIIWAControllerThread = new boost::thread(boost::bind(&UDPController::runThread,
				&udpIIWAController));
		udpIIWAPublisherThread = new boost::thread(boost::bind(&UDPPublisher::runThread,
				&udpIIWAPublisher));
	} else if (robot == KUKA::Robotname::Both) {
		udpOmnirobControllerThread = new boost::thread(boost::bind(&UDPController::runThread,
				&udpOmnirobController));
		udpOmnirobPublisherThread = new boost::thread(boost::bind(&UDPPublisher::runThread,
				&udpOmnirobPublisher));
		platformControllerThread = new boost::thread(boost::bind(&UDPPlatformController::runThread,
				&platformController));
		omnirobPublisherThread = new boost::thread(boost::bind(&OmnirobPublisher::runThread,
				&omnirobPublisher));

		udpIIWAControllerThread = new boost::thread(boost::bind(&UDPController::runThread,
				&udpIIWAController));
		udpIIWAPublisherThread = new boost::thread(boost::bind(&UDPPublisher::runThread,
				&udpIIWAPublisher));
	}
}

void KukaManager::spin()
{
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::waitForShutdown();
//	// Stay alive
//	ros::Rate r(500);
//	while (ros::ok())
//	{
//		//std::cout << "bla" << std::endl;
//		r.sleep();
//	}
}

void KukaManager::callbackOmnirob(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory)
{
	bool success = true;
	success = udpOmnirobController.sendTrajectory(trajectory, actionServerOmnirob);
	if (success) {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		ROS_INFO("%s: Successful", actionNameOmnirob.c_str());
		actionServerOmnirob->setSucceeded(_result);
	} else {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
		ROS_INFO("%s: Error", actionNameOmnirob.c_str());
		actionServerOmnirob->setAborted(_result);
	}
}

void KukaManager::callbackOmnirobAsync(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory)
{
	std::stringstream str;
	str << actionNameOmnirob << "_async";
	std::string actionNameAsync = str.str();
	bool success = true;
	success = udpOmnirobController.sendTrajectoryAsync(trajectory, actionServerAsyncOmnirob);
	if (success) {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		ROS_INFO("%s: Successful", actionNameAsync.c_str());
		actionServerAsyncOmnirob->setSucceeded(_result);
	} else {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
		ROS_INFO("%s: Error", actionNameAsync.c_str());
		actionServerAsyncOmnirob->setAborted(_result);
	}
}

void KukaManager::readPortConfig(std::string filename) {
	std::ifstream stream(filename);
	std::string name;
	int port;
	while(stream >> name >> port) {
		portMap[name] = port;
	}
}

void KukaManager::callbackIIWA(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory)
{
	bool success = true;
	success = udpIIWAController.sendTrajectory(trajectory, actionServerIIWA);
	if (success) {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		ROS_INFO("%s: Successful", actionNameIIWA.c_str());
		actionServerIIWA->setSucceeded(_result);
	} else {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
		ROS_INFO("%s: Error", actionNameIIWA.c_str());
		actionServerIIWA->setAborted(_result);
	}
}

void KukaManager::callbackOmnirobPreempt()
{
	ROS_INFO("Stop Arm!");
	udpOmnirobController.stopArm();
}

void KukaManager::callbackIIWAAsync(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory)
{
	std::stringstream str;
	str << actionNameIIWA << "_async";
	std::string actionNameAsync = str.str();
	bool success = true;
	success = udpIIWAController.sendTrajectoryAsync(trajectory, actionServerAsyncIIWA);
	if (success) {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		ROS_INFO("%s: Successful", actionNameAsync.c_str());
		actionServerAsyncIIWA->setSucceeded(_result);
	} else {
		_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
		ROS_INFO("%s: Error", actionNameAsync.c_str());
		actionServerAsyncIIWA->setAborted(_result);
	}
}
