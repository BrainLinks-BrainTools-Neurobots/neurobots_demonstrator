/*
 * UDPPlatformController.cpp
 *
 *  Created on: 06.06.2014
 *      Author: andreas
 */

#include "kuka_manager/UDPPlatformController.h"
#include "kuka_manager/definitions.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

UDPPlatformController::UDPPlatformController() {
	// Init parameter
	velCounter = 0;
}

UDPPlatformController::~UDPPlatformController() {
	// TODO Auto-generated destructor stub
}

void UDPPlatformController::start(ros::NodeHandle& nh, const char* host, int port) {
	// Init ros stuff
	this->nh = nh;
	//commandVelSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &UDPPlatformController::commandVelCallback, this);
	commandVelAsyncSub = nh.subscribe<geometry_msgs::Twist>("omnirob/cmd_vel", 1, &UDPPlatformController::commandVelAsyncCallback, this);
	changeVelAccSub = nh.subscribe<kuka_manager::Speed>("omnirob/ch_vel_acc_platform", 1, &UDPPlatformController::changeVelAccCallback, this);
	commandVelFinishedPub = nh.advertise<std_msgs::Bool>("omnirob/cmd_vel_finished", 1);

	// Init udp stuff
	udpSender.init(host, port, port + 1);

	// Init finished
	ROS_INFO("Successfully initialized platform controller");
}

void UDPPlatformController::runThread() {
  ros::spin();
  /*	ros::Rate r(100.0);
	while (nh.ok()) {
		ros::spinOnce();
		r.sleep();
	}
  */
}

void UDPPlatformController::changeVelAccCallback(const kuka_manager::Speed::ConstPtr &speed) {
	VelAccPacket velAccPacket;
	velAccPacket.accX = speed->accX;
	velAccPacket.accY = speed->accY;
	velAccPacket.accTheta = speed->accTheta;

	udpSender.send(&velAccPacket, sizeof(velAccPacket), 0.0);
}

void UDPPlatformController::commandVelCallback(const geometry_msgs::Twist::ConstPtr &cmd) {
	bool isFinished = moveOmnirob(ros::Time::now().toNSec(), velCounter, cmd->linear.x, cmd->linear.y, cmd->angular.z);
	std_msgs::Bool isFinishedMsg;
	isFinishedMsg.data = isFinished;

	commandVelFinishedPub.publish(isFinishedMsg);
}

void UDPPlatformController::commandVelAsyncCallback(const geometry_msgs::Twist::ConstPtr &cmd) {
	moveOmnirobAsync(ros::Time::now().toNSec(), velCounter, cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

bool UDPPlatformController::moveOmnirob(double timestamp, int counter, double vx, double vy, double vtheta) {
	VelocityPacket velocity;
	velocity.timestamp_msecs = timestamp;
	velocity.counter = counter;
	velocity.v_x = vx;
	velocity.v_y = vy;
	velocity.v_theta = vtheta;

	udpSender.send(&velocity, sizeof(velocity), 0.0);
	velCounter++;

	// Wait for return
	ActionFinishedPacket packet;
	int length = -1;
	ros::Rate r(100.0);
	while (length == -1) {
		length = udpSender.receive(&packet, sizeof(packet), 0.0);
		if (length != -1 && packet.isFinished) {
			return true;
		}
		r.sleep();
	}

	return false;
}

void UDPPlatformController::moveOmnirobAsync(double timestamp, int counter, double vx, double vy, double vtheta) {
	VelocityPacket velocity;
	velocity.timestamp_msecs = timestamp;
	velocity.counter = counter;
	velocity.v_x = vx;
	velocity.v_y = vy;
	velocity.v_theta = vtheta;

	udpSender.send(&velocity, sizeof(velocity), 0.0);
	velCounter++;
}
