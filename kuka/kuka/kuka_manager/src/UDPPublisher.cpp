/*
 * UDPPublisher.cpp
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include "kuka_manager/Force.h"
#include "kuka_manager/iiwa.h"

#include "kuka_manager/UDPPublisher.h"
#include <omnirob_lbr/lbr.h>

UDPPublisher::UDPPublisher() : robot(KUKA::Robotname::None),m_base_prismatic(false) {
}

UDPPublisher::~UDPPublisher() {
}

void UDPPublisher::start(ros::NodeHandle& nh, const char* host, int port, int portRec,
		KUKA::Robotname robot) {
	this->robot = robot;
	// Init ros stuff
	this->nh = nh;


	this->nh.getParam("kuka_manager/base_prismatic_joints",m_base_prismatic);
	ROS_INFO_STREAM(m_base_prismatic);

	// Init pubs and subs
	if (robot == KUKA::Robotname::Omnirob) {
		jointstate_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
	} else if (robot == KUKA::Robotname::IIWA) {
        jointstate_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
	}

	forceflange_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench_flange", 10);

	// Init udp stuff
	udpJointStateHandle.init(host, port, portRec);

	// Init finished
	ROS_INFO("Successfully initialized joint state publisher");
}

void UDPPublisher::runThread() {
	ros::Rate r(500.0);
	JointStatePacket jointStatePacket;
	while (ros::ok()) {
		// Receive packets
		int length = udpJointStateHandle.receive(&jointStatePacket, sizeof(jointStatePacket), 0.0);
		if (length != -1) {
			pubJointState(jointStatePacket);
		}

		r.sleep();
	}
}

void UDPPublisher::pubJointState(JointStatePacket &jointStatePacket) {

	// Joint state
	sensor_msgs::JointState jointState;
	jointState.header.stamp = ros::Time::now();
	if (robot == KUKA::Robotname::Omnirob) {
		if (m_base_prismatic){
			jointState.name.push_back(LBRJoints::base_x_joint);
			jointState.name.push_back(LBRJoints::base_y_joint);
			jointState.name.push_back(LBRJoints::base_theta_joint);

			jointState.position.push_back(0.0);
			jointState.position.push_back(0.0);
			jointState.position.push_back(0.0);

			jointState.effort.push_back(0.0);
			jointState.effort.push_back(0.0);
			jointState.effort.push_back(0.0);
		}
		jointState.name.push_back(LBRJoints::joint1);
		jointState.name.push_back(LBRJoints::joint2);
		jointState.name.push_back(LBRJoints::joint3);
		jointState.name.push_back(LBRJoints::joint4);
		jointState.name.push_back(LBRJoints::joint5);
		jointState.name.push_back(LBRJoints::joint6);
		jointState.name.push_back(LBRJoints::joint7);
	} else if (robot == KUKA::Robotname::IIWA) {
		jointState.name.push_back(IIWAJoints::joint1);
		jointState.name.push_back(IIWAJoints::joint2);
		jointState.name.push_back(IIWAJoints::joint3);
		jointState.name.push_back(IIWAJoints::joint4);
		jointState.name.push_back(IIWAJoints::joint5);
		jointState.name.push_back(IIWAJoints::joint6);
		jointState.name.push_back(IIWAJoints::joint7);
	}

	jointState.position.push_back(jointStatePacket.j1);
	jointState.position.push_back(jointStatePacket.j2);
	jointState.position.push_back(jointStatePacket.j3);
	jointState.position.push_back(jointStatePacket.j4);
	jointState.position.push_back(jointStatePacket.j5);
	jointState.position.push_back(jointStatePacket.j6);
	jointState.position.push_back(jointStatePacket.j7);

	jointState.effort.push_back(jointStatePacket.t1);
	jointState.effort.push_back(jointStatePacket.t2);
	jointState.effort.push_back(jointStatePacket.t3);
	jointState.effort.push_back(jointStatePacket.t4);
	jointState.effort.push_back(jointStatePacket.t5);
	jointState.effort.push_back(jointStatePacket.t6);
	jointState.effort.push_back(jointStatePacket.t7);

	jointstate_pub.publish(jointState);

	//Flange force
	geometry_msgs::WrenchStamped wrench;
	wrench.header.stamp = jointState.header.stamp;
	wrench.wrench.force.x = jointStatePacket.forceFlangeX;
	wrench.wrench.force.y = jointStatePacket.forceFlangeY;
	wrench.wrench.force.z = jointStatePacket.forceFlangeZ;
	wrench.wrench.torque.x = jointStatePacket.torqueFlangeX;
	wrench.wrench.torque.y = jointStatePacket.torqueFlangeY;
	wrench.wrench.torque.z = jointStatePacket.torqueFlangeZ;

	forceflange_pub.publish(wrench);
}

