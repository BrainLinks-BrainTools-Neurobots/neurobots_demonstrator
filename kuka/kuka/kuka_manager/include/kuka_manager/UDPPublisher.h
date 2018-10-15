/*
 * UDPPublisher.h
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */

#ifndef UDPPUBLISHER_H_
#define UDPPUBLISHER_H_

#include "kuka_manager/definitions.h"
#include "kuka_manager/UDPHandle.h"
#include "kuka_manager/kuka_robots.h"

#include <ros/ros.h>

class UDPPublisher {
public:
	UDPPublisher();
	virtual ~UDPPublisher();

	// Thread stuff
	void start(ros::NodeHandle& nh, const char* host, int port, int portRec,
			KUKA::Robotname robot);
	void runThread();

private:
	KUKA::Robotname robot;

	bool m_base_prismatic;

	// Ros stuff
	ros::NodeHandle nh;
	ros::Publisher jointstate_pub;
	ros::Publisher forceflange_pub;

	// UDP stuff
	UDPHandle udpJointStateHandle;


	// Functions
	void pubJointState(JointStatePacket &jointStatePacket);
};

#endif /* UDPPUBLISHER_H_ */
