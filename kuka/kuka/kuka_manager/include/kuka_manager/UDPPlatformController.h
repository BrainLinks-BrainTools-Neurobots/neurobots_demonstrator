/*
 * UDPPlatformController.h
 *
 *  Created on: 06.06.2014
 *      Author: andreas
 */

#ifndef UDPPLATFORMCONTROLLER_H_
#define UDPPLATFORMCONTROLLER_H_

#include <ros/ros.h>
#include "kuka_manager/UDPHandle.h"
#include <geometry_msgs/Twist.h>
#include "kuka_manager/Speed.h"

class UDPPlatformController {
public:
	UDPPlatformController();
	virtual ~UDPPlatformController();

	// Thread stuff
	void start(ros::NodeHandle& nh, const char* host, int port);
	void runThread();

	// Ros stuff
	void commandVelCallback(const geometry_msgs::Twist::ConstPtr &cmd);
	void commandVelAsyncCallback(const geometry_msgs::Twist::ConstPtr &cmd);
	void changeVelAccCallback(const kuka_manager::Speed::ConstPtr &speed);

private:
	// Parameter
	int velCounter;

	// UDP stuff
	UDPHandle udpSender;

	// Ros objects
	ros::NodeHandle nh;
	ros::Subscriber commandVelSub;
	ros::Subscriber commandVelAsyncSub;
	ros::Subscriber changeVelAccSub;
	ros::Publisher commandVelFinishedPub;

	// Help functions
	bool moveOmnirob(double timestamp, int counter, double vx, double vy, double vtheta);
	void moveOmnirobAsync(double timestamp, int counter, double vx, double vy, double vtheta);
};

#endif /* UDPPLATFORMCONTROLLER_H_ */
