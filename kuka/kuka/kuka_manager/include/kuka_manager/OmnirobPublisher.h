/*
 * UDPOmnirobPublisher.h
 *
 *  Created on: 06.06.2014
 *      Author: andreas
 */

#ifndef OMNIROBPUBLISHER_H_
#define OMNIROBPUBLISHER_H_

#include "kuka_manager/UDPHandle.h"
#include "kuka_manager/definitions.h"
#include "kuka_manager/omniRobMsgDefs.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

class OmnirobPublisher {
public:
	OmnirobPublisher();
	virtual ~OmnirobPublisher();

	void start(ros::NodeHandle& nh);
	void runThread();
	void spinOnce();

private:
	void fillLaserMsg(sensor_msgs::LaserScan &roslaser, LaserMsg* laser, std::string &frameName);
	bool readLaserMsgFromBuffer(int dataLength, char* buffer, LaserMsg& targetMsg);

	void fillOdometryMsgAndTransform(nav_msgs::Odometry &rosOdometry, geometry_msgs::TransformStamped &rosTransform, OdoData* odomMsg);
	bool readOdometryMsgFromBuffer(int dataLength, char* buffer, OdoData& targetMsg);

	// Ros stuff
	ros::NodeHandle nh;

	UDPHandle udpListenerFrontLaser;
	UDPHandle udpListenerRearLaser;
	UDPHandle udpListenerOdometry;

	double laserFrequency;

	std::string laserFrontFrameName;
	std::string laserRearFrameName;
	std::string odometryFrameName;
	int portFrontLaser;
	int portRearLaser;
	int portOdometry;

	ros::Publisher laser_front_pub;
	ros::Publisher laser_rear_pub;
	tf::TransformBroadcaster pose_odo_broadcaster;
	ros::Publisher pose_odo_pub;

	ros::Subscriber commandVel_sub;

	sensor_msgs::LaserScan laserMsg;
	nav_msgs::Odometry odoMsg;
	geometry_msgs::TransformStamped transOdo;

	LaserMsg frontLaserUDP;
	LaserMsg rearLaserUDP;
	OdoData odometryUDP;
};

#endif /* OMNIROBPUBLISHER_H_ */
