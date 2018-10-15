/*
 * UDPOmnirobPublisher.cpp
 *
 *  Created on: 06.06.2014
 *      Author: andreas
 */

#include "kuka_manager/OmnirobPublisher.h"

#include <sensor_msgs/JointState.h>
#include <timelib/timelib.h>
#include <math.h>

OmnirobPublisher::OmnirobPublisher() :
				portRearLaser(0),
				portOdometry(0),
				portFrontLaser(0)
{

	ros::NodeHandle nh;

	std::string tf_prefix;
	nh.param("tf_prefix", tf_prefix, std::string(""));

	//std::cout<<"----------------------------------"<<std::endl;
	//std::cout<<tf_prefix<<std::endl;
	//std::cout<<"----------------------------------"<<std::endl;

	laserMsg.header.seq = 0;
	//TODO get correct values
	laserMsg.range_min = 0.0;
	laserFrequency = 12.5;
	laserFrontFrameName = tf_prefix + "/laser_front_link";
	laserRearFrameName = tf_prefix + "/laser_rear_link";
	odometryFrameName = tf_prefix + "/odom";

	frontLaserUDP.num_readings = 0;
	frontLaserUDP.range = NULL;
	frontLaserUDP.num_remissions = 0;
	frontLaserUDP.remission = NULL;
	frontLaserUDP.host = new char[255];

	rearLaserUDP.num_readings = 0;
	rearLaserUDP.range = NULL;
	rearLaserUDP.num_remissions = 0;
	rearLaserUDP.remission = NULL;
	rearLaserUDP.host = new char[255];

	odometryUDP.host = new char[255];

	transOdo.header.frame_id = tf_prefix + "/odom";
	transOdo.child_frame_id = tf_prefix + "/base_link";
	transOdo.header.seq = 0;
	transOdo.transform.translation.z = 0.0;

	odoMsg.pose.pose.position.z = 0.0;
	odoMsg.header.frame_id = tf_prefix + "/odom";
	odoMsg.child_frame_id = tf_prefix + "/base_link";
	odoMsg.header.seq = 0;
	odoMsg.twist.twist.linear.z = 0.0;
	odoMsg.twist.twist.angular.x = 0.0;
	odoMsg.twist.twist.angular.y = 0.0;

	//TODO make these parameters!
	//  omnirob.initialize(1.5, 1.5, 3.0, 0.1);
}

OmnirobPublisher::~OmnirobPublisher()
{
	timelib::cleanUp();

	if (frontLaserUDP.range)
	{
		delete[] frontLaserUDP.range;
		frontLaserUDP.range = NULL;
	}
	if (frontLaserUDP.remission)
	{
		delete[] frontLaserUDP.remission;
		frontLaserUDP.remission = NULL;
	}
	if (frontLaserUDP.host != NULL)
	{
		delete[] frontLaserUDP.host;
		frontLaserUDP.host = NULL;
	}

	if (rearLaserUDP.range)
	{
		delete[] rearLaserUDP.range;
		rearLaserUDP.range = NULL;
	}
	if (rearLaserUDP.remission)
	{
		delete[] rearLaserUDP.remission;
		rearLaserUDP.remission = NULL;
	}
	if (rearLaserUDP.host != NULL)
	{
		delete[] rearLaserUDP.host;
		rearLaserUDP.host = NULL;
	}
	if (odometryUDP.host != NULL)
	{
		delete[] odometryUDP.host;
		odometryUDP.host = NULL;
	}
}

void OmnirobPublisher::start(ros::NodeHandle& nh)
{
	// Init ros stuff
	this->nh = nh;

	// Init udp stuff
	std::string sunrise_server;
	nh.param<std::string>("kuka_manager/sunrise_server", sunrise_server, "192.168.42.155");
	nh.param<int>("kuka_manager/portFrontLaser", portFrontLaser, 34791);
	nh.param<int>("kuka_manager/portRearLaser", portRearLaser, 34792);
	nh.param<int>("kuka_manager/portOdometry", portOdometry, 34789);

	bool status = udpListenerFrontLaser.initRecieve(portFrontLaser);
	if (!status)
	{
		std::cerr << "Unable to init UDP Front listener" << std::endl;
	}

	status = udpListenerRearLaser.initRecieve(portRearLaser);
	if (!status)
	{
		std::cerr << "Unable to init UDP Rear listener" << std::endl;
	}

	status = udpListenerOdometry.initRecieve(portOdometry);
	if (!status)
	{
		std::cerr << "Unable to init Odometry listener" << std::endl;
	}

	timelib::init(sunrise_server.c_str());

	// Init pubs and subs
	laser_front_pub = nh.advertise<sensor_msgs::LaserScan>("scan_front", 10);
	laser_rear_pub = nh.advertise<sensor_msgs::LaserScan>("scan_rear", 10);
	pose_odo_pub = nh.advertise<nav_msgs::Odometry>("sensor_pose_odo", 10);

	// Init finished
	ROS_INFO("Ports of Frontlaser: %d Rearlaser: %d Odometry: %d", portFrontLaser, portRearLaser, portOdometry);
	ROS_INFO("Successfully initialized Omnirob publisher");
}

void OmnirobPublisher::runThread()
{
	ros::Rate r(500.0);
	while (ros::ok())
	{
		spinOnce();
		r.sleep();
	}
}

bool OmnirobPublisher::readLaserMsgFromBuffer(int dataLength,
		char* buffer,
		LaserMsg& targetMsg)
{
	char* p = buffer;
	if (p + sizeof(int) > buffer + dataLength)
		return false;
	targetMsg.id = *((int*) p);
	p += sizeof(int);
	if (p + sizeof(int) > buffer + dataLength)
		return false;
	targetMsg.laser_type = *((int*) p);
	p += sizeof(int);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.start_angle = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.fov = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.angular_resolution = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.maximum_range = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.accuracy = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(int) > buffer + dataLength)
		return false;
	targetMsg.remission_mode = *((int*) p);
	p += sizeof(int);
	int oldNumReadings = targetMsg.num_readings;
	if (p + sizeof(int) > buffer + dataLength)
		return false;
	targetMsg.num_readings = *((int*) p);
	p += sizeof(int);
	if (p + targetMsg.num_readings * sizeof(float) > buffer + dataLength)
		return false;
	if (oldNumReadings != targetMsg.num_readings)
	{
		if (targetMsg.range != NULL)
		{
			delete[] targetMsg.range;
		}
		targetMsg.range = new float[targetMsg.num_readings];
	}
	for (int i = 0; i < targetMsg.num_readings; i++)
	{
		targetMsg.range[i] = *((float*) p);
		p += sizeof(float);
	}
	int oldNumRemissons = targetMsg.num_remissions;
	if (p + sizeof(int) > buffer + dataLength)
		return false;
	targetMsg.num_remissions = *((int*) p);
	p += sizeof(int);
	if (p + targetMsg.num_remissions * sizeof(float) > buffer + dataLength)
		return false;
	if (oldNumRemissons != targetMsg.num_remissions)
	{
		if (targetMsg.remission != NULL)
		{
			delete[] targetMsg.remission;
		}
		targetMsg.remission = new float[targetMsg.num_remissions];
	}
	for (int i = 0; i < targetMsg.num_remissions; i++)
	{
		targetMsg.remission[i] = *((float*) p);
		p += sizeof(float);
	}
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.laser_pose_x = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.laser_pose_y = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.laser_pose_theta = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.robot_pose_x = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.robot_pose_y = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.robot_pose_theta = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.tv = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.rv = *((double*) p);
	p += sizeof(double);

	if (p + sizeof(unsigned long long) > buffer + dataLength)
		return false;
	targetMsg.timestamp_sec = *((unsigned long long*) p);
	p += sizeof(unsigned long long);
	if (p + sizeof(unsigned long long) > buffer + dataLength)
		return false;
	targetMsg.timestamp_nsec = *((unsigned long long*) p);
	p += sizeof(unsigned long long);

	int length = strlen(p);
	if (p + length == buffer + dataLength)
	{
		strcpy(targetMsg.host, p);
	}
	else
	{
		return false;
	}

	return true;
}

bool OmnirobPublisher::readOdometryMsgFromBuffer(int dataLength,
		char* buffer,
		OdoData& targetMsg)
{
	char* p = buffer;
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.x = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.y = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.theta = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.xv = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.yv = *((double*) p);
	p += sizeof(double);
	if (p + sizeof(double) > buffer + dataLength)
		return false;
	targetMsg.thetav = *((double*) p);
	p += sizeof(double);

	if (p + sizeof(unsigned long long) > buffer + dataLength)
		return false;
	targetMsg.timestamp_sec = *((unsigned long long*) p);
	p += sizeof(unsigned long long);
	if (p + sizeof(unsigned long long) > buffer + dataLength)
		return false;
	targetMsg.timestamp_nsec = *((unsigned long long*) p);
	p += sizeof(unsigned long long);

	int length = strlen(p);
	if (p + length == buffer + dataLength)
	{
		strcpy(targetMsg.host, p);
	}
	else
	{
		return false;
	}

	return true;
}

void OmnirobPublisher::fillLaserMsg(sensor_msgs::LaserScan &roslaser,
		LaserMsg* laser,
		std::string &frameName)
{
	if (!timelib::filterStable())
	{
		std::cerr << "Time filter not yet stable" << std::endl;
		return;
	}

	//  std::cerr<<"id "<<laser->id<<" type "<<laser->laser_type<<" start angle "<<laser->start_angle<<" "<<laser->tv<<" "<<laser->rv<<std::endl;
	//std::cerr << "sec " << laser->timestamp_sec << " nsec " << laser->timestamp_nsec << std::endl;

	int64_t timestamp = timelib::getClientTime((int64_t) (laser->timestamp_sec * 1000000000 + laser->timestamp_nsec));
	roslaser.header.stamp.fromNSec(timestamp);
	roslaser.header.seq++;
	roslaser.header.frame_id = frameName;
	roslaser.angle_min = laser->start_angle;
	roslaser.angle_max = laser->start_angle + laser->fov;
	roslaser.angle_increment = laser->angular_resolution;
	roslaser.range_max = laser->maximum_range;
	roslaser.scan_time = (1.0 / laserFrequency);
	roslaser.time_increment = (1.0 / laserFrequency) / (laser->num_readings);

	if (laser->num_readings > 0)
		roslaser.ranges.assign(laser->range, laser->range + laser->num_readings);
	else
		roslaser.ranges.clear();

	if (laser->num_remissions > 0)
		roslaser.intensities.assign(laser->remission, laser->remission + laser->num_remissions);
	else
		roslaser.intensities.clear();
}

void OmnirobPublisher::fillOdometryMsgAndTransform(nav_msgs::Odometry &rosOdometry,
		geometry_msgs::TransformStamped &rosTransform,
		OdoData* odomMsg)
{
	if (!timelib::filterStable())
	{
		std::cerr << "Time filter not yet stable" << std::endl;
		return;
	}

	//std::cerr << "sec 2 " << std::endl;

	int64_t timestamp = timelib::getClientTime((int64_t) (odomMsg->timestamp_sec * 1000000000 + odomMsg->timestamp_nsec));
	rosOdometry.header.stamp.fromNSec(timestamp);
	rosOdometry.header.seq++;

	geometry_msgs::Quaternion thetaQuat = tf::createQuaternionMsgFromYaw(odomMsg->theta);
	rosOdometry.pose.pose.position.x = odomMsg->x;
	rosOdometry.pose.pose.position.y = odomMsg->y;
	rosOdometry.pose.pose.orientation = thetaQuat;
	rosOdometry.twist.twist.linear.x = odomMsg->xv;
	rosOdometry.twist.twist.linear.y = odomMsg->yv;
	rosOdometry.twist.twist.angular.z = odomMsg->thetav;

	rosTransform.header.stamp.fromNSec(timestamp);
	rosTransform.header.seq++;
	rosTransform.transform.translation.x = odomMsg->x;
	rosTransform.transform.translation.y = odomMsg->y;
	rosTransform.transform.rotation = thetaQuat;
}

void OmnirobPublisher::spinOnce()
{
	//TODO these are now hardcoded values that should be big enough for a laser message from the omnirob. accounting for 600 rays with remmissions and a 255 character hostname
	static int laserBufferSize = sizeof(frontLaserUDP) + 2 * 600 * sizeof(float) + 255;
	static char laserBuffer[sizeof(frontLaserUDP) + 2 * 600 * sizeof(float) + 255];

	int received = udpListenerFrontLaser.receiveFrom(laserBuffer, laserBufferSize, 0);
	//std::cout << "Received size: " << received << std::endl;
	if (received > laserBufferSize)
	{
		std::cerr << "Buffer too small for laser message! " << laserBufferSize << " vs. " << received << std::endl;
		return;
	}
	if (received > 0)
	{
		//std::cerr << "f";
		bool status = readLaserMsgFromBuffer(received, laserBuffer, frontLaserUDP);

		if (status == true)
		{
			fillLaserMsg(laserMsg, &frontLaserUDP, laserFrontFrameName);
			laser_front_pub.publish(laserMsg);
		}
		else
		{
			std::cerr << "laser parse error" << std::endl;
		}
	}

	received = udpListenerRearLaser.receiveFrom(laserBuffer, laserBufferSize, 0);
	if (received > laserBufferSize)
	{
		std::cerr << "Buffer too small for laser message! " << laserBufferSize << " vs. " << received << std::endl;
		return;
	}
	if (received > 0)
	{
		//std::cerr << "b";
		bool status = readLaserMsgFromBuffer(received, laserBuffer, rearLaserUDP);

		if (status == true)
		{
			fillLaserMsg(laserMsg, &rearLaserUDP, laserRearFrameName);
			laser_rear_pub.publish(laserMsg);
		}
		else
		{
			std::cerr << "laser parse error" << std::endl;
		}
	}

	//TODO these are now hardcoded values that should be big enough for a odometry message from the omnirob. accounting for a 255 character hostname
	static int odoBufferSize = sizeof(odometryUDP) + 255;
	static char odoBuffer[sizeof(odometryUDP) + 255];

	received = udpListenerOdometry.receiveFrom(odoBuffer, odoBufferSize, 0);
	if (received > odoBufferSize)
	{
		std::cerr << "Buffer too small for odometry message! " << odoBufferSize << " vs. " << received << std::endl;
		return;
	}
	if (received > 0)
	{
		//std::cerr << "o";
		bool status = readOdometryMsgFromBuffer(received, odoBuffer, odometryUDP);
		if (status == true)
		{
			fillOdometryMsgAndTransform(odoMsg, transOdo, &odometryUDP);
			pose_odo_pub.publish(odoMsg);
			pose_odo_broadcaster.sendTransform(transOdo);
		}
		else
		{
			std::cerr << "Odometry parse error" << std::endl;
		}
	}
}

