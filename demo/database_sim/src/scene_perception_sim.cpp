/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: scene_perception_sim.cpp
 */

#include <boost/variant.hpp>
#include <database_conversions/database_conversions.h>
#include <database_msgs/get_objects.h>
#include <database_msgs/update_object.h>
#include <database_msgs/add_object.h>
#include <database_msgs/remove_object.h>
#include <ros/ros.h>
#include <ctime>

using namespace std;

ros::ServiceClient clientGetObjects;
ros::ServiceClient clientUpdateObject, clientAddObject, clientRemoveObject;
ObjectMap objects;

void updateObjects()
{
	database_msgs::get_objects srv;
	if (!clientGetObjects.call(srv))
	{
		ROS_ERROR("cannot call service get_objects");
	}

	database_conversions::convertMsgToObjects(srv.response.objects, srv.response.names, objects);
}

void init()
{
	updateObjects();

	for (auto& it : objects)
	{
		if (IS_PERCEPTIBLE(it.second))
		{
			database_msgs::update_object srv;
			srv.request.name = it.first;
			srv.request.send_changes = false;

			//set attribute seen
			Object object;
			object["seen"].push_back(false);
			database_conversions::convertObjectToMsg(object, srv.request.object);

			clientUpdateObject.call(srv);
		}
	}
}

void sendPoseChange()
{
	updateObjects();

	Object object = std::next(std::begin(objects), rand() % objects.size())->second;
	while (!IS_PERCEPTIBLE(object))
		object = std::next(std::begin(objects), rand() % objects.size())->second;

	Object changes;
	Eigen::Affine3d pose = Eigen::Affine3d::Identity();
	pose.translate(Eigen::Vector3d::Random());
	changes["pose"].push_back(pose);

	database_msgs::update_object srv;
	srv.request.name = CONVERT_TO_STRING(object["name"]);
	srv.request.send_changes = true;
	database_conversions::convertObjectToMsg(changes, srv.request.object);

	clientUpdateObject.call(srv);
}

void addObject()
{
	auto t = std::time(NULL);
	auto tm = *std::localtime(&t);
	std::string name = "bottle_" + std::to_string(tm.tm_hour) + "_" + std::to_string(tm.tm_min) + "_" + std::to_string(tm.tm_sec);
	Eigen::Affine3d pose = Eigen::Affine3d::Identity();
	pose.translate(Eigen::Vector3d::Random());

	Object object;
	object["name"].push_back(name);
	object["type"].push_back(std::string("bottle"));
	object["seen"].push_back(bool(true));
	object["pose"].push_back(pose);

	database_msgs::add_object srv;
	srv.request.name = name;
	database_conversions::convertObjectToMsg(object, srv.request.object);

	clientAddObject.call(srv);

	updateObjects();
}

void removeObject()
{
	updateObjects();

	Object object = std::next(std::begin(objects), rand() % objects.size())->second;
	while (!IS_PERCEPTIBLE(object))
		object = std::next(std::begin(objects), rand() % objects.size())->second;

	database_msgs::remove_object srv;
	srv.request.name = CONVERT_TO_STRING(object["name"]);
	clientRemoveObject.call(srv);

	updateObjects();
}

int main(int argc,
		char** argv)
{
	//Init Node
	ros::init(argc, argv, "scene_perception_sim");

	//Node Handle
	ros::NodeHandle nh("neurobots_database");

	clientGetObjects = nh.serviceClient<database_msgs::get_objects>("get_objects");
	clientUpdateObject = nh.serviceClient<database_msgs::update_object>("update_object");
	clientAddObject = nh.serviceClient<database_msgs::add_object>("add_object");
	clientRemoveObject = nh.serviceClient<database_msgs::remove_object>("remove_object");

	ros::AsyncSpinner spinner(0);
	spinner.start();

	init();

	ros::Rate r(0.25);

	int c = 0;
	while (ros::ok())
	{
		//send changes
		sendPoseChange();

		std::cout << "*";
		std::flush(cout);
		++c;
		if (c == 10)
		{
			c = 0;
			std::cout << std::endl;
		}

		//send new object
		if (rand() % 5 == 0)
		{
			addObject();
			std::cout << "+";
			std::flush(cout);
			++c;
			if (c == 10)
			{
				c = 0;
				std::cout << std::endl;
			}
		}

		//remove object
		if (rand() % 5 == 0)
		{
			removeObject();
			std::cout << "-";
			std::flush(cout);
			++c;
			if (c == 10)
			{
				c = 0;
				std::cout << std::endl;
			}
		}

		r.sleep();
	}

	return 0;
}

