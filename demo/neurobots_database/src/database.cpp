/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: database.cpp
 */

#include <boost/filesystem/operations.hpp>
#include <database_conversions/database_conversions.h>
#include <database_msgs/database_change.h>
#include <neurobots_database/database.h>
#include <neurobots_database/pddl_parser.h>
#include <neurobots_database/attributes.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <unordered_map>

namespace neurobots_database
{

Database::Database(std::string domain)
{
	std::cout << "creating database " << domain << std::endl;
	init(domain);

	//send initial database change msg
	database_msgs::database_change msg;
	msg.init = true;
	sleep(1);
	//Send initial database change message
	m_databaseChange.publish(msg);
}

Database::~Database()
{
}

//Service Callbacks
bool Database::getObject(database_msgs::get_object::Request& req,
		database_msgs::get_object::Response& res)
{
	auto it = m_world.objects.find(req.name);
	if (it == m_world.objects.end())
	{
		res.found = false;
	}
	else
	{
		res.found = true;
		database_conversions::convertObjectToMsg(it->second, res.object);
	}

	return true;
}

bool Database::getObjects(database_msgs::get_objects::Request& req,
		database_msgs::get_objects::Response& res)
{
	for (auto& it : m_world.objects)
	{
		database_msgs::object msg;
		database_conversions::convertObjectToMsg(it.second, msg);
		res.names.push_back(it.first);
		res.objects.push_back(msg);
	}

	return true;
}

bool Database::getWorld(database_msgs::get_world::Request& req,
		database_msgs::get_world::Response& res)
{
	for (auto& it : m_world.objects)
	{
		database_msgs::object msg;
		database_conversions::convertObjectToMsg(it.second, msg);
		res.world.object_names.push_back(it.first);
		res.world.objects.push_back(msg);
	}
	res.world.domain = m_world.domain;
	return true;
}

bool Database::addObject(database_msgs::add_object::Request& req,
		database_msgs::add_object::Response& res)
{
	res.success = modifyObject(req.object, req.name, req.send_changes);

	return true;
}

bool Database::addEmptyObject(database_msgs::add_empty_object::Request& req,
		database_msgs::add_empty_object::Response& res)
{
	if (m_world.objects.find(req.name) == m_world.objects.end())
	{
		Object object;
		object["type"].push_back(req.type);
		object["name"].push_back(req.name);

		SpecialType st = getSpecialType(req.type);

		//add default attributes, if required
		if (st.locatable)
		{
			object["locatable"].resize(1);
			object["locatable"][0] = true;
			for (auto& defaultAtt : Attributes::defaultAttributes["locatable"])
			{
				if (object.find(defaultAtt.first) == object.end())
				{
					object[defaultAtt.first] = defaultAtt.second;
				}
			}
		}

		if (st.perceptible)
		{
			object["perceptible"].resize(1);
			object["perceptible"][0] = true;
			for (auto& defaultAtt : Attributes::defaultAttributes["perceptible"])
			{
				if (object.find(defaultAtt.first) == object.end())
				{
					object[defaultAtt.first] = defaultAtt.second;
				}
			}
		}

		m_world.objects[req.name] = object;

		res.success = true;
	}
	else
	{
		res.success = false;
	}
	return true;
}

bool Database::updateObject(database_msgs::update_object::Request& req,
		database_msgs::update_object::Response& res)
{
	res.success = modifyObject(req.object, req.name, req.send_changes);

	return true;
}

bool Database::updateObjectStringAttribute(database_msgs::set_string_attribute::Request &req,
		database_msgs::set_string_attribute::Response &res)
{
	bool success = false;

	//Add object
	if (m_world.objects.find(req.name) != m_world.objects.end())
	{
		Object& object = m_world.objects[req.name];

		if (object[req.attribute].empty())
			object[req.attribute].push_back(std::string(req.value));
		else
			object[req.attribute][0] = req.value;

		if (req.send_change)
			sendChange(req.name);

		success = true;
	}

	res.success = success;

	return true;
}

bool Database::updateObjectBoolAttribute(database_msgs::set_bool_attribute::Request& req,
		database_msgs::set_bool_attribute::Response& res)
{
	bool success = false;

	//Add object
	if (m_world.objects.find(req.name) != m_world.objects.end())
	{
		Object& object = m_world.objects[req.name];

		if (object[req.attribute].empty())
			object[req.attribute].push_back(req.value);
		else
			object[req.attribute][0] = req.value;

		if (req.send_change)
			sendChange(req.name);

		success = true;
	}

	res.success = success;

	return true;
}

bool Database::modifyObject(database_msgs::object req_object,
		std::string name,
		bool send_changes)
{
	bool success = false;

	if (name.empty())
	{
		ROS_ERROR("Cannot modify object with empty name!");
		return false;
	}

	//Add object
	if (m_world.objects.find(name) == m_world.objects.end())
	{
		Object object;
		database_conversions::convertMsgToObject(req_object, object);
		success = true;

		if (!HAS_ATTRIBUTE(object, "type") || !HAS_ATTRIBUTE(object, "name"))
		{
			ROS_ERROR("Cannot add an object without type and name!");
			success = false;
			return false;
		}

		SpecialType st = getSpecialType(CONVERT_TO_STRING(object["type"]));

		//add default attributes, if required
		if (st.locatable)
		{
			object["locatable"].resize(1);
			object["locatable"][0] = true;
			for (auto& defaultAtt : Attributes::defaultAttributes["locatable"])
			{
				if (object.find(defaultAtt.first) == object.end())
				{
					object[defaultAtt.first] = defaultAtt.second;
				}
			}
		}

		if (st.perceptible)
		{
			object["perceptible"].resize(1);
			object["perceptible"][0] = true;
			for (auto& defaultAtt : Attributes::defaultAttributes["perceptible"])
			{
				if (object.find(defaultAtt.first) == object.end())
				{
					object[defaultAtt.first] = defaultAtt.second;
				}
			}
		}

		m_world.objects[name] = object;

		sendChange(name);

//        ROS_INFO("Added object with name %s, type %s, and special types: locatable: %d, perceptible: %d",
//                CONVERT_TO_STRING(m_world.objects[name]["name"]).c_str(),
//                CONVERT_TO_STRING(m_world.objects[name]["type"]).c_str(),
//                CONVERT_TO_BOOL(m_world.objects[name]["locatable"]),
//                CONVERT_TO_BOOL(m_world.objects[name]["perceptible"]));
	}
	else //Update object
	{
		Object object;
		database_conversions::convertMsgToObject(req_object, object);
		Object& databaseObject = m_world.objects[name];

		for (auto& it : object)
		{
			std::string attribute = it.first;
			size_t attributeSize = it.second.size();

			//continue read only parameters
			if ((attribute == "type" && CONVERT_TO_STRING(object["type"]) != CONVERT_TO_STRING(databaseObject["type"]))
					|| (attribute == "perceptible" && CONVERT_TO_BOOL(object["perceptible"]) != CONVERT_TO_BOOL(databaseObject["perceptible"]))
					|| (attribute == "locatable" && CONVERT_TO_BOOL(object["locatable"]) != CONVERT_TO_BOOL(databaseObject["locatable"])))
			{
//                ROS_ERROR("You cannot overwrite read only attributes as type, perceptible, or locatable");
				continue;
			}

			//remove attribute
			if (attributeSize == 0)
			{
				//we cannot remove such attributes
				if (attribute == "location" || attribute == "in" || attribute == "contents")
				{
					ROS_ERROR("You cannot remove the attributes location, in, or contents");
					continue;
				}

				//check if user wants to delete attributes of locatables and perceptibles
				if (CONVERT_TO_BOOL(object["locatable"]))
				{
					for (auto& defaultAtt : Attributes::defaultAttributes["locatable"])
					{
						if (attribute == defaultAtt.first)
						{
							ROS_ERROR("You cannot delete the attribute %s", attribute.c_str());
							continue;
						}
					}
				}
				else if (CONVERT_TO_BOOL(object["perceptible"]))
				{
					for (auto& defaultAtt : Attributes::defaultAttributes["perceptible"])
					{
						if (attribute == defaultAtt.first)
						{
							ROS_ERROR("You cannot delete the attribute %s", attribute.c_str());
							continue;
						}
					}
				}

				//delete attribute
				auto it = databaseObject.find(attribute);
				if (it != databaseObject.end())
					databaseObject.erase(it);

				continue;
			}

			databaseObject[attribute] = it.second;
		}

		success = true;

		if (send_changes)
		{
			sendChange(name);
		}

//        ROS_INFO("Updated object with name %s, type %s, and special types: locatable: %d, perceptible: %d",
//                CONVERT_TO_STRING(m_world.objects[name]["name"]).c_str(),
//                CONVERT_TO_STRING(m_world.objects[name]["type"]).c_str(),
//                CONVERT_TO_BOOL(m_world.objects[name]["locatable"]),
//                CONVERT_TO_BOOL(m_world.objects[name]["perceptible"]));
	}

	return success;

}

bool Database::removeObject(database_msgs::remove_object::Request& req,
		database_msgs::remove_object::Response& res)
{
	auto it = m_world.objects.find(req.name);
	if (it != m_world.objects.end())
	{
		m_world.objects.erase(it);
		res.success = true;
		sendChange(req.name);

		ROS_INFO("Removed object with name %s", req.name.c_str());
	}
	else
	{
		res.success = false;
	}

	return true;
}

void Database::sendChange(const std::string& objectName)
{
	//send database change msg
	database_msgs::database_change msg;
	msg.init = false;
	msg.changed_objects.push_back(objectName);
	m_databaseChange.publish(msg);
}

Database::SpecialType Database::getSpecialType(const std::string& type)
{
	SpecialType st;
	st.locatable = st.perceptible = false;
	for (auto& it : m_world.locatables)
	{
		if (it == type)
		{
			st.locatable = true;
			break;
		}
	}

	for (auto& it : m_world.locatables)
	{
		if (it == type)
		{
			st.perceptible = true;
			break;
		}
	}

	return st;
}

Eigen::Affine3d Database::getMat(const std::vector<double>& vec)
{
	Eigen::Affine3d mat;
	for (int i = 0; i < vec.size(); ++i)
	{
		mat.data()[i] = vec[i];
	}

	return mat;
}

void Database::init(std::string domain)
{
	ros::NodeHandle nh("~");

	std::string scenarioPath = ros::package::getPath("neurobots_scenario");
	std::string problemFile = scenarioPath + "/" + domain + ".pddl";
	std::string domainFile = scenarioPath + "/" + domain + "-domain.pddl";

	//load from parameter server
	if (domain.empty())
	{
		ROS_INFO_STREAM("Looking for problem_file/domain_file parameter in namespace " << nh.getNamespace().c_str());
		nh.getParam("problem_file", problemFile);
		nh.getParam("domain_file", domainFile);
	}

	if (!boost::filesystem::exists(problemFile)){
		ROS_ERROR("File does not exist: %s", problemFile.c_str());
		exit(-1);
	}
	if(!boost::filesystem::exists(domainFile)) {
		ROS_ERROR("File does not exist: %s", domainFile.c_str());
		exit(-2);
	}

	//Load World Knowledge Base from pddl file
	PDDLParser pddl;
	ROS_INFO("Using problem definition: %s", problemFile.c_str());
	pddl.load(problemFile, domainFile, m_world);

	//messages
	m_databaseChange = nh.advertise<database_msgs::database_change>("database_change", 5);

	ros::NodeHandle n("/robot_poses");
	std::vector<std::string> available;
	if (!n.getParam("available", available))
	{
		ROS_ERROR("Provide the robot_poses parameter!");
		exit(5);
	}

	for (auto& it : available)
	{
		std::vector<double> values;
		std::string paramName = "poses/" + it;
		n.getParam(paramName, values);

		//joint state
		if (values.size() == 7)
		{
			m_world.objects[it]["jointstate"].push_back(values);
			if (m_world.objects[it]["name"].empty())
				m_world.objects[it]["name"].push_back(it);

			if (m_world.objects[it]["type"].empty())
				m_world.objects[it]["type"].push_back(std::string("poses"));

		}
		else
		{
			Eigen::Affine3d pose = getMat(values);
			m_world.objects[it]["pose"].push_back(pose);
			if (m_world.objects[it]["name"].empty())
				m_world.objects[it]["name"].push_back(it);

			if (m_world.objects[it]["type"].empty())
				m_world.objects[it]["type"].push_back(std::string("poses"));
		}
	}
}

} /* namespace neurobots_database */
