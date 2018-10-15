/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: database.h
 */

#ifndef NEUROBOTS_DATABASE_DATABASE_H_
#define NEUROBOTS_DATABASE_DATABASE_H_

#include <database_msgs/get_object.h>
#include <database_msgs/get_objects.h>
#include <database_msgs/get_world.h>

#include <database_msgs/add_object.h>
#include <database_msgs/update_object.h>
#include <database_msgs/remove_object.h>

#include <database_msgs/add_empty_object.h>
#include <database_msgs/set_bool_attribute.h>
#include <database_msgs/set_string_attribute.h>

#include <database_msgs/object.h>

#include <ros/ros.h>
#include <unordered_map>

//Struct for Knowledge Base
#include <database_conversions/knowledge_base.h>

namespace neurobots_database
{
//* NeurobotsDatabase
/**
 * Class holding the state of the world
 */
class Database
{
public:
	struct SpecialType
	{
		bool perceptible;
		bool locatable;
	};

public:
	Database(std::string domain); // e.g. "neurobots-demo" (".pddl" and "-domain.pddl" will be added automatically)
	virtual ~Database();

	/**
	 * Service callback to get an object based on its name.
	 * If no object was found, the corresponding flag in result is set.
	 * In this case the resulting object information is also invalid.
	 * */
	bool getObject(database_msgs::get_object::Request& req,
			database_msgs::get_object::Response& res);

	/**
	 * Returns a list of all objects
	 */
	bool getObjects(database_msgs::get_objects::Request& req,
			database_msgs::get_objects::Response& res);

	bool getWorld(database_msgs::get_world::Request& req,
			database_msgs::get_world::Response& res);

	/**
	 * Adds an object using its name as a key.
	 * If an object with the same name exists,
	 * the old object will not be replaced by the new one.
	 */
	bool addObject(database_msgs::add_object::Request &req,
			database_msgs::add_object::Response &res);

	/**
	 * Updates an object using its name as a key.
	 * If the object not exists nothing happens
	 */
	bool updateObject(database_msgs::update_object::Request &req,
			database_msgs::update_object::Response &res);

	/**
	 * Updates an object using its name as a key.
	 * If the object not exists nothing happens (used for
	 * operator, not inteded to use in real application)
	 */
	bool addEmptyObject(database_msgs::add_empty_object::Request &req,
			database_msgs::add_empty_object::Response &res);

	/**
	 * Updates an object using its name as a key.
	 * If the object not exists nothing happens (used for
	 * operator, not intended to use in real application)
	 */
	bool updateObjectBoolAttribute(database_msgs::set_bool_attribute::Request &req,
			database_msgs::set_bool_attribute::Response &res);

	/**
	 * Updates an object using its name as a key.
	 * If the object not exists nothing happens (used for
	 * operator, not intended to use in real application)
	 */
	bool updateObjectStringAttribute(database_msgs::set_string_attribute::Request &req,
			database_msgs::set_string_attribute::Response &res);

	/**
	 * Remove an object using its name as a key.
	 * Return false when object does not exist.
	 */
	bool removeObject(database_msgs::remove_object::Request &req,
			database_msgs::remove_object::Response &res);

	/**
	 * Converts a vector of doubles to a Eigen matrix.
	 * vec needs to have < 16 elements
	 */
	static Eigen::Affine3d getMat(const std::vector<double>& vec);

private:

    /**
     * Modifies an object of the database
     */
    bool modifyObject(database_msgs::object req_object, std::string name, bool send_changes);

	/**
	 * Sends changes of the database
	 */
	void sendChange(const std::string& objectName);

	/**
	 * Returns the special type of the object based on
	 * its type. The special type can be locatable or
	 * perceptible
	 */
	SpecialType getSpecialType(const std::string& type);

	/**
	 * Initializes the database
	 */
	void init(std::string domain);

private:

	//Knowledge Base
	World m_world;

	ros::Publisher m_databaseChange; /**< Publisher: sends a message with database information and can be used as a trigger */

};

} /* namespace neurobots_database */

#endif /* NEUROBOTS_DATABASE_DATABASE_H_ */
