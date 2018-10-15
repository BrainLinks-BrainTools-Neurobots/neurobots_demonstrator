/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 1, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: database_conversions.cpp
 */

#include <database_conversions/database_conversions.h>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>

void database_conversions::convertObjectToMsg(Object& object,
		database_msgs::object& msg)
{
	for (auto& it : object)
	{
		msg.attribute_names.push_back(it.first);
		std::stringstream stream;
		boost::archive::text_oarchive ar(stream);
		ar & it.second;
		std::string result = boost::replace_first_copy(stream.str(), "serialization::archive 12", "serialization::archive 10"); //hack because of boost version 10 vs 12
		msg.attribute_values.push_back(result);
	}
}

void database_conversions::convertMsgToObject(const database_msgs::object& msg,
		Object& object)
{
	for (size_t i = 0; i < msg.attribute_names.size(); ++i)
	{
		std::stringstream stream(msg.attribute_values[i]);
		boost::archive::text_iarchive ar(stream);
		ar & object[msg.attribute_names[i]];
	}
}

void database_conversions::convertMsgToObjects(const std::vector<database_msgs::object>& msgObjects,
		const std::vector<std::string>& msgNames,
		ObjectMap& object)
{
	assert(msgObjects.size() == msgNames.size());

	for (size_t obj = 0; obj < msgObjects.size(); ++obj)
	{
		const database_msgs::object& objectMsg = msgObjects[obj];
		for (size_t i = 0; i < objectMsg.attribute_names.size(); ++i)
		{
			std::stringstream stream(objectMsg.attribute_values[i]);
			boost::archive::text_iarchive ar(stream);
			ar & object[msgNames[obj]][objectMsg.attribute_names[i]];
		}
	}
}
