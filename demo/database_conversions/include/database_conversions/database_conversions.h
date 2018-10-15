/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 1, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: database_conversions.h
 */

#ifndef DATABASE_CONVERSIONS_H_
#define DATABASE_CONVERSIONS_H_

#include <database_msgs/object.h>
#include <database_conversions/knowledge_base.h>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <limits.h>

namespace database_conversions
{
/**
 * Function to convert Knowledge Base Object to ROS Message
 * */
void convertObjectToMsg(Object& object,
		database_msgs::object& msg);

/**
 * Function to convert ROS Message to Knowledge Base Object
 */
void convertMsgToObject(const database_msgs::object& msg,
		Object& object);

/**
 * Function to convert a ROS Message vector of objects to
 * a map of Knowledge Base Objects
 */
void convertMsgToObjects(const std::vector<database_msgs::object>& msgObjects,
		const std::vector<std::string>& msgNames,
		ObjectMap& object);
}

/**
 * Always takes the first(!) element and converts it correspondingly
 */
#define CONVERT_TO_STRING(value) (value.empty() ? std::string("__empty_string__") : boost::get<std::string>(value.front()))
#define CONVERT_TO_BOOL(value) (value.empty() ? false : boost::get<bool>(value.front()))
#define CONVERT_TO_INT(value) (value.empty() ? std::numerical_limits<int>::nan() : boost::get<int>(value.front()))
#define CONVERT_TO_DOUBLE(value) (value.empty() ? std::numerical_limits<double>::nan() : boost::get<double>(value.front()))
#define CONVERT_TO_VECTOR(value) (value.empty() \
	? Eigen::Vector3d(std::numerical_limits<double>::nan(), \
			std::numerical_limits<double>::nan(), \
			std::numerical_limits<double>::nan())\
			: boost::get<Eigen::Vector3d>(value.front()))
#define CONVERT_TO_QUATERNION(value) (value.empty() \
	? Eigen::Quaterniond(std::numerical_limits<double>::nan(), \
			std::numerical_limits<double>::nan(), \
			std::numerical_limits<double>::nan(),\
			std::numerical_limits<double>::nan())\
			: boost::get<Eigen::Quaterniond>(value.front()))
#define CONVERT_TO_AFFINE(value) (value.empty() \
	? Eigen::Affine3d::Identity() \
			: boost::get<Eigen::Affine3d>(value.front()))
#define CONVERT_TO_DOUBLE_STD_VECTOR(value) (value.empty() \
	? std::vector<double>() \
			: boost::get<std::vector<double>>(value.front()))

#define HAS_ATTRIBUTE(object, query) (object.find(query) != object.end())
#define IS_PERCEPTIBLE(object) (boost::get<bool>(object["perceptible"].front()))
#define IS_LOCATABLE(object) (boost::get<bool>(object["locatable"].front()))

#endif /* DATABASE_CONVERSIONS_H_ */
