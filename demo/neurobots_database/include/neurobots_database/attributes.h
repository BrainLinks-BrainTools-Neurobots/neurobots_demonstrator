/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 6, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: attributes.h
 */

#ifndef ATTRIBUTES_H_
#define ATTRIBUTES_H_
#include <database_conversions/knowledge_base.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace neurobots_database
{
struct Attributes
{
	/**
	 * All default attributes are defined here
	 */
	static std::unordered_map<std::string, std::unordered_map<std::string, AttributeValues>> defaultAttributes;
};
}

#endif /* ATTRIBUTES_H_ */
