/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: element.cpp
 */

#include <neurobots_database_gui/element.h>
#include <ros/ros.h>

namespace neurobots_database_gui
{

Element::Element(const std::string& type,
		const std::string& name,
		const std::unordered_map<std::string, std::vector<std::string>>& otherStringAttributes,
		const std::unordered_map<std::string, bool>& otherBoolAttributes) :
				m_type(type),
				m_name(name),
				m_stringAttributes(otherStringAttributes),
				m_boolAttributes(otherBoolAttributes)
{
}

Element::~Element()
{
}

Element::Element()
{
}

} /* namespace neurobots_database_gui */
