/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: reference.cpp
 */

#include <goal_planner_gui/references/reference.h>

namespace goal_planner_gui
{

Reference::Reference()
{
}

Reference::~Reference()
{
}

bool Reference::operator !=(const Reference& rhs) const
		{
	return !(*this == rhs);
}

bool Reference::operator ==(const Reference& rhs) const
		{
	return hash() == rhs.hash();
}

bool Reference::semanticEqual(const std::shared_ptr<Reference>& other) const
		{
	return false;
}

bool Reference::matches(const std::shared_ptr<pddl::TypedObject>& o) const
		{
	return false;
}

std::string Reference::str() const
{
	static const auto defaultO = pddl::DefaultTypedObjects::unknownObject();
	return str(defaultO);
}

int Reference::refCount()
{
	return 1;
}

} /* namespace goal_planner_gui */
