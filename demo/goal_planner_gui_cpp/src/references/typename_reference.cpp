/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: typename_reference.cpp
 */

#include <goal_planner_gui/references/typename_reference.h>
#include <goal_planner_gui/pddl/utils.h>

namespace goal_planner_gui
{

TypenameReference::TypenameReference(std::shared_ptr<pddl::Type> type) :
				m_type(type)
{
}

TypenameReference::~TypenameReference()
{
}

size_t TypenameReference::hash() const
{
	std::size_t h = 0;
	pddl::hash_combine(h, m_type->hash());
	return h;
}

bool TypenameReference::semanticEqual(const std::shared_ptr<Reference>& other) const
		{
	return *this == *other;
}

std::string TypenameReference::str(const std::shared_ptr<pddl::TypedObject>& obj) const
		{
	static const auto defaultO = pddl::DefaultTypedObjects::unknownObject();
	if (obj != defaultO)
		return "some " + m_type->getName() + " (" + obj->getName() + ")";

	return "some " + m_type->getName();
}

const std::shared_ptr<pddl::Type>& TypenameReference::getType() const
{
	return m_type;
}

} /* namespace goal_planner_gui */
