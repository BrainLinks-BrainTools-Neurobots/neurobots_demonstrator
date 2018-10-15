/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: identity_reference.cpp
 */

#include <goal_planner_gui/references/identity_reference.h>
#include <goal_planner_gui/pddl/utils.h>

namespace goal_planner_gui
{

IdentityReference::IdentityReference(std::shared_ptr<pddl::TypedObject> obj) :
				m_obj(obj)
{
}

IdentityReference::~IdentityReference()
{
}

size_t IdentityReference::hash() const
{
	std::size_t h = 0;
	pddl::hash_combine(h, m_obj->hash());
	return h;
}

bool IdentityReference::matches(const std::shared_ptr<pddl::TypedObject>& o) const
		{
	return m_obj == o;
}

std::string IdentityReference::str(const std::shared_ptr<pddl::TypedObject>& obj) const
		{
	return m_obj->getName();
}

const std::shared_ptr<pddl::TypedObject>& IdentityReference::getObj() const
{
	return m_obj;
}

void IdentityReference::setObj(const std::shared_ptr<pddl::TypedObject>& obj)
{
	m_obj = obj;
}

} /* namespace goal_planner_gui */

