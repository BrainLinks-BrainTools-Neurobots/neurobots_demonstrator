/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: other_reference.cpp
 */

#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/references/other_reference.h>
#include <algorithm>

namespace goal_planner_gui
{

OtherReference::OtherReference(const std::vector<std::shared_ptr<Reference> >& excludingRefs) :
				m_excludingRefs(excludingRefs)
{
}

OtherReference::~OtherReference()
{
}

size_t OtherReference::hash() const
{
	std::size_t h = 0;

	std::vector<size_t> hashes;
	hashes.reserve(m_excludingRefs.size());

	for (auto& c : m_excludingRefs)
	{
		hashes.push_back(c->hash());
	}

	std::sort(hashes.begin(), hashes.end());

	for (auto& it : hashes)
	{
		pddl::hash_combine(h, it);
	}

//	for (auto& it : m_excludingRefs)
//		pddl::hash_combine(h, it->hash());
	return h;
}

bool OtherReference::matches(const std::shared_ptr<pddl::TypedObject>& o) const
		{
	return !pddl::any<std::shared_ptr<Reference>>(m_excludingRefs, [&o](const std::shared_ptr<Reference>& r)
	{
		return r->matches(o);
	});
}

std::string OtherReference::str(const std::shared_ptr<pddl::TypedObject>& obj) const
		{
//	std::string other;
//	for (auto& it : m_excludingRefs)
//		other += it->str() + ", ";
//	if (!other.empty())
//		other.pop_back(); //remove last comma
//	return "other ref [exclude: " + other + "]";
	return "other";
}

} /* namespace goal_planner_gui */
