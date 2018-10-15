/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: neurobots_world.cpp
 */

#include <goal_planner_gui/neurobots_world.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/utils.h>

namespace goal_planner_gui
{

NeurobotsObject::NeurobotsObject(const std::unordered_map<std::string, boost::variant<std::string, bool, std::vector<std::string> > >& attributes) :
				m_attributes(attributes)
{
	ASSERT(CONTAINS("name", m_attributes));
	ASSERT(CONTAINS("type", m_attributes));
}

const std::string& NeurobotsObject::getString(const std::string& att)
{
	return boost::get<std::string>(m_attributes[att]);
}

const bool& NeurobotsObject::getBool(const std::string& att)
{
	return boost::get<bool>(m_attributes[att]);
}

const std::vector<std::string>& NeurobotsObject::getStringVector(const std::string& att)
{
	return boost::get<std::vector<std::string>>(m_attributes[att]);
}

class my_visitor : public boost::static_visitor<std::string>
{
public:
	std::string operator()(std::string i) const
    {
        return i;
    }

	std::string operator()(bool i) const
       {
           return i ? "true" : "false";
       }

	std::string operator()(const std::vector<std::string> & str) const
    {
        std::string os = "[";
        for (auto& it: str)
        {
            os += " " + it;
        }
        os += "]";
        return os;
    }
};

bool NeurobotsObject::isEqual(const std::string& arg,
		const boost::variant<std::string, bool, std::vector<std::string> >& other)
{
	auto& value = m_attributes[arg];

	std::string res1 = boost::apply_visitor( my_visitor(), value );
	std::string res2 = boost::apply_visitor( my_visitor(), other );
	LOG_INFO("Check equal: " << arg << " (" << res1 << ") == " << res2);

	if (value.type() == typeid(std::string) && other.type() == typeid(std::string)
			&& boost::get<std::string>(value) == boost::get<std::string>(other))
	{
		return true;
	}
	else if (value.type() == typeid(bool) && other.type() == typeid(bool)
			&& boost::get<bool>(value) == boost::get<bool>(other))
	{
		return true;
	}
	else if (value.type() == typeid(std::vector<std::string>) && other.type() == typeid(std::vector<std::string>)
			&& boost::get<std::vector<std::string>>(value) == boost::get<std::vector<std::string>>(other))
	{
		return true;
	}

	return false;
}

NeurobotsWorld::NeurobotsWorld(const std::vector<std::unordered_map<std::string, boost::variant<std::string, bool, std::vector<std::string>>> >& dict)
{
	for (auto& obj: dict)
	{
		NeurobotsObjectPtr o(new NeurobotsObject(obj));
		m_objects[o->getString("name")] = o;
	}
}

NeurobotsWorld::~NeurobotsWorld()
{
}

void NeurobotsWorld::add(const NeurobotsObjectPtr& object)
{
	m_objects[object->getString("name")] = object;
}

void NeurobotsWorld::remove(const std::string& name)
{
	m_objects.erase(name);
}

bool NeurobotsWorld::contains(const std::string& name)
{
	return CONTAINS(name, m_objects);
}

NeurobotsObjectPtr& NeurobotsWorld::get(const std::string& name)
{
	return m_objects[name];
}

} /* namespace goal_planner_gui */

