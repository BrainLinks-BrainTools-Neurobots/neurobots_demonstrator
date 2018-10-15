/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: neurobots_world.h
 */

#ifndef HE22B4A9D_3F63_4932_8DA4_095402931E3E
#define HE22B4A9D_3F63_4932_8DA4_095402931E3E
#include <boost/variant.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

namespace goal_planner_gui
{

class NeurobotsObject
{
public:
	NeurobotsObject(const std::unordered_map<std::string, boost::variant<std::string, bool, std::vector<std::string>>>& attributes);

	std::unordered_map<std::string, boost::variant<std::string, bool, std::vector<std::string>>> m_attributes;

	const std::string& getString(const std::string& att);
	const bool& getBool(const std::string& att);
	const std::vector<std::string>& getStringVector(const std::string& att);

	bool isEqual(const std::string& arg, const boost::variant<std::string, bool, std::vector<std::string>>& other);
};

typedef std::shared_ptr<NeurobotsObject> NeurobotsObjectPtr;

class NeurobotsWorld
{
public:
	NeurobotsWorld(const std::vector<std::unordered_map<std::string, boost::variant<std::string, bool, std::vector<std::string>>> >&dict);
	virtual ~NeurobotsWorld();

	void add(const NeurobotsObjectPtr& object);
	void remove(const std::string& name);
	bool contains(const std::string& name);
	NeurobotsObjectPtr& get(const std::string& name);
//	void update(const std::string& name);

private:
	std::unordered_map<std::string, NeurobotsObjectPtr> m_objects;
};

}
/* namespace goal_planner_gui */

#endif /* HE22B4A9D_3F63_4932_8DA4_095402931E3E */
