/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: config_parser.h
 */

#ifndef HE166AA18_B9C5_410F_97D7_D3F4D795C29F
#define HE166AA18_B9C5_410F_97D7_D3F4D795C29F
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace goal_planner_gui
{

class ConfigParser
{
public:
	ConfigParser(const std::string& filename);
	virtual ~ConfigParser();

	const std::vector<std::pair<std::string, std::string>>& items(const std::string& section);

private:
	std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>> m_elements;
};

} /* namespace goal_planner_gui */

#endif /* HE166AA18_B9C5_410F_97D7_D3F4D795C29F */
