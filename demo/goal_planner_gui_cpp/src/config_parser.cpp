/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 5, 2017
 *      Author: kuhnerd
 * 	  Filename: config_parser.cpp
 */

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <goal_planner_gui/config_parser.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/utils.h>
#include <fstream>

namespace goal_planner_gui
{

ConfigParser::ConfigParser(const std::string& filename)
{
	std::ifstream file(filename);
	if (!file.is_open())
	{
		throw pddl::FileNotFoundExcetion(filename);
	}

	std::string currentGroup;
	std::vector<std::pair<std::string, std::string>> elements;
	std::string line;
	while (std::getline(file, line))
	{
		if (boost::algorithm::starts_with(line, "["))
		{
			if (!currentGroup.empty())
			{
				m_elements[currentGroup] = elements;
				elements.clear();
			}

			currentGroup = line;
			boost::algorithm::replace_all(currentGroup, "[", "");
			boost::algorithm::replace_all(currentGroup, "]", "");
			continue;
		}
		else if (line.empty())
		{
			continue;
		}
		else if (boost::algorithm::starts_with(line, ";"))
		{
			continue;
		}
		else
		{
			const auto idx = line.find_first_of(':');
			if (std::string::npos != idx)
			{
				std::string key = line.substr(0, idx);
				std::string value = line.substr(idx + 1);
				elements.push_back(std::make_pair(key, value));
			}
			else
			{
				throw pddl::DefaultParseError("Cannot read line in file " + filename + ": " + line);
			}
		}
	}

	if (!elements.empty())
	{
		m_elements[currentGroup] = elements;
	}
}

ConfigParser::~ConfigParser()
{
}

const std::vector<std::pair<std::string, std::string>>& ConfigParser::items(const std::string& section)
{
	if (CONTAINS_NOT(section, m_elements))
	{
		throw pddl::NotInMapException(section);
	}
	return m_elements[section];
}

} /* namespace goal_planner_gui */
