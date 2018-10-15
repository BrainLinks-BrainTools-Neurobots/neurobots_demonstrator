/*
 * requirements.h
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_REQUIREMENTS_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_REQUIREMENTS_H_
#include <string>
#include <unordered_set>
#include <vector>

namespace pddl
{

class Requirement
{
public:
	static void getDependencies(const std::string& req,
			std::unordered_set<std::string>& dependencies);
	static void getDependencies(const std::vector<std::string>& req,
			std::unordered_set<std::string>& dependencies);

	static void initAvailableModules();

private:
	static bool initialized;
};

} /* namespace pddl */

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_REQUIREMENTS_H_ */
