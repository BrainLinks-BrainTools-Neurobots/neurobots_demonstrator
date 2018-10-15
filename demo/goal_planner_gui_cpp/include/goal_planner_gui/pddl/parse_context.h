/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Oct 9, 2017
 *      Author: kuhnerd
 * 	  Filename: parse_context.h
 */

#ifndef H59E177B5_9EDF_4266_AF30_3557DCAEF2F4
#define H59E177B5_9EDF_4266_AF30_3557DCAEF2F4
#include <memory>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

namespace pddl
{

class Scope;
class TagableObject;

class ParseContext
{
public:
	ParseContext();
	virtual ~ParseContext();

	void addClass(const std::string& name,
			const std::string& c);

	void getHandlers(const std::string& name,
			const std::string& tag = "");

private:
	std::shared_ptr<Scope> m_scope;
	std::stack<std::shared_ptr<TagableObject>> m_elementStack;
	std::unordered_map<std::string, std::vector<std::string>> m_classes;
};

}
/* namespace pddl */

#endif /* H59E177B5_9EDF_4266_AF30_3557DCAEF2F4 */
