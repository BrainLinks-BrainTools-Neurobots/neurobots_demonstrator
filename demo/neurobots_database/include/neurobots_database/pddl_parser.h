/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: pddl_parser.h
 */

#ifndef PDDL_PARSER_H_
#define PDDL_PARSER_H_
#include <string>

//Struct for Knowledge Base
#include <database_conversions/knowledge_base.h>

namespace neurobots_database
{

class PDDLParser
{
public:
	PDDLParser();
	virtual ~PDDLParser();

	//Load PDDL File
	bool load(const std::string& file,
			const std::string& fileDomain,
			World &world);
};

} /* namespace neurobots_database */

#endif /* PDDL_PARSER_H_ */
