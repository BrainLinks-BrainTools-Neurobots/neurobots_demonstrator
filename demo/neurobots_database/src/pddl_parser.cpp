/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: pddl_parser.cpp
 */

#include <neurobots_database/pddl_parser.h>
#include <neurobots_database/attributes.h>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <unordered_map>

namespace neurobots_database
{

PDDLParser::PDDLParser()
{

}

PDDLParser::~PDDLParser()
{
}

bool PDDLParser::load(const std::string& file,
		const std::string& fileDomain,
		World &world)
{
	boost::smatch domainTypesMatch, domainMatch, objectBlockMatch, initMatch, initConnected;

	boost::regex domainRegex("\\(:domain ([a-zA-Z0-9-]+)\\)");
	boost::regex objectBlockRegex("\\(:objects\\s+([a-zA-Z0-9\\-;_\\n ]+?)\\)");
	boost::regex objectRegex("\\s*(.+?) - (.+?)\\n");
	boost::regex initRegex("\\(:init\\s*([a-zA-Z0-9\\-_ \\n\\(\\)=]+?\\))\\s*\\)");
	boost::regex initUnaryRegex("\\(\\s*([a-zA-Z0-9-]+)\\s+([a-zA-Z0-9-]+)\\)(?!\\s+[a-zA-Z])");
	boost::regex initBinaryRegex("\\(\\s*([a-zA-Z0-9-]+)\\s+([a-zA-Z0-9-]+)\\s+([a-zA-Z0-9-]+)\\)(?!\\s+[a-zA-Z])");
	boost::regex initAttributeRegex("\\(=\\s*\\(([_a-zA-Z0-9-]+)\\s+([_a-zA-Z0-9-]+)\\)\\s*([_a-zA-Z0-9-]+)\\)");
	boost::regex commentRegex("(.*?);(.*?)\n");

	boost::regex domainTypesBlockRegex("\\(:types\\s+([a-zA-Z0-9\\-;_\\n ]+?)\\)");
	boost::regex domainTypesRegex("\\s*(.+?) - (.+?)\\n");

	std::ifstream t(file);
	std::ifstream tDomain(fileDomain);

	std::string pddl((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());

	std::string pddlDomain((std::istreambuf_iterator<char>(tDomain)),
			std::istreambuf_iterator<char>());

	boost::sregex_iterator end;

	try
	{
		pddlDomain = boost::regex_replace(pddlDomain, commentRegex, "$1\n");
		pddl = boost::regex_replace(pddl, commentRegex, "$1\n");
//		std::cout <<pddlDomain <<std::endl;
//		std::cout <<pddl <<std::endl;

//first: get types from domain file
		boost::regex_search(pddlDomain, domainTypesMatch, domainTypesBlockRegex);

		//get types
		{
			std::string typesBlock = domainTypesMatch[1].str();
			boost::sregex_iterator it(typesBlock.begin(), typesBlock.end(), domainTypesRegex);
			std::cout << "types: " << std::endl;
			for (; it != end; ++it)
			{
				std::vector<std::string> types;
				std::string typesString = (*it)[1].str();
				//comments
				if (boost::starts_with(typesString, ";;"))
				{
					continue;
				}
				boost::split(types, typesString, boost::is_any_of(" "));
				std::cout << "|--> groups: ";
				for (auto& type : types)
				{
					std::cout << type << ", ";
				}
				std::cout << std::endl;
				std::cout << "|--> of type: " << (*it)[2] << "\n" << std::endl;

				std::string objectType = (*it)[2];
				if (objectType == "internal_locatable")
				{
					for (auto& type : types)
					{
						world.locatables.push_back(type);
					}
				}
				else if (objectType == "internal_perceptible")
				{
					for (auto& type : types)
					{
						world.perceptibles.push_back(type);
					}
				}
			}
		}

		//domain
		boost::regex_search(pddl, domainMatch, domainRegex);
		std::cout << "domain: " << domainMatch[1] << std::endl;
		world.domain = domainMatch[1];

		//Set Domain Name
		//"world.domain = domainMatch[1];

		//get object block
		boost::regex_search(pddl, objectBlockMatch, objectBlockRegex);

		//get objects
		{
			std::string objectBlock = objectBlockMatch[1].str();
			boost::sregex_iterator it(objectBlock.begin(), objectBlock.end(), objectRegex);
			std::cout << "objects: " << std::endl;
			for (; it != end; ++it)
			{
				std::vector<std::string> objects;
				std::string objectString = (*it)[1].str();
				//comments
				if (boost::starts_with(objectString, ";;"))
				{
					continue;
				}
				boost::split(objects, objectString, boost::is_any_of(" "));
				std::cout << "|--> objects: ";
				for (auto& object : objects)
				{
					std::cout << object << ", ";

					//add default attributes
					world.objects[object]["name"].push_back(object);
					world.objects[object]["type"].push_back((*it)[2]);
					world.objects[object]["locatable"].push_back(false);
					world.objects[object]["perceptible"].push_back(false);

					//add default attributes for locatables
					auto locatableIt = std::find(world.locatables.begin(), world.locatables.end(), (*it)[2]);
					if (locatableIt != world.locatables.end())
					{
						for (auto& attribute : Attributes::defaultAttributes["locatable"])
						{
							world.objects[object][attribute.first] = attribute.second;
						}
						world.objects[object]["locatable"][0] = true;
					}

					//add default attributes for perceptibles
					auto perceptiblesIt = std::find(world.perceptibles.begin(), world.perceptibles.end(), (*it)[2]);
					if (perceptiblesIt != world.perceptibles.end())
					{
						for (auto& attribute : Attributes::defaultAttributes["perceptible"])
						{
							world.objects[object][attribute.first] = attribute.second;
						}
						world.objects[object]["perceptible"][0] = true;
					}
				}
				std::cout << std::endl;
				std::cout << "|--> group: " << (*it)[2] << "\n" << std::endl;
			}
		}

		//get init block
		boost::regex_search(pddl, initMatch, initRegex);

		//init unary attributes
		std::string initBlock = initMatch[1].str();
		std::cout << "INIT BLOCK DEBUG = \n"<< initBlock << std::endl;
		boost::sregex_iterator it2(initBlock.begin(), initBlock.end(), initUnaryRegex);
		std::cout << "\n found " << (*it2).size() << " matches" << std::endl;
		std::cout << "unary: " << std::endl;
		for (; it2 != end; ++it2)
		{
			std::cout << "|--> '" << (*it2)[1] << "': '" << (*it2)[2] << "'" << std::endl;
			world.objects[(*it2)[2]][(*it2)[1]].push_back(true);
		}

		std::cout << std::endl;

		//init binary attributes
		boost::sregex_iterator it3(initBlock.begin(), initBlock.end(), initBinaryRegex);
		std::cout << "binary: " << std::endl;
		for (; it3 != end; ++it3)
		{
			std::cout << "|--> '" << (*it3)[1] << "': '" << (*it3)[2] << "' and '" << (*it3)[3] << "'" << std::endl;
			world.objects[(*it3)[2]][(*it3)[1]].push_back((*it3)[3]);
		}

		std::cout << std::endl;

		//init attribute objects
		boost::sregex_iterator it4(initBlock.begin(), initBlock.end(), initAttributeRegex);
		std::cout << "attributes: " << std::endl;
		for (; it4 != end; ++it4)
		{
			std::cout << "|--> attribute '" << (*it4)[1] << "' of object '" << (*it4)[2] << "' is '" << (*it4)[3] << "'" << std::endl;

			world.objects[(*it4)[2]][(*it4)[1]].push_back((*it4)[3]);
		}
	}
	catch (boost::regex_error& e)
	{
		std::cerr << "ERROR: " << e.code() << std::endl;
		exit(-1);
	}

	return true;
}

} /* namespace neurobots_database */
