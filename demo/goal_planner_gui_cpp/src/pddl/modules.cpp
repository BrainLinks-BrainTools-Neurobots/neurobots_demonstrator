/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: modules.cpp
 */

#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/modules.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/tagable_object.h>
#include <goal_planner_gui/pddl/types.h>

namespace pddl
{

std::unordered_map<std::string, std::shared_ptr<ModuleDescription>> ModuleDescription::registeredModules;

ModuleDescription::ModuleDescription(const OptionMap& options)
{
	if (CONTAINS("dependencies", options))
		m_dependencies = boost::any_cast<std::vector<std::string>>(options.at("dependencies"));

	if (CONTAINS("default_types", options))
		m_types = boost::any_cast<std::vector<std::shared_ptr<Type>>>(options.at("default_types"));

	if (CONTAINS("default_predicates", options))
		m_predicates = boost::any_cast<std::vector<std::shared_ptr<Predicate>>>(options.at("default_predicates"));

	if (CONTAINS("default_functions", options))
		m_functions = boost::any_cast<std::vector<std::shared_ptr<Function>>>(options.at("default_functions"));

	findTaggedElements(options);
}

ModuleDescription::~ModuleDescription()
{
}

std::shared_ptr<ModuleDescription> ModuleDescription::getModule(const std::string& name)
{
	auto mod = registeredModules.find(name);
	if (mod != registeredModules.end())
		return mod->second;
	else
		throw Exception("Unknown module " + name + "!");
}

const std::vector<std::string>& ModuleDescription::getDependencies() const
{
	return m_dependencies;
}

const std::string& ModuleDescription::getModuleName() const
{
	return m_moduleName;
}

void ModuleDescription::setModuleName(const std::string& moduleName)
{
	m_moduleName = moduleName;
}

const std::string& ModuleDescription::getName() const
{
	return m_name;
}

void ModuleDescription::setName(const std::string& name)
{
	m_name = name;
}

const std::vector<std::shared_ptr<Type> >& ModuleDescription::getTypes() const
{
	return m_types;
}

const std::vector<std::shared_ptr<Function> >& ModuleDescription::getFunctions() const
{
	return m_functions;
}

const std::unordered_map<std::string, std::vector<std::string > >& ModuleDescription::getParseClasses() const
{
	return m_parseClasses;
}

const std::vector<std::shared_ptr<Predicate> >& ModuleDescription::getPredicates() const
{
	return m_predicates;
}

void ModuleDescription::createModule(const std::string& name,
		const OptionMap& options)
{
	std::shared_ptr<ModuleDescription> desc(new ModuleDescription(options));
	desc->setName(name);
	desc->setModuleName("");
	registeredModules[name] = desc;
}

void ModuleDescription::findTaggedElements(const OptionMap& options)
{
	auto reg = TagableObjectRegistry::getInstance();
	for (auto it : options)
	{
		try
		{
			std::string tag = boost::any_cast<std::string>(it.second);
			if (!reg->exists(tag))
				continue;

			LOG_INFO("Found tagable class with parser method: " << it.first << " of type " << tag);

			m_parseClasses[reg->tag(tag)].push_back(tag);

			return;
		}
		catch (const boost::bad_any_cast &)
		{
		}

		try
		{
			auto tags = boost::any_cast<std::vector<std::string>>(it.second);

			for (auto& tag : tags)
			{
				if (!reg->exists(tag))
					continue;

				LOG_INFO("Found tagable class with parser method: " << it.first << " of type " << tag);

				m_parseClasses[reg->tag(tag)].push_back(tag);
			}
		}
		catch (const boost::bad_any_cast &)
		{
		}
	}
}

}
/* namespace pddl */

