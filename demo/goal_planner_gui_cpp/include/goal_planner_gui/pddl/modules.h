/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: modules.h
 */

#ifndef H1507DB0E_53C6_45E0_BCF2_73E6C4EAC8A6
#define H1507DB0E_53C6_45E0_BCF2_73E6C4EAC8A6
#include <boost/any.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace pddl
{

class Type;
class Predicate;
class Function;
class BaseElement;
class Element;
class Scope;

class ModuleDescription
{
public:
	typedef std::unordered_map<std::string, boost::any> OptionMap;

public:
	ModuleDescription(const OptionMap& options);
	virtual ~ModuleDescription();

	static std::shared_ptr<ModuleDescription> getModule(const std::string& name);
	static void createModule(const std::string& name,
			const OptionMap& options);

	void findTaggedElements(const OptionMap& options);

	const std::vector<std::string>& getDependencies() const;
	const std::string& getModuleName() const;
	void setModuleName(const std::string& moduleName);
	const std::string& getName() const;
	void setName(const std::string& name);
	const std::vector<std::shared_ptr<Type> >& getTypes() const;
	const std::vector<std::shared_ptr<Function> >& getFunctions() const;
	const std::vector<std::shared_ptr<Predicate> >& getPredicates() const;
	const std::unordered_map<std::string, std::vector<std::string> >& getParseClasses() const;

private:
	std::vector<std::string> m_dependencies;
	std::string m_name;
	std::string m_moduleName;
	std::vector<std::shared_ptr<Type>> m_types;
	std::vector<std::shared_ptr<Predicate>> m_predicates;
	std::vector<std::shared_ptr<Function>> m_functions;
	std::unordered_map<std::string, std::vector<std::string>> m_parseClasses;

	static std::unordered_map<std::string, std::shared_ptr<ModuleDescription>> registeredModules;
};

}
/* namespace pddl */

#endif /* H1507DB0E_53C6_45E0_BCF2_73E6C4EAC8A6 */
