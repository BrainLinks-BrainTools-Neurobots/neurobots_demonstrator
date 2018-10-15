/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: domain.cpp
 */

#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/axioms.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/modules.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/requirements.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/parse_context.h>

#include <memory>
#include <string>
#include <unordered_set>

namespace pddl
{

Domain::Domain(const std::string& name,
		const std::unordered_set<std::string>& requirements,
		const std::unordered_map<std::string, std::shared_ptr<Type>>& types,
		const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& constants,
		const std::shared_ptr<FunctionTable>& predicates,
		const std::shared_ptr<FunctionTable>& functions,
		const std::vector<std::shared_ptr<Action>>& actions,
		const std::vector<std::shared_ptr<Axiom>>& axioms) :
				Scope(constants, std::shared_ptr<Scope>()),
				m_name(name),
				m_constants(constants),
				m_actions(actions),
				m_axioms(axioms)
{
	m_requirements = requirements;
	m_types = types;
	m_predicates = predicates;
	m_functions = functions;
	m_parseContext.reset(new ParseContext());

	std::vector<std::shared_ptr<ModuleDescription>> modules;
	getModules(modules);
	for (auto& m : modules)
		registerModuleHandlers(m);
	registerModuleHandlers(ModuleDescription::getModule("strips"));
}

Domain::~Domain()
{
}

std::shared_ptr<Domain> Domain::parse(const std::shared_ptr<Element>& root)
{
	root->get("define");
	std::shared_ptr<Element> j = root->get(Element::ExpectedTypeList, "(domain 'domain identifier')");
	j->get("domain");
	std::string domname = j->get(Element::ExpectedTypeNone, "domain identifier")->token.string;

	std::unordered_map<std::string, std::shared_ptr<Type>> typeDict = Builtin::defaultTypesAsMap();
	std::shared_ptr<FunctionTable> domainPredicates(new FunctionTable( { Builtin::equals(), Builtin::assign(), Builtin::equalAssign() }));
	std::shared_ptr<FunctionTable> domainFunctions(new FunctionTable);
	std::vector<std::shared_ptr<Function>> defaultFunctions;

	std::shared_ptr<Element> req = root->get(Element::ExpectedTypeList, "requirement definition");
	req->get(":requirements");
	std::unordered_set<std::string> domRequirements;
	std::vector<std::shared_ptr<ModuleDescription>> domModules;
	std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> constants;

	//init available modules defined in requirements.cpp
	Requirement::initAvailableModules();

	for (auto it = req->current(); it != req->end(); ++it)
	{
		std::shared_ptr<Element> r = *it;
		if (!r->isTerminal() || r->token.string.empty() || r->token.string.at(0) != ':')
			throw UnexpectedTokenError(r->token, "requirement identifier");
		std::string requirement = r->token.string.substr(1); //remove :
		std::unordered_set<std::string> dependencies;
		Requirement::getDependencies(requirement, dependencies);
		for (auto& d : dependencies)
		{
			size_t oldSize = domRequirements.size();
			domRequirements.insert(d);
			if (oldSize != domRequirements.size())
				domModules.push_back(ModuleDescription::getModule(d));
		}
	}

	for (auto& module : domModules)
	{
		for (auto& type : module->getTypes())
			typeDict[type->getName()] = type;

		auto& preds = module->getPredicates();
		auto& funcs = module->getFunctions();

		for (auto& pred : preds)
			try
			{
				domainPredicates->add(pred);
			}
			catch (...)
			{
			}


		for (auto& func : funcs)
			try
			{
				domainFunctions->add(func);
			}
			catch (...)
			{
			}

		defaultFunctions.insert(defaultFunctions.end(), funcs.begin(), funcs.end());
		defaultFunctions.insert(defaultFunctions.end(), preds.begin(), preds.end());
	}

	std::shared_ptr<Domain> domain;

	for (auto it = root->current(); it != root->end(); ++it)
	{
		auto element = (*it);
		Token tag = element->get(Element::ExpectedTypeTerminal)->token;
		if (tag.string != ":types" && tag.string != ":constants" && tag.string != ":predicates" && tag.string != ":functions" && !domain)
		{
			LOG_INFO("Creating domain");
			domain.reset(new Domain(domname, domRequirements, typeDict, constants, domainPredicates, domainFunctions, { }, { }));
		}

		if (tag.string == ":types")
		{
			std::unordered_map<Token, Token> tlist;
			Type::parseTypeList(element->current(), element->end(), tlist);
			for (auto it : tlist)
			{
				const Token& key = it.first;
				const Token& value = it.second;

				if (key.string == "object")
					continue;

				if (typeDict.find(value.string) == typeDict.end())
					typeDict[value.string].reset(new Type(value.string, { DefaultTypes::objectType() }));

				if (typeDict.find(key.string) == typeDict.end())
					typeDict[key.string].reset(new Type(key.string, { DefaultTypes::objectType() }));

				if (typeDict[value.string] != DefaultTypes::objectType())
				{
					typeDict[key.string]->m_supertypes.push_back(typeDict[value.string]);
				}
			}
		}
		else if (tag.string == ":constants")
		{
			std::unordered_map<Token, Token> clist;
			Type::parseTypeList(element->current(), element->end(), clist);
			for (auto it : clist)
			{
				const Token& key = it.first;
				const Token& value = it.second;

				if (typeDict.find(value.string) == typeDict.end())
					throw ParseError(value, "undeclared type");

				constants.insert(std::shared_ptr<TypedObject>(new TypedObject(key.string, typeDict[value.string])));
			}

//			if (domain)
//			{
//				domain->m_constants = constants;
//				for (auto c : constants)
//					domain->add(c, true);
//			}
		}
		else if (tag.string == ":predicates")
		{
			for (auto it2 = element->current(); it2 != element->end(); ++it2)
			{
				auto elem = (*it2);
				if (elem->isTerminal())
					throw UnexpectedTokenError(elem->token, "predicate declaration");

				std::shared_ptr<Predicate> f = Predicate::parse(elem, typeDict);

				try
				{
					domainPredicates->add(f);
				}
				catch (Exception& e)
				{
					if (std::find(defaultFunctions.begin(), defaultFunctions.end(), f) == defaultFunctions.end())
						throw e;
				}
			}
		}
		else if (tag.string == ":functions")
		{
			auto leftFunc = [](std::shared_ptr<Element> elem)
			{
				if (elem->isTerminal())
				{
					throw UnexpectedTokenError(elem->token, "function declaration");
				}
				return elem;
			};

			auto rightFunc1 = [](std::shared_ptr<Element> elem,
					std::unordered_map<std::string, std::shared_ptr<Type> > typeDict)
			{
				return Type::parse(elem, typeDict);
			};

			auto rightFunc = std::bind(rightFunc1, std::placeholders::_1, typeDict);

			std::vector<std::vector<std::shared_ptr<Element>>> leftResult;
			std::vector<std::shared_ptr<Type>> rightResult;
			Parser::parseTypedList<std::shared_ptr<Element>, std::shared_ptr<Type>>(
					element->current(), element->end(), leftFunc, rightFunc, leftResult, rightResult, "function declarations", "type specification",
					true);

			for (size_t i = 0; i < leftResult.size(); ++i)
			{
				auto& funcs = leftResult[i];
				auto& type = rightResult[i];

				for (auto& elem : funcs)
				{
					std::shared_ptr<Function> f = Function::parse(elem, type, typeDict);
					try
					{
						domainFunctions->add(f);
					}
					catch (Exception& e)
					{
						if (std::find(defaultFunctions.begin(), defaultFunctions.end(), f) == defaultFunctions.end())
							throw e;
					}
				}
			}
		}
		if (tag.string == ":action")
		{
			if (domain == NULL)
			{
				throw Exception("Domain is not available yet! Please specify :action at the end.");
			}

			element->reset();
			domain->m_actions.push_back(Action::parse(element, domain));
		}
		if (tag.string == ":derived")
		{
			throw NotImplementedException(FILE_AND_LINE);
		}

	}

	if (!domain)
	{
		domain.reset(new Domain(domname, domRequirements, typeDict, constants, domainPredicates, domainFunctions, { }, { }));
	}

	return domain;
}

std::shared_ptr<Domain> Domain::loadDomain(const std::string& filename)
{
	std::shared_ptr<Parser> parser = Parser::parseFile(filename);
	return parse(parser->getRoot());
}

void Domain::getModules(std::vector<std::shared_ptr<ModuleDescription>>& modules)
{
	for (auto& req : m_requirements)
		modules.push_back(ModuleDescription::getModule(req));
}

void Domain::registerModuleHandlers(std::shared_ptr<ModuleDescription> m)
{
	for (auto& it : m->getParseClasses())
	{
		for (auto& it2 : it.second)
		{
			m_parseContext->addClass(it.first, it2);
//			LOG_INFO("Register module handle " << it.first);
		}
	}
}

void Domain::getAllObjects(const std::shared_ptr<Type> type,
		std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& result)
{
	if (CONTAINS_NOT(type, m_objectsByType))
	{
		auto& set = m_objectsByType[type];
		for (auto& o : m_constants)
		{
			if (o->isInstanceOf(type) && o != DefaultTypedObjects::unknownObject())
			{
				set.insert(o);
			}
		}
	}

	result = m_objectsByType[type];
}

std::shared_ptr<Action> Domain::getAction(const std::string& name)
{
	if (m_name2action.empty())
	{
		for (auto& a : m_actions)
		{
			m_name2action[a->m_name] = a;
		}
	}

	ASSERT(CONTAINS(name, m_name2action));

	return m_name2action[name];
}

} /* namespace pddl */

