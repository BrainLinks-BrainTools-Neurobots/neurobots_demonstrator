/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: domain.h
 */

#ifndef HA376D5DA_B8EA_47F4_B415_6F4090038029
#define HA376D5DA_B8EA_47F4_B415_6F4090038029
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>
#include <unordered_set>

namespace pddl
{

class Element;
class Type;
class TypedObject;
class Predicate;
class Function;
class Action;
class Axiom;
class ParseContext;
class ModuleDescription;

class Domain: public Scope
{
public:
	Domain(const std::string& name,
			const std::unordered_set<std::string>& requirements,
			const std::unordered_map<std::string, std::shared_ptr<Type>>& types,
			const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& constants,
			const std::shared_ptr<FunctionTable>& predicates,
			const std::shared_ptr<FunctionTable>& functions,
			const std::vector<std::shared_ptr<Action>>& actions,
			const std::vector<std::shared_ptr<Axiom>>& axioms);
	virtual ~Domain();

	static std::shared_ptr<Domain> parse(const std::shared_ptr<Element>& root);
	static std::shared_ptr<Domain> loadDomain(const std::string& filename);

	virtual void getAllObjects(const std::shared_ptr<Type> type,
			std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& result);
	std::shared_ptr<Action> getAction(const std::string& name);

	std::string m_name;
	std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> m_constants;

	std::unordered_map<std::string, std::shared_ptr<Action>> m_name2action;
	std::vector<std::shared_ptr<Action>> m_actions;
	std::vector<std::shared_ptr<Axiom>> m_axioms;
	std::unordered_map<std::shared_ptr<Type>, std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>, TypeHasher> m_objectsByType;

private:
	void registerModuleHandlers(std::shared_ptr<ModuleDescription> m);
	void getModules(std::vector<std::shared_ptr<ModuleDescription>>& modules);
};

POINTER_DEF(Domain);

} /* namespace pddl */

#endif /* HA376D5DA_B8EA_47F4_B415_6F4090038029 */
