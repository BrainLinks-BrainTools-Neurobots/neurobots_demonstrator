/*
 * requirements.cpp
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/conditions/existential_condition.h>
#include <goal_planner_gui/pddl/conditions/universal_condition.h>
#include <goal_planner_gui/pddl/effects/conditional_effect.h>
#include <goal_planner_gui/pddl/effects/conjunctive_effect.h>
#include <goal_planner_gui/pddl/effects/universal_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/modules.h>
#include <goal_planner_gui/pddl/requirements.h>
#include <goal_planner_gui/pddl/translators.h>
#include <goal_planner_gui/pddl/tagable_object.h>

namespace pddl
{

bool Requirement::initialized = false;

void Requirement::getDependencies(const std::string& req,
		std::unordered_set<std::string>& dependencies)
{
	std::shared_ptr<ModuleDescription> module = ModuleDescription::getModule(req);
	dependencies.insert(req);
	for (auto& it : module->getDependencies())
		getDependencies(it, dependencies);
}

void Requirement::getDependencies(const std::vector<std::string>& req,
		std::unordered_set<std::string>& dependencies)
{
	for (auto& it : req)
		getDependencies(it, dependencies);
}

void Requirement::initAvailableModules()
{
	if (initialized)
		return;

	//register objects
	TagableObjectRegistry* reg = TagableObjectRegistry::getInstance();
	reg->reg("Conjunction", Conjunction::parse, Conjunction::tag);
	reg->reg("ConjunctiveEffect", ConjunctiveEffect::parse, ConjunctiveEffect::tag);
	reg->reg("Disjunction", Disjunction::parse, Disjunction::tag);
	reg->reg("ExistentialCondition", ExistentialCondition::parse, ExistentialCondition::tag);
	reg->reg("UniversalCondition", UniversalCondition::parse, UniversalCondition::tag);
	reg->reg("ConditionalEffect", ConditionalEffect::parse, ConditionalEffect::tag);
	reg->reg("UniversalEffect", UniversalEffect::parse, UniversalEffect::tag);

	//the basic strips module. Everyone needs this.
	ModuleDescription::createModule("strips", { { "con", std::string("Conjunction") }, { "coneff", std::string("ConjunctiveEffect") } });

	//empty modules for what is essentially hardcoded functionality
	ModuleDescription::createModule("typing", { });
	ModuleDescription::createModule("equality", { });
	ModuleDescription::createModule("negative-preconditions", { });
	ModuleDescription::createModule("derived-predicates", { });

	//modules for the various builtin preconditions and effects for adl
	ModuleDescription::createModule("disjunctive-preconditions", { { "clas", std::string("Disjunction") } });
	ModuleDescription::createModule("existential-preconditions", { { "clas", std::string("ExistentialCondition") } });
	ModuleDescription::createModule("universal-preconditions", { { "clas", std::string("UniversalCondition") } });
	ModuleDescription::createModule("conditional-effects", { { "ceff", std::string("ConditionalEffect") }, { "uneff", std::string("UniversalEffect") } });

	{
		boost::any o1 = std::vector<std::string> { "existential-preconditions", "universal-preconditions" };
		ModuleDescription::createModule("quantified-preconditions", { { "dependencies", o1 } });
	}
	{
		boost::any o1 = std::vector<std::string> { "strips", "typing", "equality", "negative-preconditions",
				"disjunctive-preconditions", "quantified-preconditions",
				"conditional-effects", "derived-predicates" };
		ModuleDescription::createModule("adl", { { "dependencies", o1 } });
	}

	{
		boost::any o1 = std::shared_ptr<ObjectFluentCompiler>(new ObjectFluentCompiler);
		ModuleDescription::createModule("object-fluents", { { "default_compiler", o1 } });
	}
}

} /* namespace pddl */

