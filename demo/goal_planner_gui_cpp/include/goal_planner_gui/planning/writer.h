/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Feb 15, 2018
 *      Author: kuhnerd
 * 	  Filename: writer.h
 */

#ifndef H67E101EA_4C97_4127_B5F1_7FC4E9D7AD43
#define H67E101EA_4C97_4127_B5F1_7FC4E9D7AD43
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/axioms.h>
#include <goal_planner_gui/pddl/conditions/condition.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/types.h>
#include <string>

namespace goal_planner_gui
{

class GoalSpec;

class Writer
{
public:
	Writer(const std::string& fileDomain,
			const std::string& fileProblem);
	virtual ~Writer();

	bool write(const pddl::ProblemPtr& problem,
			const pddl::DomainPtr& domain,
			const pddl::ConditionPtr& goal);

private:
	bool writeDomain(std::ofstream& file,
			const pddl::DomainPtr& domain,
			std::shared_ptr<pddl::FunctionTable>& newPredicates);
	bool writeProblem(std::ofstream& file,
			const pddl::ProblemPtr& problem,
			const pddl::ConditionPtr& goal,
			const std::shared_ptr<pddl::FunctionTable>& predicates);
	bool writeTypes(std::ofstream& file,
			std::unordered_map<std::string, std::shared_ptr<pddl::Type>>& types);
	bool writeObjects(std::ofstream& file,
			const std::string& name,
			const pddl::TypedObjectUnorderedSet& objects);
	bool writeSection(std::ofstream& file,
			const std::string& name,
			const std::vector<std::string>& content,
			const bool parent = true);
	bool writePredicates(std::ofstream& file,
			const std::shared_ptr<pddl::FunctionTable>& predicates);
	bool writeFunctions(std::ofstream& file,
			const std::shared_ptr<pddl::FunctionTable>& functions);
	bool writeAxiom(std::ofstream& file,
			const std::shared_ptr<pddl::Axiom>& axiom);
	bool writeAction(std::ofstream& file,
			const std::shared_ptr<pddl::Action>& action,
			const std::shared_ptr<pddl::FunctionTable>& predicates);
	bool writeInit(std::ofstream& file,
			const std::unordered_set<std::shared_ptr<pddl::InitLiteral>, pddl::InitLiteralHasher>& init,
			const std::shared_ptr<pddl::FunctionTable>& predicates);

	std::string getType(const pddl::TypePtr& type);
	std::string getPredicate(const pddl::FunctionPtr& pred);
	std::string getFunction(const pddl::FunctionPtr& func);
	std::string getTerm(const pddl::TermPtr& term);
	std::string getCondition(const pddl::ConditionPtr& cond,
			const std::shared_ptr<pddl::FunctionTable>& predicates,
			int indent = 0,
			bool addParantheses = false);
	std::string getEffect(const pddl::EffectPtr& eff,
			const std::shared_ptr<pddl::FunctionTable>& predicates,
			int indent = 0);
	std::string getLiteral(const pddl::LiteralPtr& lit,
			int indent = 0);
	std::string getTypedObjectList(const std::vector<pddl::ParameterPtr>& list);

private:
	std::string m_filenameDomain;
	std::string m_filenameProblem;

	std::vector<std::pair<pddl::TermPtr, pddl::TermPtr>> m_readFacts;
	std::unordered_map<pddl::TermPtr, pddl::TermVector, pddl::TermHasher> m_previousValues;
};

} /* namespace goal_planner_gui */

#endif /* H67E101EA_4C97_4127_B5F1_7FC4E9D7AD43 */
