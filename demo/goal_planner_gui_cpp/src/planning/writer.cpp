/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Feb 15, 2018
 *      Author: kuhnerd
 * 	  Filename: writer.cpp
 */

#include <boost/algorithm/string/trim.hpp>
#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/conditions/existential_condition.h>
#include <goal_planner_gui/pddl/conditions/intermediate_condition.h>
#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/conditions/preference_condition.h>
#include <goal_planner_gui/pddl/conditions/universal_condition.h>
#include <goal_planner_gui/pddl/effects/conditional_effect.h>
#include <goal_planner_gui/pddl/effects/conjunctive_effect.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/effects/universal_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/planning/writer.h>
#include <fstream>

namespace goal_planner_gui
{

Writer::Writer(const std::string& fileDomain,
		const std::string& fileProblem) :
				m_filenameDomain(fileDomain),
				m_filenameProblem(fileProblem)
{
}

Writer::~Writer()
{
}

bool Writer::write(const pddl::ProblemPtr& problem,
		const pddl::DomainPtr& domain,
		const pddl::ConditionPtr& goal)
{
	std::ofstream p(m_filenameProblem);
	std::ofstream d(m_filenameDomain);

	if (!p.is_open())
	{
		LOG_ERROR("Cannot write file " << m_filenameProblem);
		return false;
	}

	if (!d.is_open())
	{
		LOG_ERROR("Cannot write file " << m_filenameDomain);
		return false;
	}

	//write domain
	std::shared_ptr<pddl::FunctionTable> newPredicates;
	writeDomain(d, domain, newPredicates);
	d.close();

	//write problem
	writeProblem(p, problem, goal, newPredicates);
	p.close();

	return true;
}

bool Writer::writeDomain(std::ofstream& file,
		const pddl::DomainPtr& domain,
		std::shared_ptr<pddl::FunctionTable>& newPredicates)
{
	file << "(define (domain " + domain->m_name + ")\n\n";

	//requirements
//	file << "(:requirements";
//	for (auto& it : domain->m_requirements)
//	{
//		file << " :" + it;
//	}
//	file << ")\n\n";

	file << "(:requirements :quantified-preconditions :strips :equality "
			":derived-predicates :negative-preconditions :existential-preconditions "
			":conditional-effects :typing :adl :disjunctive-preconditions "
			":universal-preconditions)\n\n";

	//types
	writeTypes(file, domain->m_types);
	file << "\n";

	//constants
	if (!domain->m_constants.empty())
	{
		writeObjects(file, "constants", domain->m_constants);
	}
	file << "\n";

	//ObjectFluentCompiler (translators.py) combines all non number functions into predicates
	newPredicates.reset(new pddl::FunctionTable());
	std::shared_ptr<pddl::FunctionTable> newFunctions(new pddl::FunctionTable());
	for (auto& it : *domain->m_functions)
	{
		for (auto& f : it.second)
		{
			if (!f->getType()->equalOrSubtypeOf(pddl::DefaultTypes::numberType()))
			{
				auto args = f->getArgs();
				args.push_back(std::shared_ptr<pddl::Parameter>(new pddl::Parameter("?value", f->getType())));
				newPredicates->add(pddl::FunctionPtr(new pddl::Predicate(f->getName(), args)));
			}
			else
			{
				newFunctions->add(f);
			}
		}
	}

	for (auto& it : *domain->m_predicates)
	{
		for (auto& p : it.second)
		{
			newPredicates->add(p);
		}
	}

	//predicates
	writePredicates(file, newPredicates);
	file << "\n";

	//functions
	writeFunctions(file, newFunctions);
	file << "\n";

	//axioms
	for (auto& a : domain->m_axioms)
	{
		writeAxiom(file, a);
		file << "\n";
	}

	//actions
	for (auto& a : domain->m_actions)
	{
		writeAction(file, a, newPredicates);
		m_readFacts.clear();
		m_previousValues.clear();
		file << "\n";
	}

	file << ")";

	return true;
}

bool Writer::writeProblem(std::ofstream& file,
		const pddl::ProblemPtr& problem,
		const pddl::ConditionPtr& goal,
		const std::shared_ptr<pddl::FunctionTable>& predicates)
{
	file << "(define (problem " + problem->m_name + ")\n\n";
	file << "(:domain " + problem->getDomain()->m_name + ")\n\n";

	if (!problem->m_objects.empty())
	{
		writeObjects(file, "objects", problem->m_objects);
	}

	writeInit(file, problem->m_init, predicates);

	if (goal)
	{
		writeSection(file, ":goal", { getCondition(goal, predicates, 9) }, true);
	}

	file << "\n)\n";

	return true;
}

bool Writer::writeTypes(std::ofstream& file,
		std::unordered_map<std::string, std::shared_ptr<pddl::Type> >& types)
{
	pddl::TypeUnorderedSet toplevel { pddl::DefaultTypes::objectType(), pddl::DefaultTypes::numberType() };
	std::vector<std::string> strings;
	for (auto& it : types)
	{
		auto& type = it.second;
		if (type->t() != pddl::Types::Type)
		{
			continue;
		}

		if (!type->m_supertypes.empty())
		{
			for (auto& st : type->m_supertypes)
			{
				//only write the lowest supertype(s)
				bool res = false;
				for (auto& t : type->m_supertypes)
				{
					if (st->isSupertypeOf(t))
					{
						res = true;
						break;
					}
				}
				if (!res)
				{
					strings.push_back(type->getName() + " - " + st->getName());
				}
			}
		}
		else if (CONTAINS_NOT(type, toplevel))
		{
			toplevel.insert(type);
		}
	}

	toplevel.erase(pddl::DefaultTypes::objectType());
	toplevel.erase(pddl::DefaultTypes::numberType());

	std::string tl;
	for (auto& t : toplevel)
	{
		tl += t->getName() + " ";
	}

	boost::algorithm::trim(tl);
	strings.push_back(tl);

	return writeSection(file, ":types", strings);
}

bool Writer::writeObjects(std::ofstream& file,
		const std::string& name,
		const pddl::TypedObjectUnorderedSet& objects)
{
	std::vector<std::string> strings;
	std::unordered_map<pddl::TypePtr, pddl::TypedObjectUnorderedSet, pddl::TypeHasher> perType;
	for (auto& o : objects)
	{
		perType[o->getType()].insert(o);
	}

	for (auto& it : perType)
	{
		std::string ostr;
		for (auto& o : it.second)
		{
			ostr += o->getName() + " ";
		}
		boost::algorithm::trim(ostr);
		strings.push_back(ostr + " - " + getType(it.first));
	}

	return writeSection(file, ":" + name, strings);
}

bool Writer::writeSection(std::ofstream& file,
		const std::string& name,
		const std::vector<std::string>& content,
		const bool parent)
{
	if (parent)
	{
		file << "(";
	}
	file << name << "  ";
	bool first = true;

	std::string indent(name.size() + 3, ' ');

	for (auto& it : content)
	{
		if (!first)
			file << indent;
		file << it << "\n";
		first = false;
	}

	if (parent)
	{
		file << ")\n";
	}

	return true;
}

bool Writer::writePredicates(std::ofstream& file,
		const std::shared_ptr<pddl::FunctionTable>& predicates)
{
	std::vector<std::string> strings;

	LOG_ERROR("check at()...")
	for (auto& it : *predicates)
	{
		for (auto& it2 : it.second)
		{
			if (!it2->isBuiltin())
			{
				strings.push_back(getPredicate(it2));
			}
		}
	}

	if (!strings.empty())
	{
		std::sort(strings.begin(), strings.end());
		return writeSection(file, ":predicates", strings);
	}

	return true;
}

bool Writer::writeFunctions(std::ofstream& file,
		const std::shared_ptr<pddl::FunctionTable>& functions)
{
	std::vector<std::string> strings;

	for (auto& it : *functions)
	{
		for (auto& it2 : it.second)
		{
			if (!it2->isBuiltin())
			{
				strings.push_back(getFunction(it2));
			}
		}
	}

	if (!strings.empty())
	{
		std::sort(strings.begin(), strings.end());
		return writeSection(file, ":functions", strings);
	}

	return true;
}

std::string Writer::getType(const pddl::TypePtr& type)
{
	if (isInstanceSharedCast(type, t, pddl::CompositeType))
	{
		std::string s;
		for (auto& t2: t->getTypes())
		{
			s += getType(t2) + " ";
		}
		boost::algorithm::trim(s);

		return "(either " + s + ")";
	}
	if (isInstanceSharedCast(type, t, pddl::FunctionType))
	{
		return "(function " + getType(t->getType()) + ")";
	}
	if (isInstanceSharedCast(type, t, pddl::ProxyType))
	{
		return "(typeof " + t->getParameter()->getName() + ")";
	}
	return type->getName();
}

std::string Writer::getPredicate(const pddl::FunctionPtr& pred)
{
	return "(" + pred->getName() + " " + getTypedObjectList(pred->getArgs()) + ")";
}

std::string Writer::getFunction(const pddl::FunctionPtr& func)
{
	return "(" + func->getName() + " " + getTypedObjectList(func->getArgs()) + ") - " + getType(func->getType());
}

bool Writer::writeAxiom(std::ofstream& file,
		const std::shared_ptr<pddl::Axiom>& axiom)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

bool Writer::writeAction(std::ofstream& file,
		const std::shared_ptr<pddl::Action>& action,
		const std::shared_ptr<pddl::FunctionTable>& predicates)
{
	file << "(:action  " + action->m_name + "\n";

	if (!action->m_args.empty())
	{
		writeSection(file, "          :parameters", { "(" + getTypedObjectList(action->m_args) + ")" }, false);
	}

	if (action->m_precondition)
	{
		writeSection(file, "          :precondition", { getCondition(action->m_precondition, predicates, 26) }, false);
		for (auto& it : m_readFacts)
		{
			m_previousValues[it.first].push_back(it.second);
		}
	}

	if (action->m_effect)
	{
		writeSection(file, "          :effect", { "(" + getEffect(action->m_effect, predicates, 20) + std::string(19, ' ') + ")" }, false);
	}

	file << ")\n";

	return true;
}

std::string Writer::getCondition(const pddl::ConditionPtr& cond,
		const std::shared_ptr<pddl::FunctionTable>& predicates,
		int indent,
		bool addParantheses)
{
	if (isInstanceSharedCast(cond, c, pddl::LiteralCondition))
	{
		std::shared_ptr<pddl::LiteralCondition> condTranslated = c;
		//translate (= (a ?b) ?c) => (a ?b ?c)
		if (c->m_predicate == pddl::Builtin::equals())
		{
			if ((isInstance(c->m_args[0], pddl::VariableTerm) || isInstance(c->m_args[0], pddl::ConstantTerm)) &&
					(isInstance(c->m_args[1], pddl::VariableTerm) || isInstance(c->m_args[1], pddl::ConstantTerm)))
			{
				condTranslated = c;
			}
			else
			{
				dynamicSharedPointerCast(c->m_args[0], a0, pddl::FunctionTerm);
				auto argsA0 = a0->getArgs();
				argsA0.push_back(c->m_args.back());

				ASSERT(a0);

				auto newPred = predicates->get(a0->getFunction()->getName(), argsA0);

				if (!newPred)
				{
					throw pddl::Exception("newProd is NULL");
				}

				argsA0 = a0->getArgs();
				argsA0.push_back(c->m_args[1]);
				if (c->m_negated)
				{
					condTranslated.reset(new pddl::LiteralCondition(std::static_pointer_cast<pddl::Predicate>(newPred), argsA0, std::shared_ptr<pddl::Scope>(), true));
				}
				else
				{
					condTranslated.reset(new pddl::LiteralCondition(std::static_pointer_cast<pddl::Predicate>(newPred), argsA0, std::shared_ptr<pddl::Scope>(), c->m_negated));
					m_readFacts.push_back(
							{	c->m_args[0], c->m_args[1]});
				}
			}
		}

		return getLiteral(condTranslated, 0);
	}

	std::string head;
	std::vector<std::string> parts;
	if (isInstanceSharedCast(cond, c, pddl::Conjunction))
	{
		head = "and";
		for (auto& it: c->getParts())
		{
			parts.push_back(getCondition(it, predicates, indent + head.size() + 2, true));
		}
	}
	else if (isInstanceSharedCast(cond, c, pddl::Disjunction))
	{
		head = "or";
		for (auto& it: c->getParts())
		{
			parts.push_back(getCondition(it, predicates, indent + head.size() + 2, true));
		}
	}
	else if (isInstanceSharedCast(cond, c, pddl::UniversalCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(cond, c, pddl::ExistentialCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(cond, c, pddl::PreferenceCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(cond, c, pddl::IntermediateCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}

	std::string res = "(" + head + "  ";

	bool first = true;
	for (auto& it : parts)
	{
		if (!first)
		{
			res += std::string(indent + head.size() + 2, ' ');
		}
		else
		{
			boost::algorithm::trim(it);
			it += "  ";
		}
		res += it + "\n";
		first = false;
	}

	res += std::string(indent - 1, ' ') + ")";

	return res;
}

std::string Writer::getLiteral(const pddl::LiteralPtr& lit,
		int indent)
{
	std::string args;
	for (auto& arg : lit->m_args)
	{
		args += getTerm(arg) + " ";
	}
	boost::algorithm::trim(args);

	if (lit->m_negated)
	{
		return std::string(indent, ' ') + "(not (" + lit->m_predicate->getName() + " " + args + "))";
	}
	else
	{
		return std::string(indent, ' ') + "(" + lit->m_predicate->getName() + " " + args + ")";
	}
}

std::string Writer::getTerm(const pddl::TermPtr& term)
{
	if (term->getType()->equalOrSubtypeOf(pddl::DefaultTypes::numberType()))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}

	if (isInstanceSharedCast(term, t, pddl::ConstantTerm))
	{
		return t->getObj()->getName();
	}
	else if (isInstanceSharedCast(term, t, pddl::VariableTerm))
	{
		return t->getObject()->getName();
	}
	else if (isInstanceSharedCast(term, t, pddl::FunctionTerm))
	{
		std::string args;
		for (auto& arg : t->getArgs())
		{
			args += getTerm(arg) + " ";
		}
		boost::algorithm::trim(args);
		return "(" + t->getFunction()->getName() + " " + args + ")";
	}
	else
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
}

std::string Writer::getEffect(const pddl::EffectPtr& eff,
		const std::shared_ptr<pddl::FunctionTable>& predicates,
		int indent)
{
	std::string head;
	std::vector<std::string> parts;

	if (isInstanceSharedCast(eff, e, pddl::SimpleEffect))
	{
		std::string res;
		if (e->m_predicate == pddl::Builtin::assign() || e->m_predicate == pddl::Builtin::change())
		{
			dynamicSharedPointerCast(e->m_args[0], term, pddl::FunctionTerm);
			auto& value = e->m_args[1];

			auto argsTerm = term->getArgs();
			argsTerm.push_back(e->m_args.back());

			auto newPred = predicates->get(term->getFunction()->getName(), argsTerm);

			if (CONTAINS(term, m_previousValues))
			{
				for (auto& val: m_previousValues[term])
				{
					argsTerm = term->getArgs();
					argsTerm.push_back(val);
					std::shared_ptr<pddl::SimpleEffect> delEff(new pddl::SimpleEffect(std::static_pointer_cast<pddl::Predicate>(newPred), argsTerm, std::shared_ptr<pddl::Scope>(), true));
					if (isInstance(val, pddl::ConstantTerm) && isInstance(value, pddl::ConstantTerm))
					{
						if (val != value)
						{
							if (!res.empty())
							{
								res += "\n";
							}
							res += getLiteral(delEff, res.empty() ? 0 : indent - 1);
						}
					}
					else
					{
						if (!res.empty())
						{
							res += "\n";
						}
						res += getLiteral(delEff, res.empty() ? 0 : indent - 1);
					}
				}
				argsTerm = term->getArgs();
				argsTerm.push_back(value);

				if (!res.empty())
				{
					res += "\n";
				}
				res += getLiteral(std::shared_ptr<pddl::SimpleEffect>(new pddl::SimpleEffect(std::static_pointer_cast<pddl::Predicate>(newPred), argsTerm)), res.empty() ? 0 : indent - 1);
			}
			else if (term->getFunction()->getType() == pddl::DefaultTypes::booleanType())
			{
				throw pddl::NotImplementedException(FILE_AND_LINE);
			}
			else
			{
				throw pddl::NotImplementedException(FILE_AND_LINE);
			}
		}
		else
		{
			res = getLiteral(e, 0);
		}
		return res;
	}
	else if (isInstanceSharedCast(eff, e, pddl::ConjunctiveEffect))
	{
		head = "and";
		for (auto& it: e->getEffects())
		{
			parts.push_back(getEffect(it, predicates, indent + head.size() + 3));
		}
	}
	else if (isInstanceSharedCast(eff, e, pddl::UniversalEffect))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (isInstanceSharedCast(eff, e, pddl::ConditionalEffect))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}

	std::string res = head + "  ";

	bool first = true;
	for (auto& it : parts)
	{
		if (!first)
		{
			res += std::string(indent + head.size() + 2, ' ');
		}
		else
		{
			boost::algorithm::trim(it);
			it += "  ";
		}
		res += it + "\n";
		first = false;
	}

	return res;
}

bool Writer::writeInit(std::ofstream& file,
		const std::unordered_set<std::shared_ptr<pddl::InitLiteral>, pddl::InitLiteralHasher>& init,
		const std::shared_ptr<pddl::FunctionTable>& predicates)
{
	std::vector<std::string> strings;
	for (auto& i : init)
	{
		if (CONTAINS(i->m_predicate, pddl::Builtin::assignmentOps()))
		{
			dynamicSharedPointerCast(i->m_args[0], arg0, pddl::FunctionTerm);
			if (arg0->getFunction()->getType()->equalOrSubtypeOf(pddl::DefaultTypes::numberType()))
			{
				throw pddl::NotImplementedException(FILE_AND_LINE);
			}
			else
			{
				auto args = arg0->getArgs();
				args.push_back(i->m_args.back());
				auto newPred = predicates->get(arg0->getFunction()->getName(), args);

				args = arg0->getArgs();
				args.push_back(i->m_args[1]);

				strings.push_back(getLiteral(pddl::InitLiteralPtr(new pddl::InitLiteral(std::static_pointer_cast<pddl::Predicate>(newPred), args))));
			}
		}
		else
		{
			strings.push_back(getLiteral(i));
		}
	}

	return writeSection(file, ":init", strings);
}

std::string Writer::getTypedObjectList(const std::vector<pddl::ParameterPtr>& list)
{
	std::string res;
	for (auto& arg : list)
	{
		res += arg->getName() + " - " + getType(arg->getType()) + " ";
	}
	boost::algorithm::trim(res);
	return res;
}

}
/* namespace goal_planner_gui */
