/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 29, 2017
 *      Author: kuhnerd
 * 	  Filename: predicates.cpp
 */

#include <boost/algorithm/string/trim.hpp>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/types.h>

namespace pddl
{

Predicate::Predicate(const std::string& name,
		const std::vector<std::shared_ptr<Parameter> >& args,
		bool builtin,
		int functionScope) :
				Function(name, args, DefaultTypes::booleanType(), builtin, functionScope)
{
}

Predicate::~Predicate()
{
}

std::string Predicate::str() const
{
	std::string args;
	for (auto& k : m_args)
		args += k->str() + " ";
	boost::algorithm::trim(args);
	return "(" + m_name + " " + args + ") - " + m_type->str();
}

std::shared_ptr<Function> Predicate::copy()
{
	std::vector<std::shared_ptr<Parameter>> args;
	COPY_VECTOR_CAST(m_args, args, Parameter);

	return std::shared_ptr<Function>(new Predicate(m_name, args, m_builtin, m_functionScope));
}

std::shared_ptr<Predicate> Predicate::parse(std::shared_ptr<Element> it,
		std::unordered_map<std::string, std::shared_ptr<Type>>& types)
{
	std::string name = it->get(Element::ExpectedTypeTerminal, "predicate identifier")->token.string;

	std::vector<std::shared_ptr<Parameter>> args;
	Function::parseArgumentList(it, types, args);
	return std::shared_ptr<Predicate>(new Predicate(name, args));
}

} /* namespace pddl */

