/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: functions.cpp
 */

#include <boost/algorithm/string/trim.hpp>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/visitors.h>
#include <goal_planner_gui/pddl/parser.h>

namespace pddl
{

Function::Function(const std::string& name,
		const std::vector<std::shared_ptr<Parameter> >& parameters,
		const std::shared_ptr<Type>& type,
		bool builtin,
		int function_scope) :
				m_name(name),
				m_args(parameters),
				m_type(type),
				m_builtin(builtin),
				m_functionScope(function_scope),
				m_arity(parameters.size()),
				m_hash(-1)
{
}

Function::~Function()
{
}

bool Function::isModal() const
{
	for (auto& it : m_args)
		if (it->getType()->t() == Types::FunctionType)
			return true;
	return false;
}

bool Function::operator ==(const Function& rhs) const
		{
	return hash() == rhs.hash();
}

bool Function::operator !=(const Function& rhs) const
		{
	return !(*this == rhs);
}

std::shared_ptr<Function> Function::copy()
{
	std::vector<std::shared_ptr<Parameter>> args;
	COPY_VECTOR_CAST(m_args, args, Parameter);

	return std::shared_ptr<Function>(new Function(m_name, args, m_type->copy(), m_builtin, m_functionScope));
}

size_t Function::hash() const
{
	if (m_hash == -1)
	{
		m_hash = 0;
		hash_combine(m_hash, m_name, m_type->hash());
		for (auto& k : m_args)
			hash_combine(m_hash, k->hash());
	}
	return m_hash;
}

std::string Function::str() const
{
	std::string args;
	for (auto& k : m_args)
		args += k->str() + " ";
	boost::algorithm::trim(args);
	return "(" + m_name + " " + args + ") - " + m_type->str();
}

const std::vector<std::shared_ptr<Parameter> >& Function::getArgs() const
{
	return m_args;
}

const std::vector<std::shared_ptr<Type> >& Function::getArgTypes() const
{
	if (m_args.size() != m_argTypes.size())
	{
		m_argTypes.clear();
		m_argTypes.reserve(m_args.size());
		for (auto& it : m_args)
		{
			m_argTypes.push_back(it->getType());
		}
	}
	return m_argTypes;
}

const std::string& Function::getName() const
{
	return m_name;
}

std::shared_ptr<Type> Function::getType() const
{
	return m_type;
}

int Function::getFunctionScope() const
{
	return m_functionScope;
}

bool Function::isBuiltin() const
{
	return m_builtin;
}

std::shared_ptr<Function> Function::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Type> type,
		std::unordered_map<std::string, std::shared_ptr<Type> >& types)
{
	const std::string name = it->get(Element::ExpectedTypeTerminal, "function identifier")->token.string;

	std::vector<std::shared_ptr<Parameter>> args;
	Function::parseArgumentList(it, types, args);
	return std::shared_ptr<Function>(new Function(name, args, type));
}

void Function::parseArgumentList(std::shared_ptr<Element> it,
		std::unordered_map<std::string, std::shared_ptr<Type> > typeDict,
		std::vector<std::shared_ptr<Parameter>>& result,
		std::shared_ptr<Scope> parentScope,
		std::vector<std::shared_ptr<Parameter>> previousParams,
		bool requiredType)
{
	std::shared_ptr<Scope> tempScope(new Scope( { }, parentScope));

	auto leftFunc = [](std::shared_ptr<Element> elem)
	{
		if (elem->token.string[0] != '?')
		{
			throw UnexpectedTokenError(elem->token, "parameter name");
		}
		return elem;
	};

	auto rightFunc1 = [](std::shared_ptr<Element> elem,
			std::unordered_map<std::string, std::shared_ptr<Type> > typeDict,
			std::shared_ptr<Scope> parentScope)
	{
		return Type::parse(elem, typeDict, parentScope);
	};

	auto rightFunc = std::bind(rightFunc1, std::placeholders::_1, typeDict, parentScope);

	std::vector<std::vector<std::shared_ptr<Element>>> leftResult;
	std::vector<std::shared_ptr<Type>> rightResult;
	Parser::parseTypedList<std::shared_ptr<Element>, std::shared_ptr<Type>>(
			it->current(), it->end(), leftFunc, rightFunc, leftResult, rightResult, "parameter name", "type specification", requiredType);

	std::vector<std::string> names;
	for (auto& p : previousParams)
		names.push_back(p->getName());

	for (size_t i = 0; i < leftResult.size(); ++i)
	{
		auto& params = leftResult[i];
		auto& type = rightResult[i];
		if (!type)
			type = DefaultTypes::objectType();

		for (auto& p : params)
		{
			if (std::find(names.begin(), names.end(), p->token.string) != names.end())
				throw ParseError(p->token, "Duplicate parameter name: " + p->token.string);

			std::shared_ptr<Parameter> param(new Parameter(p->token.string, type));
			result.push_back(param);
			names.push_back(p->token.string);
			tempScope->add(param);
		}
	}
}

std::shared_ptr<Function> Function::getFunction(const std::shared_ptr<Literal>& lit)
{
	if (isFunctional(lit))
	{
		if (isInstanceSharedCast(lit->m_args[0], f, FunctionTerm))
		{
			return f->getFunction();
		}
	}
	return lit->m_predicate;
}

bool Function::isFunctional(const std::shared_ptr<Literal>& lit)
{
	return CONTAINS(lit->m_predicate, Builtin::functionalOps());
}

} /* namespace pddl */

