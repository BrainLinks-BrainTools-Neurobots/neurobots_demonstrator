/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 29, 2017
 *      Author: kuhnerd
 * 	  Filename: function_table.cpp
 */

#include <boost/algorithm/string/case_conv.hpp>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/terms.h>

namespace pddl
{

FunctionTable::FunctionTable()
{

}

FunctionTable::FunctionTable(const std::vector<std::shared_ptr<Function> >& functions)
{
	for (auto& it : functions)
	{
		operator [](it->getName()).push_back(it);
	}
}

FunctionTable::~FunctionTable()
{
}

void FunctionTable::add(std::shared_ptr<Function> function)
{
	if (get(function->getName(), function->getArgs(), function->getFunctionScope()))
		throw Exception("A function with this name and arguments already exists: " + function->str());
	operator [](function->getName()).push_back(function);
}

void FunctionTable::remove(std::shared_ptr<Function> function)
{
	if (!contains(function))
		return;

	std::string k = boost::algorithm::to_lower_copy(function->getName());
	std::vector<std::shared_ptr<Function>>& functions = m_functions[k];
	for (auto it = functions.begin(); it != functions.end(); ++it)
		if ((**it) == *function)
			functions.erase(it);

	if (functions.empty())
		m_functions.erase(k);
}

bool FunctionTable::contains(const std::shared_ptr<Function>& key) const
		{
	return contains(key->getName());
}

bool FunctionTable::contains(const std::string& key) const
		{
	std::string k = boost::algorithm::to_lower_copy(key);
	return m_functions.find(k) != m_functions.end();
}

void FunctionTable::getAll(const std::string& name,
		const std::vector<std::shared_ptr<Term>>& args,
		std::vector<std::shared_ptr<Function>>& functions,
		int functionScope)
{
	std::vector<std::shared_ptr<Type>> argtypes;
	for (auto& it : args)
		argtypes.push_back(it->getType());
	getAll(name, argtypes, functions, functionScope);
}

void FunctionTable::getAll(const std::string& name,
		const std::vector<std::shared_ptr<TypedObject>>& args,
		std::vector<std::shared_ptr<Function>>& functions,
		int functionScope)
{
	std::vector<std::shared_ptr<Type>> argtypes;
	for (auto& it : args)
		argtypes.push_back(it->getType());
	getAll(name, argtypes, functions, functionScope);
}

void FunctionTable::getAll(const std::string& name,
		const std::vector<std::shared_ptr<Parameter>>& args,
		std::vector<std::shared_ptr<Function>>& functions,
		int functionScope)
{
	std::vector<std::shared_ptr<Type>> argtypes;
	for (auto& it : args)
		argtypes.push_back(it->getType());
	getAll(name, argtypes, functions, functionScope);
}

void FunctionTable::getAll(const std::string& name,
		const std::vector<std::shared_ptr<Type>>& argtypes,
		std::vector<std::shared_ptr<Function>>& functions,
		int functionScope)
{
	if (!contains(name))
	{
		functions.clear();
		return;
	}

	std::vector<std::shared_ptr<Function>>& fs = operator [](name);
	for (auto& f : fs)
	{
		auto& fargs = f->getArgs();

//		LOG_INFO("scope: " << f->getFunctionScope() << " " << functionScope)
		if (!(f->getFunctionScope() & functionScope))
		{
			continue;
		}

		if (argtypes.size() == fargs.size())
		{
			std::unordered_map<std::shared_ptr<Parameter>, std::shared_ptr<Type>, TypedObjectHasher> funcargs;
			bool matches = true;
			for (size_t i = 0; i < argtypes.size(); ++i)
			{
				auto t = argtypes[i];
				auto arg = fargs[i];
				std::shared_ptr<Type> argType = arg->getType();
				if (isInstanceSharedCast(argType, proxytype, ProxyType))
				{
					if (funcargs.find(proxytype->getParameter()) != funcargs.end())
					{
						argType = funcargs[proxytype->getParameter()];
					}
				}

//				LOG_INFO(t->t() << " is " << (t->equalOrSubtypeOf(argType) ? "" : "not") << " a subtype of " << argType->t())
				if (!t->equalOrSubtypeOf(argType))
				{
					matches = false;
					break;
				}

				if (isInstanceSharedCast(argType, ftype, FunctionType))
				{
					staticSharedPointerCast(t, ft, FunctionType);
					ASSERT(t);
					funcargs[arg] = ft->getType();
				}
			}

			if (matches)
				functions.push_back(f);
		}
	}

//	if (functions.empty())
//	{
//	}
}

std::shared_ptr<Function> FunctionTable::get(const std::string& name,
		const std::vector<std::shared_ptr<Term>>& args,
		int functionScope)
{
	std::vector<std::shared_ptr<Function>> functions;
	getAll(name, args, functions, functionScope);
	if (functions.size() == 1)
		return functions[0];
	return std::shared_ptr<Function>();
}

std::shared_ptr<Function> FunctionTable::get(const std::string& name,
		const std::vector<std::shared_ptr<TypedObject>>& args,
		int functionScope)
{
	std::vector<std::shared_ptr<Function>> functions;
	getAll(name, args, functions, functionScope);
	if (functions.size() == 1)
		return functions[0];
	return std::shared_ptr<Function>();
}

std::shared_ptr<Function> FunctionTable::get(const std::string& name,
		const std::vector<std::shared_ptr<Parameter>>& args,
		int functionScope)
{
	std::vector<std::shared_ptr<Function>> functions;
	getAll(name, args, functions, functionScope);
	if (functions.size() == 1)
		return functions[0];
	return std::shared_ptr<Function>();
}

std::shared_ptr<Function> FunctionTable::get(const std::string& name,
		const std::vector<std::shared_ptr<Type>>& args,
		int functionScope)
{
	std::vector<std::shared_ptr<Function>> functions;
	getAll(name, args, functions, functionScope);
	if (functions.size() == 1)
		return functions[0];
	return std::shared_ptr<Function>();
}

//std::vector<std::shared_ptr<Function>> FunctionTable::operator [](const std::shared_ptr<Function>& i) const
//		{
//	return operator [](i->getName());
//}

std::vector<std::shared_ptr<Function>>& FunctionTable::operator [](const std::shared_ptr<Function>& i)
{
	return operator [](i->getName());
}

//std::vector<std::shared_ptr<Function>> FunctionTable::operator [](const std::string& i) const
//		{
//	std::string k = boost::algorithm::to_lower_copy(i);
//	return m_functions[k];
//}

std::vector<std::shared_ptr<Function>>& FunctionTable::operator [](const std::string& i)
{
	return m_functions[boost::algorithm::to_lower_copy(i)];
}

std::unordered_map<std::string, std::vector<std::shared_ptr<Function> > >::iterator FunctionTable::begin() noexcept
{
	return m_functions.begin();
}

std::unordered_map<std::string, std::vector<std::shared_ptr<Function> > >::iterator FunctionTable::end() noexcept
{
	return m_functions.end();
}

std::unordered_map<std::string, std::vector<std::shared_ptr<Function> > >::const_iterator FunctionTable::begin() const noexcept
{
	return m_functions.begin();
}

std::string FunctionTable::str() const
{
	std::string res;
	for (auto& f : m_functions)
	{
		res += "- " + f.first + ":\n";
		for (auto& f2 : f.second)
		{
			res += "-- " + f2->str() + "\n";
		}
	}
	return res;
}

std::unordered_map<std::string, std::vector<std::shared_ptr<Function> > >::const_iterator FunctionTable::end() const noexcept
{
	return m_functions.end();
}

size_t FunctionTable::size() const
{
	return m_functions.size();
}

} /* namespace pddl */

