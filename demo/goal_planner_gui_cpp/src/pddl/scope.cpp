/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 28, 2017
 *      Author: kuhnerd
 * 	  Filename: scope.cpp
 */

#include <boost/algorithm/string.hpp>
#include <boost/variant/get.hpp>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/parse_context.h>
#include <memory>

namespace pddl
{

Scope::Scope(const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& constants,
		std::shared_ptr<Scope> parent)
{
	setParent(parent);
	for (auto& it : constants)
	{
		m_contentTypedObjects[it->getName()] = it;
	}
}

Scope::~Scope()
{
}

void Scope::lookup(const std::vector<std::shared_ptr<Term> >& input,
		std::vector<std::shared_ptr<Term> >& output)
{
	for (auto& arg : input)
	{
		if (arg->t() == TermTypes::FunctionTerm)
		{
			std::shared_ptr<FunctionTerm> funcArg = std::static_pointer_cast<FunctionTerm>(arg);
			std::vector<std::shared_ptr<Term> > r;
			lookup(funcArg->getArgs(), r);
			output.push_back(std::shared_ptr<FunctionTerm>(new FunctionTerm(funcArg->getFunction(), r)));
		}
		else if (arg->t() == TermTypes::ConstantTerm)
		{
			std::shared_ptr<TypedObject> res = get(arg);
			if (CONTAINS_NOT(res, m_termCache))
			{
				m_termCache[res].reset(new ConstantTerm(res));
			}
			output.push_back(m_termCache[res]);
		}
		else if (arg->t() == TermTypes::VariableTerm)
		{
			std::shared_ptr<Parameter> res = std::static_pointer_cast<Parameter>(get(arg));
			if (CONTAINS_NOT(res, m_termCache))
			{
				m_termCache[res].reset(new VariableTerm(res));
			}
			output.push_back(m_termCache[res]);
		}
		else
		{
			throw NotImplementedException(FILE_AND_LINE);
		}
	}

}

void Scope::setParent(std::shared_ptr<Scope> parent)
{
	m_parent = parent;
	if (parent)
	{
		m_predicates = parent->m_predicates;
		m_functions = parent->m_functions;
		m_types = parent->m_types;
		m_requirements = parent->m_requirements;
	}
	else
	{
		m_predicates.reset(new FunctionTable);
		m_functions.reset(new FunctionTable);
		m_types.clear();
		m_requirements.clear();
	}
}

void Scope::add(std::shared_ptr<TypedObject> object,
		bool check)
{
	std::string key = boost::algorithm::to_lower_copy(object->getName());
	if (!check || (check && !contains(key)))
		m_contentTypedObjects[key] = object;
}

std::shared_ptr<TypedObject> Scope::get(const std::string& key)
{
	std::string key2 = boost::algorithm::to_lower_copy(key);
	if (m_contentTypedObjects.find(key2) != m_contentTypedObjects.end())
	{
		return m_contentTypedObjects[key2];
	}
	else
	{
		if (!m_parent)
			throw NotInMapException(key2);
		return m_parent->get(key2);
	}
}

std::shared_ptr<TypedObject> Scope::get(const std::shared_ptr<Term>& term)
{
	std::string key;
	if (term->t() == TermTypes::VariableTerm)
	{
		key = std::static_pointer_cast<VariableTerm>(term)->getObject()->getName();
	}
	else if (term->t() == TermTypes::ConstantTerm)
	{
		auto k = std::static_pointer_cast<ConstantTerm>(term);
		if ((*k->getObj()->getType()) == (*DefaultTypes::numberType()))
		{
			return k->getObj();
		}
		key = k->getObj()->getName();
	}
	else
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}

	return get(key);
}

bool Scope::contains(const std::string& key) const
		{
	std::string key2 = boost::algorithm::to_lower_copy(key);
	if (m_contentTypedObjects.find(key2) != m_contentTypedObjects.end())
	{
		return true;
	}
	else
	{
		if (!m_parent)
			return false;
		else
			return m_parent->contains(key2);
	}
}

std::shared_ptr<ParseContext> Scope::getParseContext()
{
	if (m_parseContext)
		return m_parseContext;
	if (m_parent)
		return m_parent->getParseContext();

	//no context found, return a temporary parse context
	return std::shared_ptr<ParseContext>(new ParseContext());
}

bool Scope::smartInstantiateBegin(const SmartInstantiateFunction& func,
		const std::vector<std::shared_ptr<Parameter> >& args,
		const std::vector<TypedObjectUnorderedSet>& argList,
		const std::shared_ptr<Scope>& parent)
{
	m_smartInstantiater.reset(new SmartInstantiater);
	std::unordered_map<TypedObjectPtr, TypedObjectVector>& values = m_smartInstantiater->values;
	std::unordered_map<TypedObjectPtr, TypedObjectPtr, TypedObjectHasher>& mapping = m_smartInstantiater->mapping;
	std::list<std::pair<pddl::TypedObjectPtr, int>>& stack = m_smartInstantiater->stack;
	std::unordered_set<TypedObjectPtr, TypedObjectHasher>& remaining = m_smartInstantiater->remaining;
	remaining.insert(args.begin(), args.end());
	m_smartInstantiater->func = func;
	m_smartInstantiater->args = args;
	m_smartInstantiater->argList = argList;
	m_smartInstantiater->parent = parent;
	m_smartInstantiater->currentIndex = -1;
	m_smartInstantiater->i = 0;
	m_smartInstantiater->continueLoop = false;

	//partial mapping not required in our case

	uninstantiate();

	if (parent)
	{
		m_originalParent = m_parent;
		setParent(parent);
	}

	ASSERT(args.size() == argList.size());
	for (size_t i = 0; i < args.size(); ++i)
	{
		auto& a = args[i];
		auto& l = argList[i];

		if (l.empty())
		{
			return false;
		}

		//		LOG_INFO(a->str() << " " << l.size());

		values[a].clear();
		values[a].insert(values[a].end(), l.begin(), l.end());

		if (l.size() == 1)
		{
			mapping[a] = *l.begin();
			stack.push_back(std::make_pair(a, 0));
			a->instantiate(*l.begin());
			remaining.erase(a);
		}
	}

	return true;
}

bool Scope::smartInstantiateNext(std::unordered_map<TypedObjectPtr, TypedObjectPtr, TypedObjectHasher>& result)
{
	auto& values = m_smartInstantiater->values;
	auto& mapping = m_smartInstantiater->mapping;
	auto& stack = m_smartInstantiater->stack;
	auto& remaining = m_smartInstantiater->remaining;
	auto& args = m_smartInstantiater->args;
	auto& currentArg = m_smartInstantiater->currentArg;
	auto& currentIndex = m_smartInstantiater->currentIndex;
	auto& func = m_smartInstantiater->func;
	auto& continueLoop = m_smartInstantiater->continueLoop;

	auto& params = m_smartInstantiater->loopData.params;
	auto& next = m_smartInstantiater->loopData.next;
	auto& nextVal = m_smartInstantiater->loopData.nextVal;

	if (!m_smartInstantiater->continueLoop)
	{
		currentIndex = -1;
	}

	while (true)
	{
		//finish previous loop (due to yield in python)
		if (m_smartInstantiater->continueLoop)
		{
			next = DefaultTypedObjects::__returnObjectFalse();
			if (!smartInstantiateNext2())
			{
				m_smartInstantiater.reset();
				return false;
			}
			m_smartInstantiater->continueLoop = false;
		}

		params.clear();

		for (auto& s : stack)
		{
			params.push_back(s.first);
		}

		nextVal.reset();

		func(mapping, params, next, nextVal);

//		LOG_INFO(next);

//if next is not None and next not in values:
		if (next && CONTAINS_NOT(next, values))
		{
			next = DefaultTypedObjects::__returnObjectTrue();
			nextVal.reset();
		}

		//if next == True and len(stack) == len(args):
//		LOG_INFO(next << " " << stack.size() << " " << args.size())
		if (next == DefaultTypedObjects::__returnObjectTrue() && stack.size() == args.size())
		{
			result = mapping;
			m_smartInstantiater->continueLoop = true;
			return true;
		}
		//elif nextval:
		else if (nextVal)
		{
			ASSERT(CONTAINS(next, values));
			auto l = values[next];
			if (std::find(l.begin(), l.end(), nextVal) == l.end())
			{
				next = DefaultTypedObjects::__returnObjectFalse();
			}
			else
			{
				currentArg = next;
				currentIndex = -1;
			}
		}
		//elif next:
		else if (next && next != DefaultTypedObjects::__returnObjectFalse())
		{
			if (next == DefaultTypedObjects::__returnObjectTrue())
			{
				next = *remaining.begin();
			}

			currentArg = next;
			currentIndex = 0;

			//next is a typed object!
			if (!values[currentArg].empty())
			{
				nextVal = values[currentArg][currentIndex];
			}
			else
			{
				next.reset();
			}
		}

		if (!smartInstantiateNext2())
		{
			m_smartInstantiater.reset();
			return false;
		}
	}

	m_smartInstantiater.reset();
	return false;
}

void Scope::uninstantiate()
{
	for (auto& val : m_contentTypedObjects)
	{
		if (isInstanceSharedCast(val.second, par, Parameter))
		{
			par->instantiate(std::shared_ptr<TypedObject>());
		}
	}

	if (m_originalParent)
	{
		setParent(m_originalParent);
		m_originalParent.reset();
	}
}

std::string Scope::str() const
{
	return "Scope";
}

bool Scope::smartInstantiateNext2()
{
	auto& params = m_smartInstantiater->loopData.params;
	auto& next = m_smartInstantiater->loopData.next;
	auto& nextVal = m_smartInstantiater->loopData.nextVal;

	auto& values = m_smartInstantiater->values;
	auto& mapping = m_smartInstantiater->mapping;
	auto& stack = m_smartInstantiater->stack;
	auto& remaining = m_smartInstantiater->remaining;
	auto& currentArg = m_smartInstantiater->currentArg;
	auto& currentIndex = m_smartInstantiater->currentIndex;

	if (!next || next == DefaultTypedObjects::__returnObjectFalse())
	{
		currentIndex = -1;
//		LOG_INFO(values.size() << " values:");
//		for (auto& v : values)
//		{
//			LOG_INFO(" -> " << v.first);
//		}
//		LOG_INFO("current: " << currentArg);
//		ASSERT(CONTAINS(currentArg, values))
		while (currentIndex == -1 || currentIndex >= values[currentArg].size())
		{
			if (stack.empty())
			{
				uninstantiate();
				m_smartInstantiater.reset();
				return false;
			}

			auto& p = stack.back();
			stack.pop_back();

			currentArg = p.first;
			currentIndex = p.second;

			mapping.erase(currentArg);
			remaining.insert(currentArg);
			currentArg->instantiate(pddl::TypedObjectPtr());

			if (currentIndex != -1)
			{
				currentIndex += 1;
			}

			ASSERT(CONTAINS(currentArg, values))
		}

		nextVal = values[currentArg][currentIndex];
//		LOG_INFO(str() << " " << currentArg << " " << currentIndex << " " << nextVal)
	}

//	pddl::TypedObjectUnorderedSet test;
//	for (auto& s: stack) {
//		test.insert(s.first);
//	}
//	ASSERT(CONTAINS_NOT(currentArg, test));

//	LOG_INFO("add " << currentArg);

	mapping[currentArg] = nextVal;
	currentArg->instantiate(nextVal);
	remaining.erase(currentArg);
	stack.push_back(std::make_pair(currentArg, currentIndex));

	return true;
}

void Scope::instantiate(const std::unordered_map<pddl::ParameterPtr, pddl::TypedObjectPtr, pddl::TypedObjectHasher>& mapping,
		const std::shared_ptr<Scope>& parent)
{
	if (parent)
	{
		m_originalParent = m_parent;
		setParent(parent);
	}

	for (auto& it : mapping)
	{
		auto& key = it.first;
		auto& val = it.second;

		key->instantiate(val);
	}
}

}

/* namespace pddl */

