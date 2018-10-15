/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: relational_reference.cpp
 */

#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/references/other_reference.h>
#include <goal_planner_gui/references/relational_reference.h>
#include <memory>

namespace goal_planner_gui
{

RelationalReference::RelationalReference(const std::shared_ptr<pddl::Function>& function,
		const std::vector<std::shared_ptr<PartitionEntry>>& args,
		const std::shared_ptr<PartitionEntry>& value,
		const std::shared_ptr<ReferenceList>& refContext) :
				m_function(function),
				m_args(args),
				m_value(value),
				m_context(refContext),
				m_cachedValuesComputed(false),
				m_cachedPartitionsComputed(false)
{
//	for (auto& it: m_args) {
//		if (!it) {
//			throw pddl::Exception("arg is NULL");
//		}
//	}
//	LOG_INFO(args.size() << " " << function->getArgs().size());
//	if (args.size() != function->getArgs().size())
//		throw pddl::SizeException();
}

RelationalReference::~RelationalReference()
{
}

const std::vector<std::shared_ptr<PartitionEntry> >& RelationalReference::getArgs() const
{
	return m_args;
}

const std::shared_ptr<ReferenceList>& RelationalReference::getContext() const
{
	return m_context;
}

const std::shared_ptr<pddl::Function>& RelationalReference::getFunction() const
{
	return m_function;
}

const std::shared_ptr<PartitionEntry>& RelationalReference::getValue() const
{
	return m_value;
}

void RelationalReference::getChildren(std::vector<std::vector<std::shared_ptr<Reference>>>& children)
{
	const auto& fargs = m_function->getArgs();
	if (fargs.empty())
	{
		return;
	}

	if (isInstanceSharedCast(m_function, pred, pddl::Predicate))
	{
		if (fargs.size() < 2)
		{
			return;
		}
	}

	for (size_t i = 0; i < m_args.size() + 1; ++i)
	{
		std::vector<std::shared_ptr<Reference>> ch;
		getChildrenForArg(i, ch);
//		LOG_INFO_CONTAINER_TEXT_STR("Refs for " << i << ": ", ch);
		children.push_back(ch);
	}
}

std::shared_ptr<RelationalReference> RelationalReference::clone(const std::vector<std::shared_ptr<PartitionEntry> >& args,
		const std::shared_ptr<PartitionEntry>& value)
{
	return std::shared_ptr<RelationalReference>(new RelationalReference(m_function, args, value, m_context));
}

int RelationalReference::refCount()
{
	int count = 0;
	if (m_value)
	{
		std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher> counted;
		count += m_value->refCount(counted);
	}
	for (auto& pe : m_args)
	{
		if (pe)
		{
			std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher> counted;
			count += pe->refCount(counted);
		}
	}
	return count + 1;
}

void RelationalReference::getChildrenForArg(int i,
		std::vector<std::shared_ptr<Reference>>& children)
{
	if (!m_value)
	{
		return;
	}

	if (i == m_args.size())
	{

		if (m_value->getMatches().size() < 2)
		{
			return;
		}

		PartitionVector partitions;
		m_value->successors(partitions);
//		LOG_INFO_CONTAINER_TEXT_STR("Successors of " << m_value, partitions);

		for (auto& p : partitions)
		{
//			LOG_INFO("p child " << p->getChildren().size())
			for (auto& re : p->getChildren())
			{
				ASSERT(re);
				dynamicSharedPointerCast(re->getRef(), oref, OtherReference);
				if (!oref)
				{
					children.push_back(clone(m_args, re));
				}
			}
		}
	}
	else
	{
		if (!m_args[i])
		{
			return;
		}

		if (m_value->getMatches().size() < 2)
		{
			return;
		}

		std::vector<std::shared_ptr<PartitionEntry>> prefix(m_args.begin(), m_args.begin() + i);
		std::vector<std::shared_ptr<PartitionEntry>> suffix(m_args.begin() + i + 1, m_args.end());

		PartitionVector partitions;
		m_value->successors(partitions);
//		LOG_INFO_CONTAINER_TEXT_STR("Successors of " << m_value, partitions);

		for (auto& p : partitions)
		{
			for (auto& re : p->getChildren())
			{
				dynamicSharedPointerCast(re->getRef(), oref, OtherReference);
				if (!oref)
				{
					prefix.push_back(re);
					prefix.insert(prefix.end(), suffix.begin(), suffix.end());
					children.push_back(clone(prefix, m_value));
				}
			}
		}
	}
}

bool TypenameReference::matches(const std::shared_ptr<pddl::TypedObject>& o) const
		{
	return o->isInstanceOf(m_type);
}

size_t RelationalReference::hash() const
{
	std::size_t h = 0;
	pddl::hash_combine(h, m_function->hash());
	for (auto& it : m_args)
		pddl::hash_combine(h, it ? it->hash() : 0);
	if (m_value)
		pddl::hash_combine(h, m_value->hash());
	return h;
}

bool RelationalReference::matches(const std::shared_ptr<pddl::TypedObject>& o) const
		{
	return CONTAINS(o, getValues());
}

const pddl::TypedObjectUnorderedSet& RelationalReference::getValues() const
{
	if (m_cachedValuesComputed)
		return m_cachedValues;

	std::vector<pddl::TypedObjectUnorderedSet> argList;
	auto& fargs = m_function->getArgs();
	for (size_t i = 0; i < m_args.size(); ++i)
	{
		auto& arg = m_args[i];
		auto& param = fargs[i];
		if (arg)
		{
			argList.push_back(arg->getMatches());
		}
		else
		{
			pddl::TypedObjectUnorderedSet objects;
			m_context->getProblem()->getAllObjects(param->getType(), objects);
			argList.push_back(objects);
		}
	}

	if (pddl::any<pddl::TypedObjectUnorderedSet>(argList, [](const pddl::TypedObjectUnorderedSet& l)
	{	return l.empty();}))
	{
		m_cachedValues.clear();
		//missing line
		throw pddl::NotImplementedException(FILE_AND_LINE);
		m_cachedValuesComputed = true;
		return m_cachedValues;
	}

	auto& init = m_context->getInit();
//	LOG_INFO("init: " << init->str());

	if (!m_value)
	{
		m_cachedValues.clear();
		for (auto& it : *init)
		{
			auto& svar = it.first;
			auto& val = it.second;
			auto& sVarArgs = svar->getArgs();
			bool all = true;
			const int maxI = std::min(sVarArgs.size(), argList.size());
			for (size_t i = 0; i < maxI; ++i)
			{
				if (CONTAINS_NOT(sVarArgs[i], argList[i]))
				{
					all = false;
					break;
				}
			}

			if (all)
			{
				m_cachedValues.insert(val);
			}
		}
	}
	else
	{
		int i = pddl::index(m_args, std::shared_ptr<PartitionEntry>());
		auto& valueMatches = m_value->getMatches();

		pddl::TypedObjectUnorderedSet values;
		std::shared_ptr<pddl::TypedObject> booleanValue;
		if (isInstanceSharedCast(m_function, pred, pddl::Predicate))
		{
			if (valueMatches.size() >= 2)
			{
				//true for all values
				m_context->getProblem()->getAllObjects(pred->getArgs()[i]->getType(), m_cachedValues);
				m_cachedValuesComputed = true;
				return m_cachedValues;
			}
			else
			{
				ASSERT(valueMatches.size() == 1);
				booleanValue = *valueMatches.begin();
			}
		}

		for (auto& it : *init)
		{
			auto& svar = it.first;
			auto& val = it.second;
			auto& svarArgs = svar->getArgs();

			bool all = true;
			for (size_t j = 0; j < argList.size(); ++j)
			{
				if (i != j)
				{
					auto& argListJ = argList[j];
					if (std::find(argListJ.begin(), argListJ.end(), svarArgs[j]) == argListJ.end())
					{
						all = false;
						break;
					}
				}
			}

			if (svar->getFunction() == m_function && all)
			{
				if (booleanValue)
				{
					if (val == pddl::DefaultTypedObjects::trueObject())
					{
						values.insert(svarArgs[i]);
					}
				}
				else if (valueMatches.find(val) != valueMatches.end())
				{
					values.insert(svarArgs[i]);
				}
			}
		}

		if (booleanValue && booleanValue == pddl::DefaultTypedObjects::falseObject())
		{
			pddl::TypedObjectUnorderedSet res;
			m_context->getProblem()->getAllObjects(m_function->getArgs()[i]->getType(), res);
			values.clear();
			for (auto& o : res)
			{
				if (CONTAINS_NOT(o, values))
				{
					values.insert(o);
				}
			}
		}

		m_cachedValues = values;
	}

	m_cachedValuesComputed = true;
	return m_cachedValues;
}

const std::vector<pddl::TypedObjectUnorderedSet>& RelationalReference::getOptimisticPartition()
{
	if (!m_cachedPartitionsComputed)
	{
		int i = -1;
		if (!m_value)
		{
			if (isInstanceSharedCast(m_function, pred, pddl::Predicate))
			{
				return m_cachedPartitions;
			}
			i = m_args.size();
		}
		else
		{
			i = pddl::index(m_args, std::shared_ptr<PartitionEntry>());
		}

		ASSERT(i != -1);

		const auto& best = m_context->getBestFunctionPartitions()[m_function][i];

		auto& values = getValues();

		m_cachedPartitions.clear();
		for (auto& p : best)
		{
			m_cachedPartitions.push_back(pddl::TypedObjectUnorderedSet());
			auto& back = m_cachedPartitions.back();
			for (auto& o : p)
			{
				if (CONTAINS(o, values))
				{
					back.insert(o);
				}
			}
		}

//		LOG_INFO("Computed " << m_cachedPartitions.size() << " optimistic partitions");

		m_cachedPartitionsComputed = true;
	}
	return m_cachedPartitions;
}

std::string RelationalReference::str(const std::shared_ptr<pddl::TypedObject>& obj) const
		{
	std::string oname = obj != pddl::DefaultTypedObjects::unknownObject() ? obj->getName() : "x";
	std::string valstr = m_value ? m_value->str() : oname;
	std::string argstr;
	for (auto& a : m_args)
		argstr += (a ? a->str() : oname) + ",";
	if (!argstr.empty())
		argstr.pop_back();
	return m_function->getName() + "(" + argstr + ") = " + valstr;
}

bool RelationalReference::semanticEqual(const std::shared_ptr<Reference>& other) const
		{
	if (isInstanceSharedCast(other, otherRel, RelationalReference))
	{
		return m_function == otherRel->m_function;
	}
	else
	{
		return false;
	}
}

void RelationalReference::setArgs(const std::vector<std::shared_ptr<PartitionEntry> >& args)
{
	m_args = args;
}

void RelationalReference::setValue(const std::shared_ptr<PartitionEntry>& value)
{
	m_value = value;
}
}

/* namespace goal_planner_gui */
