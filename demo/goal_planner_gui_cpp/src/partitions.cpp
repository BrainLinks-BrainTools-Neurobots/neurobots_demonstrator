/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: partitions.cpp
 */

#include <goal_planner_gui/alternative_partition_entry.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/references/other_reference.h>
#include <goal_planner_gui/references/relational_reference.h>

#include <numeric>
#include <cmath>

namespace goal_planner_gui
{

Partition::Partition() :
				m_hash(0),
				m_otherCount(-1),
				m_splitType(Other)
{
}

void Partition::init(const std::vector<std::shared_ptr<Reference> >& refs,
		const std::shared_ptr<PartitionEntry>& parent,
		const SplitType splitType,
		const std::shared_ptr<ReferenceList>& refContext)
{
	m_parent = parent;
	m_context = !parent ? refContext : parent->getContext();
	m_splitType = splitType;

	for (auto& r : refs)
		m_children.push_back(std::shared_ptr<PartitionEntry>(new PartitionEntry(r, shared_from_this())));
}

Partition::~Partition()
{
}

bool Partition::operator ==(const Partition& rhs) const
		{
	return hash() == rhs.hash();
}

bool Partition::operator !=(const Partition& rhs) const
		{
	return !(*this == rhs);
}

size_t Partition::hash() const
{
	if (m_hash == 0)
	{
		std::vector<size_t> hashes;
		hashes.reserve(m_children.size());

		for (auto& c : m_children)
		{
			hashes.push_back(c->hash());

//			for (auto& m : c->getMatches())
//			{
//				pddl::hash_combine(m_hash, m->hash());
//			}
		}

		std::sort(hashes.begin(), hashes.end());

		for (auto& it : hashes)
		{
			pddl::hash_combine(m_hash, it);
		}
	}
	return m_hash;
}

std::string Partition::str(bool printMatches) const
		{
	std::string res;
	for (auto& c : m_children)
	{
		res += std::to_string(c->getMatches().size()) + ": " + c->getRef()->str() + ", ";
	}
	if (!res.empty())
	{
		res.pop_back(); //remove white space
		res.pop_back(); //remove comma
	}

	std::string matchesString;
	if (printMatches)
	{
		std::vector<pddl::TypedObjectUnorderedSet> matches;
		getMatches(matches);
		matchesString = "\n\t[";
		for (auto& r : matches)
		{
			if (matchesString != "\n\t[")
			{
				matchesString += ", ";
			}
			matchesString += "[";

			std::string objects;
			for (auto& o : r)
			{
				objects += o->getName() + ", ";
			}
			REMOVE_LAST_COMMA(objects);

			matchesString += objects + "]";
		}
		matchesString += "]";
	}

	if (!m_parent)
	{
		return "[ " + res + " ]" + matchesString;
	}
	else
	{
		std::string resParent;
		for (auto& r : m_parent->getReferences())
		{
			resParent += r->str() + ", ";
		}
		if (!resParent.empty())
		{
			resParent.pop_back(); //remove white space
			resParent.pop_back(); //remove comma
		}
		return resParent + " [ " + res + " ]" + matchesString + "; Split: "
				+ (m_splitType == Type ? "type" : m_splitType == Instance ? "instance" : "other");
	}
}

const std::vector<std::shared_ptr<PartitionEntry> >& Partition::getChildren() const
{
	return m_children;
}

const std::shared_ptr<ReferenceList>& Partition::getContext() const
{
	return m_context;
}

std::size_t Partition::getHash() const
{
	return m_hash;
}

int Partition::otherCount()
{
	if (m_otherCount == -1)
	{
		for (auto& c : m_children)
		{
			if (isInstanceSharedCast(c->getRef(), cref, OtherReference))
			{
				m_otherCount = c->getMatches().size();
			}
		}
	}

	return m_otherCount;
}

bool Partition::hasMoreThanOnePartitionWithMatches()
{
	int counter = 0;
	for (auto& c : m_children)
	{
		if (!c->getMatches().empty())
		{
			++counter;
			if (counter > 1)
			{
				return true;
			}
		}
	}
	return false;
}

bool Partition::isProper()
{
	pddl::TypedObjectUnorderedSet all;
	std::vector<pddl::TypedObjectUnorderedSet> matches;
	getMatches(matches);
	for (auto& m : matches)
	{
		if (pddl::any<std::shared_ptr<pddl::TypedObject>>(m, [&all](const std::shared_ptr<pddl::TypedObject>& o)
		{	return CONTAINS(o, all);}))
		{
			return false;
		}
		all.insert(m.begin(), m.end());
	}

	return true;
}

double Partition::information(bool potential)
{
//	double weight = 1.0;
//
//	//Hack to prefere some attributes
//	for (auto& c : m_children)
//	{
//		auto& ref = c->getRef();
//		if (isInstanceSharedCast(ref, iRef, IdentityReference))
//		{
//			weight = 10.0;
//		}
//		else if (isInstanceSharedCast(ref, rRef, RelationalReference))
//		{
//			std::string functionName = rRef->getFunction()->getName();
//			if (functionName == "in")
//			{
//				weight = 5.0;
//				break;
//			}
//			else if (functionName == "aligned")
//			{
//				weight = 5.0;
//				break;
//			}
//		}
//	}

	std::vector<pddl::TypedObjectUnorderedSet> sets;
	for (auto& c : m_children)
	{
		pddl::TypedObjectUnorderedSet pk = c->getMatches();
		isInstanceSharedCast(c->getRef(), rRef, RelationalReference);
		if (potential && rRef)
		{
			for (auto& p : rRef->getOptimisticPartition())
			{
				sets.push_back(pddl::unordered_helpers::makeUnion(p, pk));
			}
		}
		else
		{
			sets.push_back(pk);
		}
	}

	std::vector<pddl::TypedObjectUnorderedSet> partition;
	//basically copies all sets into partition and adds another set which
	//contains the missing elements (i.e., the "other" partition entry)
	makePartition(sets, partition);
	std::vector<double> counts;
	counts.reserve(partition.size());
	for (auto& s : partition)
	{
		if (s.size() != 0) //other could be empty
			counts.push_back(s.size());
	}

	double total = std::accumulate(counts.begin(), counts.end(), 0);

	std::vector<double> probs;
	probs.reserve(counts.size());
	for (auto& c : counts)
	{
		probs.push_back(total / c);
	}

	double I = 0;
	for (auto& p : probs)
	{
		if (p != 0)
		{
			I += -p * log2(p);
		}
	}

//	LOG_INFO("Information: " + std::to_string(weight * I));

	return /*weight **/I;
}

void Partition::getMatches(std::vector<pddl::TypedObjectUnorderedSet>& matches) const
		{
	for (auto& child : m_children)
	{
		matches.push_back(child->getMatches());
	}
}

const std::shared_ptr<PartitionEntry>& Partition::getParent() const
{
	return m_parent;
}

size_t Partition::size() const
{
	if (m_parent)
	{
		return m_parent->getMatches().size();
	}
	else
	{
		return m_context->getObjects().size();
	}
}

void Partition::setParent(const std::shared_ptr<PartitionEntry>& parent)
{
	m_parent = parent;
}

void Partition::makePartition(std::vector<pddl::TypedObjectUnorderedSet>& sets,
		std::vector<pddl::TypedObjectUnorderedSet>& result)
{
	pddl::TypedObjectUnorderedSet all, used;
	std::sort(sets.begin(), sets.end(),
			[](const pddl::TypedObjectUnorderedSet& a, const pddl::TypedObjectUnorderedSet& b)
			{
				return a.size() < b.size();
			});
	for (auto& s : sets)
	{
//		LOG_INFO_CONTAINER_STR(s)
		all.insert(s.begin(), s.end());
		pddl::TypedObjectUnorderedSet u = pddl::unordered_helpers::makeIntersection(s, used);
		if (u.empty())
		{
			result.push_back(s);
			used.insert(s.begin(), s.end());
		}
	}

//	if (pddl::unordered_helpers::isSubset(used, all))
//	{
//		LOG_INFO_CONTAINER_STR(pddl::unordered_helpers::minus(all, used));
	result.push_back(pddl::unordered_helpers::minus(all, used));
//	}
}

std::shared_ptr<Partition> Partition::expandUnique()
{
	if (m_children.size() > 1)
		return std::shared_ptr<Partition>();

	auto newP = m_children[0]->next(1);

	//remove steps which are not required, because there is only one object
	if (newP && newP->getChildren().size() == 1 && m_children[0]->getRef() == newP->getChildren()[0]->getRef())
	{
		return newP->expandUnique();
	}

	return shared_from_this();
}

std::shared_ptr<Partition> Partition::getInitialPartition(const pddl::TypePtr& type,
		const std::shared_ptr<ReferenceList>& refContext)
{
	std::shared_ptr<Partition> p(new Partition());
	p->init( { std::shared_ptr<TypenameReference>(new TypenameReference(type)) }, std::shared_ptr<PartitionEntry>(), Partition::Type, refContext);
	return p;
}

const std::shared_ptr<PartitionEntry>& Partition::getExpandParent() const
{
	return m_expandParent;
}

void Partition::setExpandParent(const std::shared_ptr<PartitionEntry>& expandParent)
{
	m_expandParent = expandParent;
}

std::shared_ptr<PartitionEntry> Partition::getChildForObject(const pddl::TypedObjectPtr& obj)
{
	for (auto& pe : m_children)
	{
		if (pe->matches(obj))
		{
			return pe;
		}
	}

	throw pddl::Exception("cannot find child");
}

const std::shared_ptr<AlternativePartitionEntry>& Partition::getAlternativeParent() const
{
	return m_alternativeParent;
}

void Partition::setAlternativeParent(const std::shared_ptr<AlternativePartitionEntry>& alternativeParent)
{
	m_alternativeParent = alternativeParent;
}

double Partition::remainingSize(const pddl::TypedObjectPtr& target,
		bool potential)
{
//	double weight = 1.0;
//	for (auto& c : m_children)
//	{
//		if (isInstance(c->getRef(), IdentityReference))
//		{
//			weight = 10.0;
//		}
//		else if (isInstanceSharedCast(c->getRef(), r, RelationalReference))
//		{
//			if (r->getFunction()->getName() == "in")
//			{
//				weight = 5;
//				break;
//			}
//			else if (r->getFunction()->getName() == "aligned")
//			{
//				weight = 5;
//				break;
//			}
//		}
//	}

	std::vector<pddl::TypedObjectUnorderedSet> targetSetList;
	for (auto& c : m_children)
	{
		auto& pc = c->getMatches();
		dynamicSharedPointerCast(c->getRef(), r, RelationalReference);
		if (r and potential)
		{
			for (auto& p : r->getOptimisticPartition())
			{
				auto newSet = pddl::unordered_helpers::makeUnion(p, pc);
				if (CONTAINS(target, newSet))
				{
					targetSetList.push_back(newSet);
				}
			}
		}
		else
		{
			if (CONTAINS(target, pc))
			{
				targetSetList.push_back(pc);
			}
		}
	}

	pddl::TypedObjectUnorderedSet targetSet;
	if (!targetSetList.empty())
	{
		targetSet = targetSetList[0];
	}

	if (targetSet.empty() || targetSet.size() == 1)
	{
		return /*weight **/-1.0;
	}

	return /*weight **/-(1.0 / ((double) targetSet.size() - 1));
}

int Partition::refCount(std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher>& counted)
{
	int count = 0;

	if (m_parent)
		count = m_parent->getPartition()->refCount(counted);

	for (auto& c : m_children)
	{
		count += c->getRef()->refCount();
	}

	return count;
}

std::shared_ptr<Partition> Partition::makeProper()
{
	pddl::TypedObjectUnorderedSet used;
	ReferenceVector group;
	for (auto& pe : m_children)
	{
		if (isInstanceSharedCast(pe->getRef(), oref, OtherReference))
		{
			continue;
		}

		const auto& matches = pe->getMatches();
		bool cont = false;
		for (auto& o: matches)
		{
			if (CONTAINS(o, used))
			{
				cont = true;
				break;
			}
		}

		if (cont)
		{
			continue;
		}

		if (matches.size() < size())
		{
			group.push_back(pe->getRef());
		}

		used.insert(matches.begin(), matches.end());
	}

	group.push_back(ReferencePtr(new OtherReference(group)));

	PartitionPtr p(new Partition());
	p->init(group, m_parent, m_splitType);
	return p;
}

} /* namespace goal_planner_gui */

