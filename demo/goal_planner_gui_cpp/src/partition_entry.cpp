/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 10, 2017
 *      Author: kuhnerd
 * 	  Filename: partition_entry.cpp
 */

#include <goal_planner_gui/alternative_partition_entry.h>
#include <goal_planner_gui/existential_partition_entry.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/state/state_variable.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/references/feature_reference.h>
#include <goal_planner_gui/references/function_reference.h>
#include <goal_planner_gui/references/other_reference.h>
#include <goal_planner_gui/references/relational_reference.h>
#include <goal_planner_gui/universal_partition_entry.h>
#include <queue>
#include <tuple>
#include <cmath>

namespace std
{

template<>
class hash<tuple<size_t, int, int>>
{
public:
	size_t operator()(const tuple<size_t, int, int>& t) const
			{
		size_t h = 0;
		pddl::hash_combine(h, std::get<0>(t), std::get<1>(t), std::get<2>(t));
		return h;
	}
};

}

namespace goal_planner_gui
{

struct SortKey
{
	SortKey(double Ip,
			double I,
			int rc,
			int depth,
			int otherGroup,
			int index) :
					Ip(Ip),
					I(I),
					rc(rc),
					depth(depth),
					otherGroup(otherGroup),
					index(index)
	{
	}

	bool operator>(const SortKey& other) const
			{
		if (Ip > other.Ip)
			return true;
		else if (Ip == other.Ip)
			if (I > other.I)
				return true;
			else if (I == other.I)
				if (rc > other.rc)
					return true;
				else if (rc == other.rc)
					if (depth > other.depth)
						return true;
					else if (depth > other.depth)
						if (otherGroup > other.otherGroup)
							return true;
		return false;
	}

	bool operator<(const SortKey& other) const
			{
		return !operator >(other);
	}

	double Ip;
	double I;
	int rc;
	int depth;
	int otherGroup;
	int index;
};

SortKey getSortKey(const std::shared_ptr<Partition>& p,
		size_t index,
		const PartitionVector& partitions,
		const pddl::TypedObjectPtr& target,
		int depth = 0)
{
	int otherGroup = 0;
	for (auto& c : p->getChildren())
	{
		if (isInstanceSharedCast(c->getRef(), oref, OtherReference))
		{
			otherGroup = 1;
			break;
		}
	}

	double I, Ip;
	if (target)
	{
		I = p->remainingSize(target);
		Ip = p->remainingSize(target, true);
	}
	else
	{
		I = p->information();
		Ip = p->information(true);
	}

	std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher> counted;
	return SortKey(Ip, I, p->refCount(counted), depth, otherGroup, index);
}

void __expand(const PartitionPtr& p,
		const pddl::TypedObjectPtr& target,
		PartitionVector& result)
{
	PartitionEntryVector relevant;
	for (auto& c : p->getChildren())
	{
		if (!c->isEmpty())
		{
			relevant.push_back(c);
		}
	}

	for (auto& pnext : relevant)
	{
		PartitionVector expanded;
		pnext->expandRelationRef(expanded);
		for (auto& p2 : expanded)
		{
			if (target)
			{
				if (isInstanceSharedCast(p2->getChildForObject(target), oref, OtherReference))
				{
					continue;
				}
			}
			result.push_back(p2);
		}
	}
}

PartitionEntry::PartitionEntry(const std::shared_ptr<Reference>& ref,
		const std::shared_ptr<Partition>& partition) :
				m_ref(ref),
				m_partition(partition),
				m_context(partition->getContext()),
				m_matchesComputed(false),
				m_matchesVectorComputed(false),
				m_allRefsComputed(false),
				m_reachableObjectsSet(false),
				m_functionRefsInit(false),
				m_relationRefsInit(false),
				m_featureRefsInit(false)
{
//	LOG_INFO("create " << ref->str() << " " << getMatches().size());
}

PartitionEntry::PartitionEntry(const std::shared_ptr<Reference>& ref,
		const std::shared_ptr<Partition>& partition,
		pddl::TypedObjectVector& matches) :
				m_ref(ref),
				m_partition(partition),
				m_context(partition->getContext()),
				m_matchesComputed(false),
				m_matchesVectorComputed(true),
				m_matchesVector(matches),
				m_allRefsComputed(false),
				m_reachableObjectsSet(false),
				m_functionRefsInit(false),
				m_relationRefsInit(false),
				m_featureRefsInit(false)
{
}

PartitionEntry::~PartitionEntry()
{
}

bool PartitionEntry::operator ==(PartitionEntry& rhs)
{
	return hash() == rhs.hash();
}

bool PartitionEntry::operator !=(PartitionEntry& rhs)
{
	return !(*this == rhs);
}

size_t PartitionEntry::hash()
{
	std::size_t h = 0;
	pddl::hash_combine(h, m_ref->hash());
	for (auto& it : getMatches())
	{
		pddl::hash_combine(h, it->hash(true));
	}
	return h;
}

std::string PartitionEntry::str() const
{
	std::string res;
	for (const auto& r : getReferences())
	{
		res += r->str() + ", ";
	}
	if (!res.empty())
	{
		res.pop_back();
		res.pop_back();
	}
	return res;
}

std::shared_ptr<Partition> PartitionEntry::next(int maxSize)
{
	if (!m_next)
	{
		m_next = searchBest(maxSize, 1, pddl::TypedObjectPtr(), false);
	}

	return m_next;
}

std::shared_ptr<Partition> PartitionEntry::searchBest(int maxSize,
		int maxDepth,
		const pddl::TypedObjectPtr& target,
		bool checkInformation,
		bool onlyAllowGoodPartitions)
{
	PartitionVector sucTemp;
	PartitionVector sucs;
	successors(sucTemp);
	for (auto& p : sucTemp)
	{
		//only has matches in the other partition entry
		if (onlyAllowGoodPartitions && !p->hasMoreThanOnePartitionWithMatches())
		{
			continue;
		}

		if (p->otherCount() == 0 || Constants::ALLOW_OTHER)
		{
			sucs.push_back(p);
		}
	}

//	LOG_INFO("Found " << sucs.size() << " successors");

	PartitionUnorderedSet used;
	PartitionVector partitions;
	for (auto& p : sucs)
	{
		if (target)
		{
			if (isInstanceSharedCast(p->getChildForObject(target), oref, OtherReference))
			{
				continue;
			}
		}

		//any
		bool isRelational = false;
		for (auto& c: p->getChildren())
		{
			if (isInstanceSharedCast(c->getRef(), rref, RelationalReference))
			{
				isRelational = true;
				break;
			}
		}

		if (CONTAINS(p, used) && !isRelational)
		{
			//pass
		}
		else
		{
			partitions.push_back(p);
		}

		used.insert(p);
	}

	std::vector<std::shared_ptr<Partition> > forbiddenPartitions;
	std::vector<std::shared_ptr<Reference> > forbiddenRefs;
	__determineFobiddenPartitionsAndRefs(forbiddenPartitions, forbiddenRefs);

//	LOG_INFO("Forbidden Refs")
//	LOG_INFO_CONTAINER_STR(forbiddenRefs)

	m_lastPartitions = partitions;

	if (partitions.empty())
	{
		return std::shared_ptr<Partition>();
	}

	std::priority_queue<SortKey, std::vector<SortKey>, std::greater<SortKey>> keys;
	for (size_t i = 0; i < partitions.size(); ++i)
	{
		auto& p = partitions[i];
		auto key = getSortKey(p, i, partitions, target);
		keys.push(key);
//		LOG_INFO("option: " << p->str() << " " << key.I << " " << key.Ip);
	}

	std::unordered_map<int, pddl::DoubleDefaultZero> layerI;
	std::shared_ptr<Partition> best;
	double bestI = std::numeric_limits<int>::max();
	int bestRc = std::numeric_limits<int>::max(); //TODO: makes more sense than 0!
	int maxGrace = 100;
	int grace = maxGrace;
	while (!keys.empty())
	{
		auto k = keys.top();
		keys.pop();

		auto& p = partitions[k.index];

//		LOG_INFO("current: " << p->str());
		ReferenceVector pRefs;
		for (auto& child : p->getChildren())
			pRefs.push_back(child->getRef());

		if (p->otherCount() > 0 && !Constants::ALLOW_OTHER)
		{
			continue;
		}

		bool semanticEqual = false;
		for (auto& r : forbiddenRefs)
		{
			if (pRefs[0]->semanticEqual(r))
			{
				semanticEqual = true;
				break;
			}
		}

		if (semanticEqual)
		{
//			LOG_DEBUG("Skip because of equal");
			continue;
		}

		layerI[k.depth].val = std::max(k.I, layerI[k.depth].val);

		if (k.I < bestI || (k.I == bestI && k.rc < bestRc))
		{
			LOG_DEBUG("#### found best: p=" << p->str(true) << ", I=" << k.I << ", Ip=" << k.Ip << ", depth=" << k.depth << ", rc=" << k.rc);
			bestI = k.I;
			bestRc = k.rc;
			best = p;
		}

		if (fabs(k.Ip - k.I) < 0.01)
		{
//			LOG_DEBUG("Break because of Ip - I < 0.01");
			break;
		}

		if (k.I <= layerI[k.depth - 1].val)
		{
			if (grace == 0)
			{
				break;
			}
			else
			{
				--grace;
			}
		}

		if (k.depth >= maxDepth)
		{
			continue;
		}

		PartitionVector expanded;
		__expand(p, target, expanded);
		for (auto& pnext : expanded)
		{
//			LOG_INFO("-> Expanded: " << pnext->str())
			int i2 = partitions.size();
			auto k2 = getSortKey(pnext, i2, partitions, target, k.depth + 1);
			if (k2.Ip <= bestI)
			{
				partitions.push_back(pnext);
				keys.push(k2);
			}
		}
	}

	if (!best)
	{
		return std::shared_ptr<Partition>();
	}

	auto self = shared_from_this();

	if (!best->isProper())
	{
		best = best->makeProper();
	}

	if (best->getParent() != self)
	{
		best->setParent(self);
	}

	if (isInstanceSharedCast(self, ape, AlternativePartitionEntry))
	{
		best->setAlternativeParent(ape);
	}
	else
	{
		best->setAlternativeParent(m_partition->getAlternativeParent());
	}

	double information = best->information(true);

	std::vector<pddl::TypedObjectUnorderedSet> matches;
	best->getMatches(matches);
//	LOG_DEBUG("best: " << best << ", information: " << information << ", length matches: " << matches.size());

	if (checkInformation && (fabs(information) <= 0.0001))
	{
		return std::shared_ptr<Partition>();
	}

	return best;
}

void PartitionEntry::successors(PartitionVector& successors)
{
	auto& objects = getMatches();
//	LOG_INFO_CONTAINER_TEXT_STR("Objects: ", objects)
	double size = objects.size();

//	LOG_INFO("successors for objects:")
//	LOG_INFO_CONTAINER_STR(objects);

	//named groups
	std::vector<ReferenceVector> namedGroups;
	m_context->generateNamedGroups(objects, namedGroups);
	for (auto& g : namedGroups)
	{
		std::shared_ptr<Partition> pnext(new Partition());
//		LOG_INFO_CONTAINER_TEXT_STR("Refs: ", g);
		pnext->init(g, shared_from_this(), Partition::Instance, m_context);
		if (CONTAINSV_NOT(pnext, successors))
		{
			successors.push_back(pnext);
		}
//		LOG_INFO(" - " << pnext->str());
	}

	//typed groups
	std::vector<ReferenceVector> typedGroups;
	m_context->generateTypeGroups(objects, typedGroups);
	for (auto& g : typedGroups)
	{
		std::shared_ptr<Partition> pnext(new Partition());
//		LOG_INFO_CONTAINER_TEXT_STR("Refs: ", g);
		pnext->init(g, shared_from_this(), Partition::Type, m_context);
		if (CONTAINSV_NOT(pnext, successors))
		{
			successors.push_back(pnext);
		}
//		LOG_INFO(" - " << pnext->str());
	}

	auto currentType = getMostSpecificTyperef()->getType();
//	LOG_INFO(currentType->str() << " matches: ");
//	LOG_INFO_CONTAINER_STR(getMatches());
//	LOG_INFO(m_partition->getParent())
	double I = -1.0 / size * log2(1.0 / size);
	if (m_context->getTypeInformation(currentType) > I)
	{
		for (auto& p : m_context->getTypePartitions(currentType))
		{
			ReferenceVector refs;
			for (auto& c : p->getChildren())
			{
				refs.push_back(c->getRef());
			}
//			LOG_INFO_CONTAINER_TEXT_STR("Refs: ", refs);
			std::shared_ptr<Partition> pnext(new Partition());
			pnext->init(refs, shared_from_this(), Partition::Other, m_context);
			if (fabs(pnext->information()) >= fabs(I) && (Constants::ALLOW_OTHER || pnext->otherCount() == 0))
			{
				if (CONTAINSV_NOT(pnext, successors))
				{
					successors.push_back(pnext);
				}
//				LOG_INFO(" - " << pnext->str());
			}
		}
	}

	std::unordered_set<std::tuple<size_t, int, int>> arcsDone;
//	LOG_INFO("Current: " << currentType->str());
	for (auto& it : m_context->getConnections(currentType))
	{
		auto& t2 = it.first;
		auto& arcs = it.second;

		for (auto& it2 : arcs)
		{
			std::tuple<size_t, int, int> arcs1(it2.f->hash(), it2.j, it2.i);
			std::tuple<size_t, int, int> arcs2(it2.f->hash(), it2.i, it2.j);
			if (CONTAINS_NOT(arcs1, arcsDone) && CONTAINS_NOT(arcs2, arcsDone))
			{
				std::vector<std::shared_ptr<pddl::Type>> allTypes;
				for (auto& a : it2.f->getArgs())
					allTypes.push_back(a->getType());
				allTypes.push_back(it2.f->getType());

				ReferenceVector refs;
				m_context->getRelationalGroup(it2.f, it2.i, allTypes, refs);

				std::shared_ptr<Partition> pnext(new Partition());
				pnext->init(refs, shared_from_this(), Partition::Other, m_context);
//				LOG_INFO("Partition: " << pnext->str(true))
				arcsDone.insert(arcs1);
				arcsDone.insert(arcs2);
				if (Constants::ALLOW_OTHER || pnext->otherCount() == 0)
				{
					if (CONTAINSV_NOT(pnext, successors))
					{
						successors.push_back(pnext);
					}
//					LOG_INFO(" - " << pnext->str());
				}
			}
		}
	}

	expandRelationRef(successors);

//	LOG_INFO("Found " << successors.size())
}

std::shared_ptr<TypenameReference> PartitionEntry::getMostSpecificTyperef(bool hiddenRefs)
{
	auto& matches = getMatches();
	if (matches.empty())
	{
		std::shared_ptr<TypenameReference> best(new TypenameReference(pddl::DefaultTypes::objectType()));
		for (auto& ref : getReferences())
		{
			if (isInstanceSharedCast(ref, tref, TypenameReference))
			{
				if (tref->getType()->isSubtypeOf(best->getType()))
				{
					best = tref;
				}
			}
		}
		return best;
	}

	pddl::TypeUnorderedSet allTypes;
	if (hiddenRefs)
	{
		const auto& t = m_context->getDomain()->m_types;
		for (auto& it : t)
		{
			allTypes.insert(it.second);
		}
	}
	else
	{
		allTypes = m_context->getTypenames();
	}

	pddl::TypeUnorderedSet validTypes;
	for (auto& t : allTypes)
	{
		bool success = true;
		for (auto& o : matches)
		{
			if (!o->isInstanceOf(t))
			{
				success = false;
				break;
			}
		}
		if (success)
		{
			validTypes.insert(t);
		}
	}

	pddl::TypePtr best = pddl::DefaultTypes::objectType();
	for (auto& t : validTypes)
	{
		if (t->isSubtypeOf(best) && t != pddl::DefaultTypes::anyType())
		{
			best = t;
		}
	}

	return std::shared_ptr<TypenameReference>(new TypenameReference(best));
}

void PartitionEntry::expandRelationRef(PartitionVector& partitions)
{
	dynamicSharedPointerCast(m_ref, rref, RelationalReference);
	if (!rref)
	{
		return;
	}

	std::vector<std::vector<std::shared_ptr<Reference>>> children;
	rref->getChildren(children);
	for (auto& ref : children)
	{
//		LOG_INFO_CONTAINER_TEXT_STR("Refs: ", ref);
		ref.push_back(std::shared_ptr<OtherReference>(new OtherReference(ref)));
		std::shared_ptr<Partition> p(new Partition());
		p->init(ref, m_partition->getParent(), Partition::Other);
		p->setExpandParent(shared_from_this());
		if (CONTAINSV_NOT(p, partitions))
		{
			partitions.push_back(p);
		}
	}
}

std::shared_ptr<PartitionEntry> PartitionEntry::fromReference(const std::shared_ptr<Reference>& ref,
		const std::shared_ptr<ReferenceList>& refContext)
{
	std::shared_ptr<Partition> part(new Partition());
	part->init( { ref }, std::shared_ptr<PartitionEntry>(), Partition::Init, refContext);
	return part->getChildren()[0];
}

const std::shared_ptr<ReferenceList>& PartitionEntry::getContext() const
{
	return m_context;
}

const std::shared_ptr<Partition>& PartitionEntry::getPartition() const
{
	return m_partition;
}

const pddl::TypedObjectUnorderedSet& PartitionEntry::getMatches()
{
	if (!m_matchesComputed)
	{
		getMatchesAsVector();
		m_matches.insert(m_matchesVector.begin(), m_matchesVector.end());
		m_matchesComputed = true;
	}

	return m_matches;
}

const pddl::TypedObjectVector& PartitionEntry::getMatchesAsVector()
{
	if (!m_matchesVectorComputed)
	{
		pddl::TypedObjectUnorderedSet objects;
		if (!m_partition->getParent())
		{
			objects = m_context->getObjects();
		}
		else
		{
			objects = m_partition->getParent()->getMatches();
		}

		for (auto& o : objects)
		{
			if (m_ref->matches(o))
			{
				m_matchesVector.push_back(o);
			}
		}

		m_matchesVectorComputed = true;
	}

	return m_matchesVector;
}

const std::shared_ptr<IdentityReference> PartitionEntry::getIdentity()
{
	for (auto& ref : getReferences())
	{
		if (isInstanceSharedCast(ref, idRef, IdentityReference))
		{
			return idRef;
		}
	}
	return std::shared_ptr<IdentityReference>();
}

const ReferenceUnorderedSet& PartitionEntry::getReferences() const
{
	if (!m_allRefsComputed)
	{
		if (!m_partition->getParent())
		{
			m_allRefs.insert(m_ref);
		}
		else
		{
			for (auto& r : m_partition->getParent()->getReferences())
			{
				isInstanceSharedCast(m_ref, featRef, FeatureReference);
				isInstanceSharedCast(r, featRef2, FeatureReference);
				isInstanceSharedCast(m_ref, funcRef, FunctionReference);
				isInstanceSharedCast(r, funcRef2, FunctionReference);
				if ((featRef && featRef2 && featRef->getFunction()->getName() == featRef2->getFunction()->getName())
						|| (funcRef && funcRef2 && funcRef->getFunction()->getName() == funcRef2->getFunction()->getName()))
				{
					continue;
				}

				m_allRefs.insert(r);
			}

			m_allRefs.insert(m_ref);
		}
		m_allRefsComputed = true;
	}
	return m_allRefs;
}

const std::shared_ptr<Reference>& PartitionEntry::getRef() const
{
	return m_ref;
}

bool PartitionEntry::matches(const pddl::TypedObjectPtr& obj)
{
	if (m_ref->matches(obj))
	{
		if (!m_partition->getParent())
		{
			return true;
		}
		return m_partition->getParent()->matches(obj);
	}
	return false;
}

void PartitionEntry::getAlternativeParents(std::unordered_set<std::shared_ptr<AlternativePartitionEntry>, PartitionEntryHasher>& alternatives)
{
	const auto& altParent = m_partition->getAlternativeParent();
	if (altParent)
	{
		altParent->getAlternativeParents(alternatives);
		alternatives.insert(altParent);
	}
}

void PartitionEntry::__determineFobiddenPartitionsAndRefs(std::vector<std::shared_ptr<Partition> >& forbiddenPartitions,
		std::vector<std::shared_ptr<Reference> >& forbiddenRefs)
{
	std::unordered_set<std::shared_ptr<AlternativePartitionEntry>, PartitionEntryHasher> alternativeParents;
	getAlternativeParents(alternativeParents);
	for (auto& p : alternativeParents)
	{
		auto& forbidPart = p->getForbidPartition();
		if (forbidPart)
		{
			forbiddenPartitions.push_back(forbidPart);
			forbiddenRefs.push_back(forbidPart->getExpandParent()->getRef());
		}
	}
}

bool PartitionEntry::isEmpty()
{
	return getMatches().empty();
}

bool PartitionEntry::isQuantified()
{
	return false;
}

bool PartitionEntry::isUnique()
{
	return getMatches().size() == 1;
}

int PartitionEntry::refCount(std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher>& counted)
{
	if (getIdentity())
	{
		return 1;
	}

	int count = 0;
	if (CONTAINS_NOT(m_ref, counted))
	{
		count = m_ref->refCount();
	}

	counted.insert(m_ref);

	if (m_partition->getParent())
	{
		count += m_partition->getParent()->refCount(counted);
	}

	return count;
}

void PartitionEntry::expand(std::vector<std::shared_ptr<PartitionEntry> >& expanded,
		bool allowUniversial)
{
	if (isLeaf())
	{
		expanded.push_back(shared_from_this());
		return;
	}

	auto newPartition = next(Constants::MAX_SIZE_NEXT);
	LOG_INFO(newPartition);

	auto& children = newPartition->getChildren();
	expanded.insert(expanded.end(), children.begin(), children.end());
	expanded.push_back(makeExistential());
	expanded.push_back(makeAlternative(newPartition));

	if (allowUniversial)
	{
		expanded.push_back(makeUniversial());
	}
}

std::shared_ptr<PartitionEntry> PartitionEntry::expandSingle(const pddl::TypedObjectPtr& target,
		bool checkInformation)
{
	PartitionPtr next = searchBest(9999, 1, target, checkInformation, true);
	if (!next)
		return std::shared_ptr<PartitionEntry>();

	return next->getChildForObject(target);
}

void PartitionEntry::setReachable(const pddl::TypedObjectUnorderedSet& reachableObjects)
{
	if (!m_reachableObjectsSet)
	{
		m_reachableObjects = reachableObjects;
		m_reachableObjectsSet = true;
	}
	else
	{
		LOG_DEBUG("[" << str() << "] Don't overwrite reachable");
	}
	m_matches.clear();
	m_matchesComputed = false;
}

std::shared_ptr<PartitionEntry> PartitionEntry::clone()
{
	std::shared_ptr<PartitionEntry> entry(new PartitionEntry(m_ref, m_partition));
	entry->setReachable(m_reachableObjects);
	entry->m_allRefs = m_allRefs;
	return entry;
}

bool PartitionEntry::isLeaf()
{
	if (isQuantified() || isUnique() || isEmpty())
		return true;

	auto newPartition = next(Constants::MAX_SIZE_NEXT);

	if (newPartition && newPartition->getChildren().size() > 1)
	{
		return false;
	}

	return true;
}

std::shared_ptr<PartitionEntry> PartitionEntry::makeExistential()
{
	return std::shared_ptr<PartitionEntry>(new ExistentialPartitionEntry(m_ref, m_partition));
}

std::shared_ptr<PartitionEntry> PartitionEntry::makeAlternative(const std::shared_ptr<Partition>& newPartition)
{
	//TODO: potential problem? newPartition is changed
	if (!newPartition->getExpandParent())
	{
		newPartition->setExpandParent(newPartition->getChildren()[0]);
	}

	LOG_INFO("Has " << getReferences().size() << " alternatives:")
	LOG_INFO_CONTAINER_TEXT_STR("alternatives: ", getReferences())

	//TODO: getReferences().begin() != python
	return std::shared_ptr<PartitionEntry>(
			new AlternativePartitionEntry(*(getReferences().begin()), newPartition, m_partition->getAlternativeParent(), newPartition));
}

std::shared_ptr<PartitionEntry> PartitionEntry::makeUniversial()
{
	return std::shared_ptr<PartitionEntry>(new UniversalPartitionEntry(m_ref, m_partition));
}

const ReferenceUnorderedSet& PartitionEntry::getAllRefs() const
{
	return m_allRefs;
}

void PartitionEntry::setAllRefs(const ReferenceUnorderedSet& allRefs)
{
	m_allRefs = allRefs;
}

std::string PartitionEntry::getOtherOrArbitrary()
{
	if (isInstance(shared_from_this(), ExistentialPartitionEntry))
	{
		return "arbitrary";
	}

	ReferenceUnorderedSet diff;
	if (m_partition->getParent())
	{
		getReferenceDiff(m_partition->getParent(), diff);
	}

	for (auto& r : diff)
	{
		if (isInstance(r, OtherReference))
		{
			return "other";
		}
	}

	return "";
}

std::string PartitionEntry::text()
{
	auto& ident = getIdentity();
	if (ident)
	{
		return m_context->getName(ident->getObj());
	}

	std::vector<std::string> functionQualifiers;
	for (auto& r : getFunctionRefs())
	{
		std::string argstring;
		for (auto& pe : r->getArgs())
		{
			argstring += pe->text() + ", ";
		}
		if (!argstring.empty())
		{
			argstring.pop_back();
			argstring.pop_back();
		}

		if (r->getValues().size() == 1)
		{
			return m_context->getName(r->getFunction()) + " = " + argstring;
		}

		functionQualifiers.push_back(m_context->getName(r->getFunction()) + " = " + argstring);
	}

	std::vector<std::string> featureQualifiers;
	for (auto& r : getFeatureRefs())
	{
		featureQualifiers.push_back(m_context->getName(r->getFunction()) + " = " + r->getValue()->text());
	}

	for (auto& r : getRelationRefs())
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}

	std::string typeQualifier = getTypeQualifier();

	if (isInstanceSharedCast(shared_from_this(), ap, AlternativePartitionEntry))
	{
		std::vector<std::string> functionQualifiers;
		auto& expandParent = ap->getForbidPartition()->getExpandParent();
		for (auto& r: expandParent->getFunctionRefs())
		{
			std::string argString;
			for (auto& pe: r->getArgs())
			{
				argString += pe->text() + ", ";
			}
			REMOVE_LAST_COMMA(argString);

			if (r->getValues().size() == 1)
			{
				return m_context->getName(r->getFunction()) + " = " + argString;
			}

			functionQualifiers.push_back(m_context->getName(r->getFunction()) + " = " + argString);
		}

		if (!functionQualifiers.empty())
		{
			std::string arg1;
			for (size_t i =0; i < functionQualifiers.size() - 1; ++i)
			{
				arg1 += functionQualifiers[i] + ", ";
			}
			REMOVE_LAST_COMMA(arg1);
			return typeQualifier + " where " + arg1 + " and " + functionQualifiers.back() + " don't care";
		}

		std::vector<std::string> featureQualifiers;
		for(auto& r: expandParent->getFeatureRefs())
		{
			functionQualifiers.push_back(m_context->getName(r->getFunction()) + " = " + r->getValue()->text());
		}

		if (!featureQualifiers.empty())
		{
			std::string arg1;
			for (size_t i =0; i < featureQualifiers.size() - 1; ++i)
			{
				arg1 += featureQualifiers[i] + ", ";
			}
			REMOVE_LAST_COMMA(arg1);
			LOG_ERROR("add split for last argument");
			return typeQualifier + " with " + arg1 + " and " + featureQualifiers.back() + " don't care";
		}
	}

	if (!functionQualifiers.empty() && !featureQualifiers.empty())
	{
		return typeQualifier + " that is " + pddl::stringConcat(functionQualifiers, " and ") + " with " + pddl::stringConcat(featureQualifiers, " and ");
	}

	if (!featureQualifiers.empty())
	{
		return typeQualifier + " with " + pddl::stringConcat(featureQualifiers, " and ");
	}

	if (!functionQualifiers.empty())
	{
		return typeQualifier + " that is " + pddl::stringConcat(functionQualifiers, " and ");
	}

	return typeQualifier;
}

void PartitionEntry::description(std::vector<DescriptionEntry>& descriptions)
{
	auto typeref = getMostSpecificTyperef();
	auto prefix = getOtherOrArbitrary();

	if (!prefix.empty())
	{
		descriptions.push_back( { prefix, "" });
	}

	auto& ident = getIdentity();

	if (ident)
	{
		descriptions.push_back( { m_context->getName(ident->getObj()), m_context->getImage(ident->getObj()) });
		return;
	}
	else
	{
		descriptions.push_back( { m_context->getName(typeref->getType()), m_context->getImage(typeref->getType()) });
	}

	std::string imagePath;
	for (auto& r : getFeatureRefs())
	{
		if (isInstanceSharedCast(r->getValue()->getRef(), iref, IdentityReference))
		{
			auto& obj = iref->getObj();
			imagePath = m_context->getImage(r->getFunction());
			if (!imagePath.empty())
			{
				descriptions.push_back(
						{	m_context->getName(r->getFunction()), imagePath});
				descriptions.push_back(
						{	"=", ""});
			}

			descriptions.push_back(
					{	m_context->getName(obj), m_context->getImage(obj)});
		}
		else
		{
			std::string t = m_context->getFeatureTranslation(r->getFunction()) + r->getValue()->text();
			descriptions.push_back(
					{	t, ""});
		}
	}

	std::string t;
	for (auto& r : getFunctionRefs())
	{
//		LOG_INFO("called " << r->getFunction()->str())
		std::string argString;
		for (auto& pe : r->getArgs())
		{
			argString += pe->text() + ", ";
		}
		REMOVE_LAST_COMMA(argString);
		t = m_context->getFunctionTranslation(r->getFunction()) + argString;
		descriptions.push_back( { t, "" });
	}

	for (auto& r : getRelationRefs())
	{
		std::string argString;
		for (auto& pe : r->getArgs())
		{
			if (pe)
			{
				argString += pe->text() + ", ";
			}
			else
			{
				argString += "x, ";
			}
		}
		REMOVE_LAST_COMMA(argString);
		ASSERT(r->getValue());
		t = m_context->getName(r->getFunction()) + "(" + argString + ") = " + r->getValue()->text();
		descriptions.push_back( { t, "" });
	}
}

const FunctionReferenceVector& PartitionEntry::getFunctionRefs()
{
	if (!m_functionRefsInit)
	{
		for (auto& ref : getReferences())
		{
			if (isInstanceSharedCast(ref, fref, FunctionReference))
			{
				m_functionRefs.push_back(fref);
			}
		}
		m_functionRefsInit = true;
	}
	return m_functionRefs;
}

const RelationalReferenceVector& PartitionEntry::getRelationRefs()
{
	if (!m_relationRefsInit)
	{
		for (auto& ref : getReferences())
		{
			if (ref->getReferenceType() == Reference::TypeRelationalReference)
			{
				staticSharedPointerCast(ref, fref, RelationalReference);
				m_relationRefs.push_back(fref);
			}
		}
		m_relationRefsInit = true;
	}
	return m_relationRefs;
}

const FeatureReferenceVector& PartitionEntry::getFeatureRefs()
{
	if (!m_featureRefsInit)
	{
		for (auto& ref : getReferences())
		{
			if (isInstanceSharedCast(ref, fref, FeatureReference))
			{
				m_featureRefs.push_back(fref);
			}
		}
		m_featureRefsInit = true;
	}
	return m_featureRefs;
}

std::string PartitionEntry::getTypeQualifier()
{
	std::shared_ptr<TypenameReference> typeref = getMostSpecificTyperef();
	const ReferenceUnorderedSet& references = getReferences();

	std::string other = "";
	for (auto& r : references)
	{
		if (isInstanceSharedCast(r, oR, OtherReference))
		{
			other = "other ";
			break;
		}
	}

	std::string special = getSpecialTypeQualifier();
	auto& matches = getMatches();

	if (matches.size() == 1)
	{
		return "the " + other + m_context->getName(typeref->getType());
	}
	else if (!special.empty())
	{
		return special + " " + other + m_context->getName(typeref->getType());
	}
	else if (matches.empty())
	{
		return "a nonexisiting " + other + m_context->getName(typeref->getType());
	}
	else
	{
		std::string tname = m_context->getName(typeref->getType());
		std::string article = "a";
		if (!tname.empty())
		{
			article = (tname[0] == 'a' || tname[0] == 'o' || tname[0] == 'u' || tname[0] == 'e' || tname[0] == 'i') ? "an" : "a";
		}
		return article + " " + other + tname;
	}
}

std::string PartitionEntry::getSpecialTypeQualifier()
{
	return "";
}

void PartitionEntry::getReferenceDiff(const std::shared_ptr<PartitionEntry>& other,
		ReferenceUnorderedSet& diff)
{
	auto& thisRefs = getReferences();
	auto& otherRefs = other->getReferences();
	diff = pddl::unordered_helpers::minus(thisRefs, otherRefs);
}

void PartitionEntry::improve()
{
	//we have args, but no value
	if (isInstanceSharedCast(m_ref, fref, FunctionReference))
	{
		ASSERT(fref->getArgs().size() == 1);
		PartitionEntryPtr arg = fref->getArgs()[0];
		ReferencePtr argRef = arg->getRef();
		ASSERT(isInstance(argRef, TypenameReference));
//		LOG_INFO(fref->getFunction()->str());
//		LOG_INFO(arg);
//		for (auto& it: fref->getArgs())
//		{
//			LOG_INFO("- " << it->str() << " " << it->getMatches().size());
//		}
//		LOG_INFO_CONTAINER_STR(getMatches());
		pddl::TypedObjectVector possibleObjects;
		for (auto& it: getMatches())
		{
			for (auto& sv: *m_context->getInit())
			{
				if (sv.first->getFunction() == fref->getFunction()
						&& it == sv.second)
				{
//					LOG_INFO(sv.first->str() << " -> " << sv.second->str());
					possibleObjects.push_back(sv.first->getArgs()[0]);
				}
			}
		}
//		LOG_INFO_CONTAINER_TEXT_STR("possible objects", possibleObjects);
		PartitionEntryVector newArgs;
//		LOG_INFO(argRef->str())
		Partition p;
		newArgs.push_back(PartitionEntryPtr(new PartitionEntry(argRef, m_partition, possibleObjects)));
		fref->setArgs(newArgs);
	}
	//we have value, but no args
	else if (isInstanceSharedCast(m_ref, fref, FeatureReference))
	{
//		LOG_INFO(fref->getValue());
	}
//	LOG_INFO(" - " << str() << " (" << getMatches().size() << ")");
}

}

/* namespace goal_planner_gui */

