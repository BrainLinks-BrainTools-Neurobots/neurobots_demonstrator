/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 10, 2017
 *      Author: kuhnerd
 * 	  Filename: partition_entry.h
 */

#ifndef H3155CF55_5819_4B54_8D12_D4807FDDC8E3
#define H3155CF55_5819_4B54_8D12_D4807FDDC8E3
#include <goal_planner_gui/constants.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/references/feature_reference.h>
#include <goal_planner_gui/references/function_reference.h>
#include <goal_planner_gui/references/identity_reference.h>
#include <goal_planner_gui/references/reference.h>
#include <goal_planner_gui/references/relational_reference.h>
#include <goal_planner_gui/references/typename_reference.h>
#include <memory>

namespace goal_planner_gui
{

class Partition;
class ReferenceList;
class Partition;
class AlternativePartitionEntry;

class PartitionEntryHasher;

class PartitionEntry: public std::enable_shared_from_this<PartitionEntry>
{
public:
	struct DescriptionEntry
	{
		std::string text;
		std::string image;
	};

	PartitionEntry(const std::shared_ptr<Reference>& ref,
			const std::shared_ptr<Partition>& partition);
	PartitionEntry(const std::shared_ptr<Reference>& ref,
			const std::shared_ptr<Partition>& partition,
			pddl::TypedObjectVector& matches);

	virtual ~PartitionEntry();

	std::shared_ptr<Partition> next(int maxSize = Constants::MAX_SIZE_NEXT);
	/**
	 * Search for the reference that best splits this partition.
	 Performs a depth-limited best-first search over the space of
	 compound references.

	 If "target" is specified, search for a split so that the set
	 that contains it is as small as possible.
	 */
	std::shared_ptr<Partition> searchBest(int maxSize,
			int maxDepth = 1,
			const pddl::TypedObjectPtr& target = pddl::TypedObjectPtr(),
			bool checkInformation = false,
			bool onlyAllowGoodPartitions = false);
	void successors(PartitionVector& successors);
	std::shared_ptr<TypenameReference> getMostSpecificTyperef(bool hiddenRefs = false);
	void expandRelationRef(PartitionVector& partitions);
	bool matches(const pddl::TypedObjectPtr& obj);
	virtual void getAlternativeParents(std::unordered_set<std::shared_ptr<AlternativePartitionEntry>, PartitionEntryHasher>& alternatives);
	void expand(std::vector<std::shared_ptr<PartitionEntry>>& expanded,
			bool allowUniversial = false);
	std::shared_ptr<PartitionEntry> expandSingle(const pddl::TypedObjectPtr& target,
				bool checkInformation);
	void setReachable(const pddl::TypedObjectUnorderedSet& reachableObjects);

	virtual std::shared_ptr<PartitionEntry> makeExistential();
	virtual std::shared_ptr<PartitionEntry> makeAlternative(const std::shared_ptr<Partition>& newPartition);
	virtual std::shared_ptr<PartitionEntry> makeUniversial();

	virtual bool isEmpty();
	virtual bool isQuantified();
	virtual bool isUnique();
	virtual bool isLeaf();

	int refCount(std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher>& counted);

	const pddl::TypedObjectUnorderedSet& getMatches();
	const pddl::TypedObjectVector& getMatchesAsVector();
	const std::shared_ptr<IdentityReference> getIdentity();
	const ReferenceUnorderedSet& getReferences() const;
	const std::shared_ptr<ReferenceList>& getContext() const;
	const std::shared_ptr<Partition>& getPartition() const;
	const std::shared_ptr<Reference>& getRef() const;
	virtual std::shared_ptr<PartitionEntry> clone();

	virtual bool operator==(PartitionEntry& rhs);
	virtual bool operator!=(PartitionEntry& rhs);
	virtual size_t hash();
	virtual std::string str() const;

	static std::shared_ptr<PartitionEntry> fromReference(const std::shared_ptr<Reference>& ref,
			const std::shared_ptr<ReferenceList>& refContext);
	const ReferenceUnorderedSet& getAllRefs() const;
	void setAllRefs(const ReferenceUnorderedSet& allRefs);

	std::string text();
	void description(std::vector<DescriptionEntry>& descriptions);
	std::string getTypeQualifier();

	const FunctionReferenceVector& getFunctionRefs();
	const RelationalReferenceVector& getRelationRefs();
	const FeatureReferenceVector& getFeatureRefs();

	void improve();

protected:
	virtual void __determineFobiddenPartitionsAndRefs(std::vector<std::shared_ptr<Partition>>& forbiddenPartitions,
			std::vector<std::shared_ptr<Reference>>& forbiddenRefs);
	virtual std::string getSpecialTypeQualifier();
	std::string getOtherOrArbitrary();
	void getReferenceDiff(const std::shared_ptr<PartitionEntry>& other,
			ReferenceUnorderedSet& diff);

protected:
	std::shared_ptr<Reference> m_ref;
	std::shared_ptr<Partition> m_partition;
	std::shared_ptr<ReferenceList> m_context;
	mutable bool m_allRefsComputed;
	std::shared_ptr<Partition> m_next;
	mutable ReferenceUnorderedSet m_allRefs;

	bool m_functionRefsInit;
	FunctionReferenceVector m_functionRefs;
	bool m_relationRefsInit;
	RelationalReferenceVector m_relationRefs;
	bool m_featureRefsInit;
	FeatureReferenceVector m_featureRefs;

	pddl::TypedObjectUnorderedSet m_matches;
	pddl::TypedObjectVector m_matchesVector;
	bool m_matchesComputed;
	bool m_matchesVectorComputed;
	pddl::TypedObjectUnorderedSet m_reachableObjects;
	bool m_reachableObjectsSet;
	std::vector<std::shared_ptr<Partition>> m_lastPartitions;
};

HASH_AND_COMPARISON_OPERATOR(PartitionEntry);
STREAM_OPERATOR(PartitionEntry);

} /* namespace goal_planner_gui */

#endif /* H3155CF55_5819_4B54_8D12_D4807FDDC8E3 */
