/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: partitions.h
 */

#ifndef HEAD6282A_AF02_4AD1_A532_FB186EB411E5
#define HEAD6282A_AF02_4AD1_A532_FB186EB411E5
#include <goal_planner_gui/references/reference.h>
#include <memory>
#include <vector>

namespace goal_planner_gui
{

class ReferenceList;
class PartitionEntry;
class AlternativePartitionEntry;

class Partition: public std::enable_shared_from_this<Partition>
{
public:
	enum SplitType
	{
		Type,
		Instance,
		Other,
		Init
	};

public:
	Partition();
	virtual ~Partition();

	void init(const std::vector<std::shared_ptr<Reference>>& refs,
			const std::shared_ptr<PartitionEntry>& parent,
			const SplitType splitType,
			const std::shared_ptr<ReferenceList>& refContext = std::shared_ptr<ReferenceList>());

	virtual int otherCount();
	virtual bool hasMoreThanOnePartitionWithMatches();
	virtual bool isProper();
	virtual double information(bool potential = false);
	virtual void getMatches(std::vector<pddl::TypedObjectUnorderedSet>& matches) const;
	virtual std::shared_ptr<PartitionEntry> getChildForObject(const pddl::TypedObjectPtr& obj);
	virtual std::shared_ptr<Partition> makeProper();
	virtual size_t size() const;

	virtual bool operator==(const Partition& rhs) const;
	virtual bool operator!=(const Partition& rhs) const;

	virtual size_t hash() const;
	virtual std::string str(bool printMatches = false) const;

	const std::vector<std::shared_ptr<PartitionEntry> >& getChildren() const;
	const std::shared_ptr<ReferenceList>& getContext() const;
	std::size_t getHash() const;

	std::shared_ptr<Partition> expandUnique();

	double remainingSize(const pddl::TypedObjectPtr& target,
			bool potential = false);
	int refCount(std::unordered_set<std::shared_ptr<Reference>, ReferenceHasher>& counted);

	/**
	 * Will sort the input set!
	 */
	static void makePartition(std::vector<pddl::TypedObjectUnorderedSet>& sets,
			std::vector<pddl::TypedObjectUnorderedSet>& result);

	static std::shared_ptr<Partition> getInitialPartition(const pddl::TypePtr& type,
			const std::shared_ptr<ReferenceList>& refContext);

	const std::shared_ptr<PartitionEntry>& getExpandParent() const;
	void setExpandParent(const std::shared_ptr<PartitionEntry>& expandParent);
	const std::shared_ptr<AlternativePartitionEntry>& getAlternativeParent() const;
	void setAlternativeParent(const std::shared_ptr<AlternativePartitionEntry>& alternativeParent);
	const std::shared_ptr<PartitionEntry>& getParent() const;
	void setParent(const std::shared_ptr<PartitionEntry>& parent);

private:
	std::shared_ptr<PartitionEntry> m_parent;
	std::shared_ptr<PartitionEntry> m_expandParent;
	std::shared_ptr<AlternativePartitionEntry> m_alternativeParent;
	std::shared_ptr<ReferenceList> m_context;
	std::vector<std::shared_ptr<PartitionEntry>> m_children;
	int m_otherCount;
	mutable std::size_t m_hash;
	SplitType m_splitType;
};

HASH_AND_COMPARISON_OPERATOR(Partition);
STREAM_OPERATOR(Partition)

} /* namespace goal_planner_gui */

#endif /* HEAD6282A_AF02_4AD1_A532_FB186EB411E5 */
