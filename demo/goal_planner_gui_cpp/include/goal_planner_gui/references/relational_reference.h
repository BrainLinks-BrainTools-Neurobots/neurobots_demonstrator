/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: relational_reference.h
 */

#ifndef H761DE233_43B6_4AC7_89AE_B5364E1DEA7A
#define H761DE233_43B6_4AC7_89AE_B5364E1DEA7A

#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/references/reference.h>

namespace goal_planner_gui
{

class PartitionEntry;
class ReferenceList;

class RelationalReference: public goal_planner_gui::Reference
{
public:
	RelationalReference(const std::shared_ptr<pddl::Function>& function,
			const std::vector<std::shared_ptr<PartitionEntry>>& args,
			const std::shared_ptr<PartitionEntry>& value,
			const std::shared_ptr<ReferenceList>& refContext);
	virtual ~RelationalReference();
	virtual size_t hash() const;
	virtual bool matches(const std::shared_ptr<pddl::TypedObject>& o) const;
	virtual bool semanticEqual(const std::shared_ptr<Reference>& other) const;
	virtual const pddl::TypedObjectUnorderedSet& getValues() const;
	virtual const std::vector<pddl::TypedObjectUnorderedSet>& getOptimisticPartition();
	void getChildren(std::vector<std::vector<std::shared_ptr<Reference>>>& children);
	virtual int refCount();

	virtual std::shared_ptr<RelationalReference> clone(const std::vector<std::shared_ptr<PartitionEntry>>& args,
	const std::shared_ptr<PartitionEntry>& value);

	virtual std::string str(const std::shared_ptr<pddl::TypedObject>& obj) const;
	const std::vector<std::shared_ptr<PartitionEntry> >& getArgs() const;
	const std::shared_ptr<ReferenceList>& getContext() const;
	const std::shared_ptr<pddl::Function>& getFunction() const;
	const std::shared_ptr<PartitionEntry>& getValue() const;

	void setArgs(const std::vector<std::shared_ptr<PartitionEntry> >& args);
	void setValue(const std::shared_ptr<PartitionEntry>& value);

	virtual inline Reference::Type getReferenceType() const
	{
		return TypeRelationalReference;
	}

private:
	void getChildrenForArg(int i, std::vector<std::shared_ptr<Reference>>& children);

protected:
	std::shared_ptr<pddl::Function> m_function;
	std::vector<std::shared_ptr<PartitionEntry>> m_args;
	std::shared_ptr<PartitionEntry> m_value;
	std::shared_ptr<ReferenceList> m_context;
	mutable pddl::TypedObjectUnorderedSet m_cachedValues;
	mutable bool m_cachedValuesComputed;
	std::vector<pddl::TypedObjectUnorderedSet> m_cachedPartitions;
	mutable bool m_cachedPartitionsComputed;
};

VECTOR_DEF(RelationalReference);

} /* namespace goal_planner_gui2 */

#endif /* H761DE233_43B6_4AC7_89AE_B5364E1DEA7A */
