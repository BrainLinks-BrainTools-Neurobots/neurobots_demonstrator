/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: action_goal.h
 */

#ifndef H426C2056_AB3B_47C8_AB3D_C26A1D91F9CC
#define H426C2056_AB3B_47C8_AB3D_C26A1D91F9CC

#include <goal_planner_gui/goals/goal_spec.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <memory>
#include <vector>

namespace goal_planner_gui
{

class GoalContext;

class ActionGoal: public GoalSpec
{
public:
	ActionGoal(const std::shared_ptr<GoalContext>& context,
			const pddl::ActionPtr& action,
			const pddl::SimpleEffectVector& effects,
			const PartitionEntryVector& args,
			const pddl::TypedObjectVector& usedArgs,
			bool initial = false);
	virtual ~ActionGoal();

	virtual std::string str() const;

	static void initialGoals(const std::shared_ptr<GoalContext>& context,
			std::vector<std::shared_ptr<GoalSpec>>& goals);
	const std::vector<pddl::TypedObjectVector>& getMatches();

	template<class T>
	static std::shared_ptr<T> goalsFromAction(const std::shared_ptr<GoalContext>& context,
			const pddl::ActionPtr& action)
	{
		pddl::SimpleEffectVector effects;
		action->m_effect->collectEffects(effects);

		std::unordered_set<std::shared_ptr<pddl::Parameter>, pddl::TypedObjectHasher> args;
		for (auto& e : effects)
		{
			std::unordered_set<std::shared_ptr<pddl::Parameter>, pddl::TypedObjectHasher> res;
			e->collectFreeVars(res);
			args.insert(res.begin(), res.end());
		}

//		LOG_INFO("Found " << args.size() << " free args");

		PartitionEntryVector argPartitions;
		pddl::TypedObjectVector usedArgs;

		for (auto& a : action->m_args)
		{
			if (CONTAINS(a, args))
			{
				auto p = Partition::getInitialPartition(a->getType(), context->m_refs)->expandUnique();
				LOG_INFO("Initial for arg: " << a->str() << ": " << p->str());
				argPartitions.push_back(p->getChildren()[0]);
				usedArgs.push_back(a);
			}
		}

		std::shared_ptr<T> goal(new T(context, action, effects, argPartitions, usedArgs, true));

		return goal;
	}

	virtual void getNext(std::vector<std::shared_ptr<GoalSpec>>& res);
	virtual int getNumReachable();
	virtual int getNumReached();
	virtual const std::vector<pddl::FactVector>& getReachableGoals();
	virtual const PartitionEntryVector& allPartitions();
	const pddl::ActionPtr& getAction() const;
	virtual const PartitionEntryVector& getArgs();

	virtual PartitionEntryPtr getCurrentArg();
	virtual int argIndex(const PartitionEntryPtr& arg);
	virtual PartitionEntryPtr getArg(int index);

	virtual pddl::ConditionPtr pddlGoal();

protected:
	class PairTypedObjectIntHasher
	{
	public:

		std::size_t operator()(const std::pair<pddl::TypedObjectPtr, int>& obj) const noexcept
		{
			size_t hash = 0;
			pddl::hash_combine(hash, obj.first->hash(), obj.second);
			return hash;
		}
	};

	struct FactVectorHasher
	{
		std::size_t operator()(const pddl::FactVector& obj) const noexcept
		{
			size_t hash = 0;
			for (auto& it : obj)
			{
				pddl::hash_combine(hash, it->hash());
			}
			return hash;
		}
	};

	virtual void getCompletedArgs(PartitionEntryVector& args);
	virtual void getFutureArgs(PartitionEntryVector& args);
	virtual bool allowUniversalExpansion(const PartitionEntryPtr& entry);
	virtual void getReachableObjects(pddl::TypedObjectUnorderedSet& result,
			const PartitionEntryPtr& arg);
	virtual std::shared_ptr<GoalSpec> getChild(const PartitionEntryVector& args);
	virtual std::shared_ptr<GoalSpec> getAlternativeChild(const PartitionEntryVector& args,
			const std::shared_ptr<Partition>& forbidPartition);
	virtual void getPartitionsForEffect(const std::shared_ptr<pddl::SimpleEffect>& effect,
			PartitionEntryVector& partitions);
	/**
	 * Returns an array of indices. A return value of -1 means back()
	 */
	virtual void getArgumentIndices(const std::shared_ptr<pddl::Literal>& literal,
			std::vector<int>& indices);
	virtual void getEffectsFromArg(const pddl::TypedObjectVector& arg,
			pddl::FactVector& effects);
	virtual void filterPreconditions(const std::vector<pddl::TypedObjectVector>& input,
			std::vector<pddl::TypedObjectUnorderedSet>& out);
	virtual int checkQuantifiedGoalCount(const std::vector<pddl::FactVector>& goals,
			const PartitionEntryVector& args,
			std::unordered_map<std::pair<pddl::TypedObjectPtr, int>, std::vector<pddl::FactVector>, PairTypedObjectIntHasher>& goalByObjectPos);
	int checkQuantifiedGoalCountRec(const PartitionEntryVector& args,
			std::unordered_map<std::pair<pddl::TypedObjectPtr, int>, std::vector<pddl::FactVector>, PairTypedObjectIntHasher>& goalByObjectPos,
			int i,
			const std::unordered_set<pddl::FactVector, FactVectorHasher>& facts,
			const std::vector<pddl::TypedObjectUnorderedSet>& objects);

protected:
	pddl::ActionPtr m_action;
	PartitionEntryVector m_args;
	int m_currentArgIndex;
	pddl::TypedObjectVector m_usedArguments;
	pddl::SimpleEffectVector m_effects;
	std::vector<pddl::FactVector> m_cachedReachableGoals;
	bool m_cachedReachableGoalsComputed;
	std::vector<pddl::TypedObjectVector> m_cachedArgMatches;
	bool m_cachedArgMatchesComputed;
	int m_cachedNumReachable;
	bool m_cachedNumReachableComputed;
	int m_cachedNumReached;
	bool m_cachedNumReachedComputed;
};

} /* namespace goal_planner_gui */

#endif /* H426C2056_AB3B_47C8_AB3D_C26A1D91F9CC */
