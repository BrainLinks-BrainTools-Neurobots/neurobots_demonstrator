/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: action_goal.cpp
 */

#include <goal_planner_gui/alternative_partition_entry.h>
#include <goal_planner_gui/existential_partition_entry.h>
#include <goal_planner_gui/goals/action_goal.h>
#include <goal_planner_gui/goals/alternative_action_goal.h>
#include <goal_planner_gui/goals/goal_context.h>
#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/state/state_variable.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/relaxed_exploration/plan_graph.h>
#include <goal_planner_gui/universal_partition_entry.h>
#include <memory>
#include <unordered_set>

namespace goal_planner_gui
{

ActionGoal::ActionGoal(const std::shared_ptr<GoalContext>& context,
		const pddl::ActionPtr& action,
		const pddl::SimpleEffectVector& effects,
		const PartitionEntryVector& args,
		const pddl::TypedObjectVector& usedArgs,
		bool initial) :
				GoalSpec(context, initial),
				m_action(action),
				m_args(args),
				m_currentArgIndex(0),
				m_usedArguments(usedArgs),
				m_effects(effects),
				m_cachedArgMatchesComputed(false),
				m_cachedNumReachable(0),
				m_cachedNumReachableComputed(false),
				m_cachedNumReached(0),
				m_cachedNumReachedComputed(false),
				m_cachedReachableGoalsComputed(false)
{
}

ActionGoal::~ActionGoal()
{
}

std::string ActionGoal::str() const
{
	std::string argString;
	for (auto& a : m_args)
	{
		if (a)
		{
			argString += a->text() + "(" + std::to_string(a->getMatches().size()) + " matches), ";
		}
		else
		{
			argString += "None, ";
		}
	}
	if (!argString.empty())
	{
		argString.pop_back();
		argString.pop_back();
	}
	return m_context->m_refs->getName(m_action->m_name) + ", " + argString;
}

void ActionGoal::initialGoals(const std::shared_ptr<GoalContext>& context,
		std::vector<std::shared_ptr<GoalSpec>>& goals)
{
	LOG_INFO("Init Action Goals!");
	for (auto& a : context->m_refs->getActions())
	{
		auto goal = goalsFromAction<ActionGoal>(context, a);

		if (!goal->isEmpty())
		{
			goals.push_back(goal);
		}
	}
	LOG_INFO("Found " << goals.size() << " goals");
}

//_next
void ActionGoal::getNext(std::vector<std::shared_ptr<GoalSpec>>& res)
{
	res.clear();

	PartitionEntryVector prefix, suffix;
	getCompletedArgs(prefix);
	PartitionEntryPtr expandedArg = getCurrentArg();

//	LOG_INFO("Args:")
//	LOG_INFO_CONTAINER_STR(m_args);

	if (!expandedArg)
	{
//		LOG_INFO("!expandedArg");
		int numReachable = getNumReachable();
		if (isReachable(numReachable) && !isReached(numReachable))
		{
			res.push_back(shared_from_this());
			return;
		}
		return;
	}

//	LOG_INFO("Current arg: " << expandedArg);
//	LOG_INFO("Matches:");
//	LOG_INFO_CONTAINER_STR(expandedArg->getMatches());

	PartitionEntryVector expansions;
	expandedArg->expand(expansions, allowUniversalExpansion(expandedArg));

	getFutureArgs(suffix);

	pddl::TypedObjectUnorderedSet reachableObjects;
	getReachableObjects(reachableObjects, expandedArg);
	size_t lenExpansion = expansions.size();

	for (auto& entry : expansions)
	{
//		LOG_INFO(entry->str())
//		LOG_INFO("Expanded: " << entry->str());
//		LOG_INFO("Matches:");
//		LOG_INFO_CONTAINER_STR(entry->getMatches());

		if (entry->isEmpty())
		{
			continue;
		}

		entry->setReachable(reachableObjects);

		if (lenExpansion > 1 && entry->isQuantified() && entry->isUnique())
		{
			continue;
		}

		PartitionEntryVector newArgs(prefix);
		newArgs.push_back(entry);
		for (auto& e : suffix)
		{
			if (e)
			{
				newArgs.push_back(e->clone());
			}
			else
			{
				newArgs.push_back(PartitionEntryPtr());
			}
		}

		for (auto& it: newArgs) {
			it->improve();
		}

		std::shared_ptr<GoalSpec> g;
		if (isInstanceSharedCast(entry, altEntry, AlternativePartitionEntry))
		{
			g = getAlternativeChild(newArgs, altEntry->getForbidPartition());
			g->setFixScore(-999999);
		}
		else
		{
			g = getChild(newArgs);
//			if (isInstanceSharedCast(entry, exEntry, ExistentialPartitionEntry))
//			{
//				g->setFixScore(-300000);
//			}
		}

		int numReachable = g->getNumReachable();
		bool isReachable = g->isReachable(numReachable);
		bool isReached = g->isReached(numReachable);

//		LOG_INFO("Goal: " << g->str());
//		LOG_INFO("isReachable: " << isReachable);
//		LOG_INFO("isReached: " << isReached);

//		LOG_INFO(m_action->str() << " " << isReachable << " " << isReached);

		if (isReachable && !isReached)
		{
			res.push_back(g);
		}
	}

//	LOG_INFO("Found " <<res.size());

	//only one option
	if (res.size() == 1)
	{
		auto next = res[0];
		res.clear();
		next->getNext(res);
	}
}

void ActionGoal::getCompletedArgs(PartitionEntryVector& args)
{
	for (auto& arg : m_args)
	{
		if (arg->isLeaf())
		{
			args.push_back(arg);
		}
		else
		{
			return;
		}
	}
}

void ActionGoal::getFutureArgs(PartitionEntryVector& args)
{
	bool foundCurrent = false;
	for (auto& arg : m_args)
	{
		if (foundCurrent)
		{
			args.push_back(arg);
		}
		else if (!arg->isLeaf())
		{
			foundCurrent = true;
		}
	}
}

PartitionEntryPtr ActionGoal::getCurrentArg()
{
	for (auto& arg : m_args)
	{
//		LOG_INFO(arg->str() << " " << arg->isQuantified() << " " << arg->isUnique() << " " << arg->isEmpty() << " " << arg->next(Constants::MAX_SIZE_NEXT));
		if (!arg->isLeaf())
		{
			return arg;
		}
	}

	return PartitionEntryPtr();
}

int ActionGoal::getNumReachable()
{
	if (!m_cachedNumReachableComputed)
	{
		auto& reachable = getReachableGoals();
//		LOG_INFO("Reachables:")
//		for (auto& it2 : reachable)
//		{
//			LOG_INFO("Reachable:");
//			LOG_INFO_CONTAINER_STR(it2);
//		}
		std::vector<pddl::TypedObjectVector> objects;
		std::unordered_map<std::pair<pddl::TypedObjectPtr, int>, std::vector<pddl::FactVector>, PairTypedObjectIntHasher> objectPos;

		for (auto& pe : m_args)
		{
			if (pe)
			{
				objects.push_back(pe->getMatchesAsVector());
			}
			else
			{
				objects.push_back(pddl::TypedObjectVector());
			}
		}

		for (auto& g : reachable)
		{
			ASSERT(g.size() == m_effects.size());
			for (size_t index = 0; index < g.size(); ++index)
			{
				auto& f = g[index];
				auto& e = m_effects[index];

				std::vector<int> ei;
				getArgumentIndices(e, ei);

				size_t i = 0;
				for (auto& a : f->allArgs())
				{
					auto& argI = ei[i];
					auto& objectArgI = objects[argI == -1 ? objects.size() - 1 : argI];
					if (std::find(objectArgI.begin(), objectArgI.end(), a) != objectArgI.end())
					{
						auto& v = objectPos[ { a, argI }];
						v.push_back(g);
					}
					++i;
				}
			}
		}

		m_cachedNumReachable = checkQuantifiedGoalCount(reachable, m_args, objectPos);
		m_cachedNumReachableComputed = true;
	}

	return m_cachedNumReachable;
}

int ActionGoal::getNumReached()
{
	if (!m_cachedNumReachedComputed)
	{
		//potential goals
		std::vector<pddl::FactVector> potentialGoals;
		for (auto& a : getMatches())
		{
			potentialGoals.resize(potentialGoals.size() + 1);
			getEffectsFromArg(a, potentialGoals.back());
		}

		//reached
		std::vector<pddl::FactVector> reached;
		for (auto& g : potentialGoals)
		{
			bool res = true;
			for (auto& f : g)
			{
				if (!m_context->m_init->contains(f))
				{
					res = false;
					break;
				}
			}
			if (res)
			{
				reached.push_back(g);
			}
		}

		std::vector<pddl::TypedObjectVector> objects;
		std::unordered_map<std::pair<pddl::TypedObjectPtr, int>, std::vector<pddl::FactVector>, PairTypedObjectIntHasher> objectPos;

		for (auto& pe : m_args)
		{
			if (pe)
			{
				objects.push_back(pe->getMatchesAsVector());
			}
			else
			{
				objects.push_back(pddl::TypedObjectVector());
			}
		}

		for (auto& g : reached)
		{
			ASSERT(g.size() == m_effects.size());
			for (size_t index = 0; index < g.size(); ++index)
			{
				auto& f = g[index];
				auto& e = m_effects[index];

				std::vector<int> ei;
				getArgumentIndices(e, ei);

				size_t i = 0;
				bool added = false;
				for (auto& a : f->allArgs())
				{
					auto& argI = ei[i];
					auto& objectArgI = objects[argI == -1 ? objects.size() - 1 : argI];
					if (std::find(objectArgI.begin(), objectArgI.end(), a) != objectArgI.end())
					{
						auto& v = objectPos[ { a, argI }];
						v.push_back(g);
						added = true;
					}
					++i;
				}
				if (added)
				{
					break;
				}
			}
		}

		m_cachedNumReached = checkQuantifiedGoalCount(reached, m_args, objectPos);
		m_cachedNumReachedComputed = true;
	}

	return m_cachedNumReached;
}

bool ActionGoal::allowUniversalExpansion(const PartitionEntryPtr& entry)
{
	for (auto& e : m_effects)
	{
		PartitionEntryVector pForE;
		getPartitionsForEffect(e, pForE);
		if (entry == pForE.back())
		{
			return false;
		}
	}
	return true;
}

const std::vector<pddl::FactVector>& ActionGoal::getReachableGoals()
{
	if (!m_cachedReachableGoalsComputed)
	{
		bool result;
		for (auto& arg : getMatches())
		{
//			LOG_INFO_CONTAINER_TEXT_STR("Args: ", arg)
			pddl::FactVector potentialGoal;
			getEffectsFromArg(arg, potentialGoal);

			result = true;
			for (auto& f : potentialGoal)
			{
//				LOG_INFO(f->str());
				if (!m_context->m_rpg->isRelaxedReachable(f))
				{
//					LOG_INFO("not reachable")
					result = false;
					break;
				}
			}

			if (result)
			{
//				LOG_INFO_CONTAINER_TEXT_STR("reachable: ", potentialGoal);
				m_cachedReachableGoals.push_back(potentialGoal);
			}
		}
		m_cachedReachableGoalsComputed = true;
	}
	return m_cachedReachableGoals;
}

void ActionGoal::getReachableObjects(pddl::TypedObjectUnorderedSet& result,
		const PartitionEntryPtr& arg)
{
	int index = pddl::index(m_args, arg);
	for (auto& args : getMatches())
	{
		bool res = true;
		pddl::FactVector effs;
		getEffectsFromArg(args, effs);
		for (auto& f : effs)
		{
			if (!m_context->m_rpg->isRelaxedReachable(f))
			{
				res = false;
				break;
			}
		}
		if (res)
		{
			result.insert(args[index]);
		}
	}
}

const std::vector<pddl::TypedObjectVector>& ActionGoal::getMatches()
{
	if (m_cachedArgMatchesComputed)
		return m_cachedArgMatches;

	std::vector<pddl::TypedObjectVector> argList;
	for (auto& a : m_args)
	{
		if (a)
		{
			argList.push_back(a->getMatchesAsVector());
		}
		else
		{
			argList.push_back(pddl::TypedObjectVector(1)); //one Null object
		}

		if (argList.back().empty())
		{
			return m_cachedArgMatches;
		}
	}

	helpers::ProductVectorOfVectors<pddl::TypedObjectPtr>().compute(argList, m_cachedArgMatches);
	m_cachedArgMatchesComputed = true;

//	LOG_INFO("Found " << m_cachedArgMatches.size() << " arg matches");

	return m_cachedArgMatches;
}

std::shared_ptr<GoalSpec> ActionGoal::getChild(const PartitionEntryVector& args)
{
	return std::shared_ptr<GoalSpec>(new ActionGoal(m_context, m_action, m_effects, args, m_usedArguments));
}

std::shared_ptr<GoalSpec> ActionGoal::getAlternativeChild(const PartitionEntryVector& args,
		const std::shared_ptr<Partition>& forbidPartition)
{
	return std::shared_ptr<GoalSpec>(new AlternativeActionGoal(m_context, m_action, m_effects, args, m_usedArguments, forbidPartition));
}

void ActionGoal::getPartitionsForEffect(const std::shared_ptr<pddl::SimpleEffect>& effect,
		PartitionEntryVector& partitions)
{
	std::vector<int> indices;
	getArgumentIndices(effect, indices);
	partitions.reserve(indices.size());
	for (auto& i : indices)
	{
		if (i < 0)
		{
			partitions.push_back(m_args.back());
		}
		else
		{
			partitions.push_back(m_args[i]);
		}
	}
}

void ActionGoal::getArgumentIndices(const std::shared_ptr<pddl::Literal>& literal,
		std::vector<int>& indices)
{
	pddl::FunctionPtr function;
	pddl::TermVector fargs;
	std::shared_ptr<pddl::Predicate> modality;
	pddl::TermVector modargs;
	pddl::TermPtr value;
	pddl::StateVariable::svarArgsFromLiteral(literal, function, fargs, modality, modargs, value);

	indices.reserve(fargs.size() + 1);
	for (auto& a : fargs)
	{
		if (isInstanceSharedCast(a, constA, pddl::ConstantTerm))
		{
			indices.push_back(pddl::index(m_usedArguments, constA->getObj()));
		}
		else if (isInstanceSharedCast(a, varA, pddl::VariableTerm))
		{
			indices.push_back(pddl::index(m_usedArguments, varA->getObject()));
		}
		else
		{
			ASSERT(false);
		}
	}

	if (isInstanceSharedCast(value, constA, pddl::ConstantTerm))
	{
		indices.push_back(pddl::index(m_usedArguments, constA->getObj()));
	}
	else if (isInstanceSharedCast(value, varA, pddl::VariableTerm))
	{
		indices.push_back(pddl::index(m_usedArguments, varA->getObject()));
	}
	else
	{
		ASSERT(false);
	}
}

void ActionGoal::getEffectsFromArg(const pddl::TypedObjectVector& args,
		pddl::FactVector& effects)
{
	for (auto& effect : m_effects)
	{
		std::vector<int> indices;
		getArgumentIndices(effect, indices);

		pddl::TypedObjectVector newArgs;
		newArgs.reserve(indices.size());
		for (auto& i : indices)
		{
			if (i >= 0)
			{
				newArgs.push_back(args[i]);
			}
			else
			{
				newArgs.push_back(pddl::TypedObjectPtr());
			}
		}

		//replace vars
		pddl::FunctionPtr function;
		pddl::TermVector litargs;
		pddl::FunctionPtr modality;
		pddl::TermVector modalArgs;
		pddl::TermPtr value;
		bool negated;
		pddl::Literal::getLiteralElements(effect, function, litargs, modality, modalArgs, value, negated);

		ASSERT(!negated);
		pddl::TypedObjectVector replacementArgs;
		const size_t endIndex = std::min(litargs.size(), newArgs.size() - 1);
		for (size_t i = 0; i < endIndex; ++i)
		{
			auto& oldArg = litargs[i];
			auto& newArg = newArgs[i];

			if (newArg)
			{
				replacementArgs.push_back(newArg);
			}
			else
			{
				//check term for object
				if (isInstanceSharedCast(oldArg, vArg, pddl::VariableTerm))
				{
					replacementArgs.push_back(vArg->getObject());

				}
				else if (isInstanceSharedCast(oldArg, cArg, pddl::ConstantTerm))
				{
					replacementArgs.push_back(cArg->getObj());
				}
				else
				{
					throw pddl::Exception("Term has no object attribute");
				}
			}
		}

		//check term for object
		pddl::TypedObjectPtr replacementValue;

		if (newArgs.back())
		{
			replacementValue = newArgs.back();
		}
		else
		{
			if (isInstanceSharedCast(value, vvalue, pddl::VariableTerm))
			{
				replacementValue = vvalue->getObject();

			}
			else if (isInstanceSharedCast(value, cvalue, pddl::ConstantTerm))
			{
				replacementValue = cvalue->getObj();
			}
			else
			{
				throw pddl::Exception("Term has no object attribute");
			}
		}

		effects.push_back(pddl::FactPtr(new pddl::Fact(pddl::StateVariablePtr(
				new pddl::StateVariable(function, replacementArgs)), replacementValue)));
	}
}

void ActionGoal::filterPreconditions(const std::vector<pddl::TypedObjectVector>& input,
		std::vector<pddl::TypedObjectUnorderedSet>& out)
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

int ActionGoal::checkQuantifiedGoalCount(const std::vector<pddl::FactVector>& goals,
		const PartitionEntryVector& args,
		std::unordered_map<std::pair<pddl::TypedObjectPtr, int>, std::vector<pddl::FactVector>, PairTypedObjectIntHasher>& goalByObjectPos)
{
	std::vector<pddl::TypedObjectUnorderedSet> objects;
	for (auto& pe : args)
	{
		if (pe)
		{
			objects.push_back(pe->getMatches());
		}
		else
		{
			objects.push_back(pddl::TypedObjectUnorderedSet());
		}
	}

	std::unordered_set<pddl::FactVector, FactVectorHasher> goalsSet(goals.begin(), goals.end());
	return checkQuantifiedGoalCountRec(args, goalByObjectPos, 0, goalsSet, objects);
}

const PartitionEntryVector& ActionGoal::allPartitions()
{
	return m_args;
}

int ActionGoal::argIndex(const PartitionEntryPtr& arg)
{
	return pddl::index(m_args, arg);
}

const PartitionEntryVector& ActionGoal::getArgs()
{
	return m_args;
}

PartitionEntryPtr ActionGoal::getArg(int index)
{
	auto& pe = m_args[index];
	if (isInstanceSharedCast(pe, aarg, AlternativePartitionEntry))
	{
		return aarg->getForbidPartition()->getExpandParent();
	}
	return pe;
}

pddl::ConditionPtr ActionGoal::pddlGoal()
{
	std::vector<pddl::ConditionPtr> pddlGoals;
	for (auto& goal : getReachableGoals())
	{
		for (auto& f : goal)
		{
			if (!m_context->m_init->contains(f))
			{
				if (goal.size() == 1)
				{
					pddlGoals.push_back(goal[0]->toCondition());
				}
				else
				{
					std::vector<pddl::ConditionPtr> conjParts;
					for (auto& f2 : goal)
					{
						LOG_INFO(f2->str() << " => " << f2->toCondition()->str());
						conjParts.push_back(f2->toCondition());
					}
					pddlGoals.push_back(pddl::ConditionPtr(new pddl::Conjunction(conjParts)));
				}
				break;
			}
		}
	}
	return pddl::ConditionPtr(new pddl::Disjunction(pddlGoals));
}

int ActionGoal::checkQuantifiedGoalCountRec(const PartitionEntryVector& args,
		std::unordered_map<std::pair<pddl::TypedObjectPtr, int>, std::vector<pddl::FactVector>, PairTypedObjectIntHasher>& goalByObjectPos,
		int i,
		const std::unordered_set<pddl::FactVector, FactVectorHasher>& facts,
		const std::vector<pddl::TypedObjectUnorderedSet>& objects)
{
	if (i >= args.size())
	{
		return facts.size();
	}

	if (!args[i])
	{
		return checkQuantifiedGoalCountRec(args, goalByObjectPos, i + 1, facts, objects);
	}
	else if (isInstanceSharedCast(args[i], exPe, ExistentialPartitionEntry))
	{
		int maxCount = -1;
		std::unordered_set<pddl::FactVector, FactVectorHasher> facts2;
		for (auto& o: objects[i])
		{
//			ASSERT(CONTAINS(std::make_pair(o, i), goalByObjectPos));
			auto& goalByOPos = goalByObjectPos[
			{	o, i}];

			for (auto& g: goalByOPos)
			{
				if (CONTAINS(g, facts))
				{
					facts2.insert(g);
				}
			}
			int count = checkQuantifiedGoalCountRec(args, goalByObjectPos, i + 1, facts2, objects);
			if (count > maxCount)
			{
				maxCount = count;
			}
		}
		return maxCount;
	}
	else if (isInstanceSharedCast(args[i], unPe, UniversalPartitionEntry))
	{
		int minCount = facts.size() + 1;
		std::unordered_set<pddl::FactVector, FactVectorHasher> facts2;
		for (auto& o: objects[i])
		{
//			ASSERT(CONTAINS(std::make_pair(o, i), goalByObjectPos));
			auto& goalByOPos = goalByObjectPos[
			{	o, i}];

			for (auto& g: goalByOPos)
			{
				if (CONTAINS(g, facts))
				{
					facts2.insert(g);
				}
			}
			int count = checkQuantifiedGoalCountRec(args, goalByObjectPos, i + 1, facts2, objects);
			if (count < minCount)
			{
				minCount = count;
			}
		}
		return minCount;
	}
	else
	{
		return checkQuantifiedGoalCountRec(args, goalByObjectPos, i + 1, facts, objects);
	}
}

const pddl::ActionPtr& ActionGoal::getAction() const
{
	return m_action;
}

}
/* namespace goal_planner_gui */
