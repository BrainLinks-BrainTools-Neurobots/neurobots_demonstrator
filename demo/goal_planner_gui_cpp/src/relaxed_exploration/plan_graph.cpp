/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 19, 2017
 *      Author: kuhnerd
 * 	  Filename: plan_graph.cpp
 */
#include <boost/type_traits/remove_cv.hpp>
#include <boost/variant/get.hpp>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/conditions/conjunction.h>
#include <goal_planner_gui/pddl/conditions/disjunction.h>
#include <goal_planner_gui/pddl/conditions/literal_condition.h>
#include <goal_planner_gui/pddl/conditions/quantified_condition.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/state/state.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/visitors.h>
#include <goal_planner_gui/relaxed_exploration/plan_graph.h>
#include <goal_planner_gui/relaxed_exploration/proposition.h>
#include <ctime>
#include <memory>
#include <queue>

namespace goal_planner_gui
{

class ReachedActionKeyHasher
{
public:

	std::size_t operator()(const std::pair<std::string, std::vector<pddl::TypedObjectPtr>>& obj) const noexcept
	{
		static pddl::TypedObjectVectorWithInstanceHasher hasher;
		size_t hash = 0;
		pddl::hash_combine(hash, obj.first, hasher(obj.second));
		return hash;
	}
};

struct CompareHeapEntry
{
	double operator()(const std::pair<double, PropositionPtr>& n1,
			const std::pair<double, PropositionPtr>& n2) const noexcept
			{
		return n1.first > n2.first;
	}
};

inline bool operator ==(std::pair<std::string, std::vector<pddl::TypedObjectPtr>> const& lhs,
		std::pair<std::string, std::vector<pddl::TypedObjectPtr>> const& rhs)
{
	return lhs.first == rhs.first && lhs.second == rhs.second;
}

enum CheckInstFuncReturn
{
	True,
	False,
	None
};

bool isInstantiatedFunction(pddl::TermPtr& term)
{
	dynamicSharedPointerCast(term, fterm, pddl::FunctionVariableTerm);
	return term->t() == pddl::TermTypes::FunctionTerm || (fterm && fterm->isInstantiated());
}

pddl::TypedObjectPtr getInstance(const pddl::TermPtr& term)
{
	if (isInstanceSharedCast(term, varterm, pddl::VariableTerm))
	{
		return varterm->getInstance();
	}
	else if (isInstanceSharedCast(term, varterm, pddl::ConstantTerm))
	{
		return varterm->getInstance();
	}
	else
	{
		throw pddl::Exception("Term has no instance!");
	}
}

CheckInstFuncReturn checkInstFunc(const pddl::ConditionPtr& cond,
		std::vector<std::tuple<pddl::TypedObjectPtr, pddl::TypedObjectPtr, pddl::ConditionPtr>>& forced,
		const pddl::StatePtr& init,
		const pddl::FunctionUnorderedSet& staticFunctions,
		std::unordered_map<pddl::ConditionPtr, std::unordered_set<pddl::ParameterPtr, pddl::TypedObjectHasher>, pddl::BaseElementHasher>& freeArgs,
		const pddl::FactUnorderedSet& reachable,
		std::vector<std::vector<pddl::ParameterPtr>>& nextCandidates,
		PlanGraph::SmartInstantiateFunctionData& data)
{
//	auto hasher = data.checked.hash_function();
	if (!cond || CONTAINS(cond, data.checked))
	{
		return True;
	}

	if (isInstanceSharedCast(cond, litcond, pddl::LiteralCondition))
	{
		if (litcond->m_predicate == pddl::Builtin::equals() && isInstantiatedFunction(litcond->m_args[0]))
		{
			if (isInstanceSharedCast(litcond->m_args[1], varterm1, pddl::VariableTerm))
			{
				auto& v = litcond->m_args.back();
				dynamicSharedPointerCast(v, back, pddl::VariableTerm);
				dynamicSharedPointerCast(litcond->m_args[0], funcArg, pddl::FunctionTerm);
				ASSERT(funcArg);

				bool res = true;

				//if all(a.is_instantiated() for a in cond.args[0].args if isinstance(a, VariableTerm)):
				for (auto& a: funcArg->getArgs())
				{
					if (isInstanceSharedCast(a, varA, pddl::VariableTerm))
					{
						if (!varA->isInstantiated())
						{
							res = false;
							break;
						}
					}
				}

				if (res && back && !back->isInstantiated())
				{
					auto f = pddl::Function::getFunction(litcond);
					if (CONTAINS(f, staticFunctions))
					{
						auto svar = pddl::StateVariable::fromLiteral(litcond, init);
						forced.push_back(std::make_tuple(back->getObject(), init->get(svar), cond));
					}
				}
			}
		}

		bool res = true;
		for (auto& a: freeArgs[cond])
		{
			if (!a->isInstantiated())
			{
				res = false;
				break;
			}
		}

		if (res)
		{
			if (pddl::Function::getFunction(litcond) == pddl::Builtin::equals())
			{
				pddl::TypedObjectPtr term0Instance = getInstance(litcond->m_args[0]);
				pddl::TypedObjectPtr term1Instance = getInstance(litcond->m_args[1]);
				ASSERT(term0Instance);
				ASSERT(term1Instance);
				if ((term0Instance == term1Instance) ^ litcond->m_negated)
				{
					data.checked.insert(cond->getCopy<pddl::Condition>() );
					return True;
				}

				return False;
			}

			pddl::FactPtr fact = pddl::Fact::fromLiteral(litcond, init);

			dynamicSharedPointerCast(fact->getSvar()->getFunction(), svarFunc, pddl::Predicate);
			if (svarFunc && fact->getValue() == pddl::DefaultTypedObjects::falseObject())
			{
				//pass
			}
			else if (CONTAINS_NOT(fact, reachable))
			{
				return False;
			}
			data.checked.insert(cond->getCopy<pddl::Condition>() );
			return True;
		}
		else
		{
			nextCandidates.resize(nextCandidates.size() + 1);
			auto& vec = nextCandidates.back();
			for (auto& a: freeArgs[cond])
			{
				if(!a->isInstantiated())
				{
					vec.push_back(a);
				}
			}
		}
	}
	else if (isInstanceSharedCast(cond, conj, pddl::Conjunction))
	{
		std::vector<CheckInstFuncReturn> results;
		for (auto& c: conj->getParts())
		{
			results.push_back(checkInstFunc(c, forced, init, staticFunctions, freeArgs, reachable, nextCandidates, data));
		}

		for (auto& c: results)
		{
			if (c == False)
			{
				return False;
			}
		}

		bool allTrue = true;
		for (auto& c: results)
		{
			if (c != True)
			{
				allTrue = false;
				break;
			}
		}

		if (allTrue)
		{
			data.checked.insert(cond->getCopy<pddl::Condition>() );
			return True;
		}
	}
	else if (isInstanceSharedCast(cond, disj, pddl::Disjunction))
	{
		std::vector<CheckInstFuncReturn> results;
		for (auto& c: disj->getParts())
		{
			results.push_back(checkInstFunc(c, forced, init, staticFunctions, freeArgs, reachable, nextCandidates, data));
		}

		bool allFalse = true;
		for (auto& c: results)
		{
			if (c == True)
			{
				data.checked.insert(cond->getCopy<pddl::Condition>() );
				return True;
			}
			else if (c != False)
			{
				allFalse = false;
				break;
			}
		}

		LOG_ERROR("Please change for loop to match conjunction");
		throw pddl::NotImplementedException(FILE_AND_LINE);

		if (allFalse)
		{
			return False;
		}
	}
	else if (isInstanceSharedCast(cond, disj, pddl::QuantifiedCondition))
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else
	{
		ASSERT(false);
	}

	return None;
}

PlanGraph::PlanGraph(const pddl::ProblemPtr& problem) :
				m_problem(problem),
				m_domain(problem->getDomain()),
				m_init(pddl::State::fromProblem(problem))
{
	computeActionPreconds();
	computeReachable();
	computeRelaxedDistance();
}

PlanGraph::~PlanGraph()
{
}

void PlanGraph::computeActionPreconds()
{
	pddl::FunctionPtr f;
	pddl::TermVector args;
	std::shared_ptr<pddl::Predicate> modality;
	pddl::TermVector modargs;
	pddl::TermPtr value;

	m_precondLookup.clear();

	for (auto& a : m_domain->m_actions)
	{
		std::vector<std::shared_ptr<pddl::LiteralCondition>> cond;
		a->m_precondition->collectConditions(cond);
		for (auto& c : cond)
		{
			if (pddl::Literal::getFunction(c) == pddl::Builtin::equals())
			{
				continue;
			}

			pddl::StateVariable::svarArgsFromLiteral(c, f, args, modality, modargs, value);
			args.push_back(value);

			auto& mapping = m_precondLookup[a][f];
			for (auto& arg : args)
			{
				if (isInstanceSharedCast(arg, carg, pddl::ConstantTerm))
				{
					mapping.push_back(carg->getObj());
				}
				else if (isInstanceSharedCast(arg, varg, pddl::VariableTerm))
				{
					mapping.push_back(pddl::index(a->m_args, varg->getObject()));
				}
				else
				{
					throw pddl::Exception("Please check python!");
				}
			}
		}
	}
}

void PlanGraph::computeReachable()
{
	pddl::FunctionUnorderedSet staticFunctions;
	for (auto& it : *m_domain->m_functions)
	{
		for (auto& it2 : it.second)
		{
			staticFunctions.insert(it2);
		}
	}

	for (auto& it : *m_domain->m_predicates)
	{
		for (auto& it2 : it.second)
		{
			staticFunctions.insert(it2);
		}
	}

	for (auto& a : m_domain->m_actions)
	{
		std::vector<std::shared_ptr<pddl::SimpleEffect>> effects;
		a->m_effect->collectEffects(effects);
		for (auto& eff : effects)
		{
			staticFunctions.erase(pddl::Literal::getFunction(eff));
		}
	}

	std::unordered_set<std::pair<std::string, pddl::TypedObjectVector>, ReachedActionKeyHasher> reachedActions;
	auto reachableList = m_init->iterfacts();
	pddl::FactUnorderedSet reachable(reachableList.begin(), reachableList.end());
//	LOG_INFO("Reachable:");
//	LOG_INFO_CONTAINER(reachable);
	m_props.clear();
	for (auto& f : reachable)
	{
		m_props[f].reset(new Proposition(f, { }));
	}

	bool changed = true;
	auto t0 = clock();

	while (changed)
	{
		auto t1 = clock();
		changed = false;

		for (auto& action : m_domain->m_actions)
		{
			SmartInstantiateFunctionData data; //partially not needed?
			auto instFunc = getInstFunc(action, reachable, staticFunctions, data);

			std::vector<pddl::TypedObjectUnorderedSet> allArgs(action->m_args.size());
			size_t i = 0;
			for (auto& a : action->m_args)
			{
				m_problem->getAllObjects(a->getType(), allArgs[i++]);
			}

			if (action->smartInstantiateBegin(instFunc, action->m_args, allArgs, m_problem))
			{
				std::unordered_map<pddl::TypedObjectPtr, pddl::TypedObjectPtr, pddl::TypedObjectHasher> m;

				while (action->smartInstantiateNext(m))
				{
					pddl::TypedObjectVector args;
					args.reserve(action->m_args.size());
					for (auto& a : action->m_args)
					{
						args.push_back(m[a]);
					}
					auto key = std::make_pair(action->m_name, args);
					if (CONTAINS_NOT(key, reachedActions))
					{
						reachedActions.insert(key);

						UnaryOpVector ops;
						UnaryOp::fromAction(action, args, ops);
						for (auto& op : ops)
						{
							m_ops.push_back(op);

							if (std::find(reachable.begin(), reachable.end(), op->m_effect) == reachable.end())
							{
								changed = true;
//								LOG_INFO("Add: " << op->m_effect->str());
								reachable.insert(op->m_effect);
								m_props[op->m_effect].reset(new Proposition(op->m_effect, { }));
							}

							op->m_effectProposition = m_props[op->m_effect];

							for (auto& c : op->m_conditions)
							{
								if (CONTAINS_NOT(c, m_props))
								{
									m_props[c].reset(new Proposition(c, { }));
								}

								m_props[c]->m_children.push_back(op);
							}
						}
					}
				}
			}
		}

		LOG_INFO("iteration: " << std::to_string(double(clock() - t1) / CLOCKS_PER_SEC) << "s");
	}

	LOG_INFO("exploration: " << std::to_string(double(clock() - t0) / CLOCKS_PER_SEC) << "s");
	LOG_INFO("num reachable: " << (reachable.size() - m_init->size()));

	m_facts.reserve(m_props.size());
	for (auto& it : m_props)
	{
		m_facts.push_back(it.second);
	}

	m_reachable = reachable;
}

void PlanGraph::computeRelaxedDistance()
{
//	t0 = time.time()
//	        for op in self.ops:
//	            op.reset()
//
//	        queue = []
//	        for prop in self.facts:
//	            prop.reset()
//	            if prop.fact in self.init:
//	                # print prop, map(str, prop.children)
//	                prop.costs = 0
//	                heapq.heappush(queue, (0, prop))
//
//	        while queue:
//	            _, prop = heapq.heappop(queue)
//	            if prop.has_fired:
//	                continue
//
//	            # print prop
//	            for op in prop.fire():
//	                # print "fire:", op
//	                if op.effect.costs == -1 or op.h_add_costs < op.effect.costs:
//	                    op.effect.costs = op.h_add_costs
//	                    op.effect.reached_by = op
//	                    heapq.heappush(queue, (op.h_add_costs, op.effect))
//
//	        logger.debug("h_add: %f", (time.time()-t0))

	for (auto& op : m_ops)
	{
		op->reset();
	}

	std::priority_queue<std::pair<double, PropositionPtr>, std::vector<std::pair<double, PropositionPtr>>, CompareHeapEntry> queue;
	for (auto& prop : m_facts)
	{
		prop->reset();
		if (m_init->contains(prop->m_fact))
		{
			prop->m_costs = 0;
			queue.push( { 0, prop });
		}
	}

	while (!queue.empty())
	{
		auto& prop = queue.top().second;
		queue.pop();

		if (prop->m_hasFired)
		{
			continue;
		}

		UnaryOpVector ops;
		prop->fire(ops);
		for (auto& op : ops)
		{
			if (op->m_effectProposition->m_costs == -1 || op->m_hAddCosts < op->m_effectProposition->m_costs)
			{
				op->m_effectProposition->m_costs = op->m_hAddCosts;
				op->m_effectProposition->m_reachedBy = op;
				queue.push( { op->m_hAddCosts, op->m_effectProposition });
			}
		}
	}
}

bool PlanGraph::isRelaxedReachable(const pddl::FactPtr& fact)
{
	return (m_init->contains(fact) || CONTAINS(fact, m_props));
}

double PlanGraph::getFFDistance(const pddl::FactVector& facts)
{
	UnaryOpVector plan;
	for (auto& f : facts)
	{
		if (!m_init->contains(f))
		{
			collectRelaxedPlan(m_props[f], plan);
		}
	}

	double sum = 0;
	for (auto& op : plan)
	{
		sum += op->m_opCosts;
	}
	return sum;
}

void PlanGraph::collectRelaxedPlan(const PropositionPtr& prop,
		UnaryOpVector& plan)
{
	auto& op = prop->m_reachedBy;
	if (op)
	{
		for (auto& c : op->m_conditions)
		{
			collectRelaxedPlan(m_props[c], plan);
		}
		plan.push_back(op);
	}
}

pddl::Scope::SmartInstantiateFunction PlanGraph::getInstFunc(const pddl::ActionPtr& action,
		const pddl::FactUnorderedSet& reachable,
		const pddl::FunctionUnorderedSet& staticFunctions,
		SmartInstantiateFunctionData& data)
{
	static std::unordered_set<std::shared_ptr<pddl::Parameter>, pddl::TypedObjectHasher> tmp;

	pddl::Visitors<pddl::Condition, pddl::Parameter, pddl::TypedObjectHasher>::VisitorFunction subcondVisitor =
			[&data](pddl::ConditionPtr& cond,
					std::unordered_set<std::shared_ptr<pddl::Parameter>, pddl::TypedObjectHasher>& result)
			{
				const std::unordered_set<pddl::ParameterPtr, pddl::TypedObjectHasher>& free = cond->free();
				for (auto& arg : free)
				{
					data.condByArg[arg].insert(cond);
				}

				if (isInstanceSharedCast(cond, litCond, pddl::LiteralCondition))
				{
					data.freeArgs[litCond] = free;
				}
			};

	auto& condition = action->m_precondition;
	pddl::Visitors<pddl::Condition, pddl::Parameter, pddl::TypedObjectHasher>::visit(condition, subcondVisitor, tmp);

	return [&data, &action, &staticFunctions, &reachable, this](std::unordered_map<pddl::TypedObjectPtr, pddl::TypedObjectPtr, pddl::TypedObjectHasher>& mapping,
			const std::vector<pddl::TypedObjectPtr>& params,
			pddl::TypedObjectPtr& next,
			pddl::TypedObjectPtr& nextval) mutable -> void
	{
		std::vector<std::vector<pddl::ParameterPtr>> nextCandidates;
		if (!data.checked.empty())
		{
			for (auto& it: data.prevMapping)
			{
				if (pddl::unordered_helpers::getDefault(mapping, it.first, pddl::TypedObjectPtr()) != it.second)
				{
					for (auto& toDelete: data.condByArg[it.first])
					{
						for (auto it2 = data.checked.begin(); it2 != data.checked.end(); )
						{
							if (toDelete->hash() == it2->get()->hash()) //check without instances!
							{
								it2 = data.checked.erase(it2);
							}
							else
							{
								++it2;
							}
						}
					}
				}
			}
		}
		pddl::unordered_helpers::makeUnionInline(data.prevMapping, mapping);

		pddl::ConditionPtr& condition = action->m_precondition;
		std::vector<std::tuple<pddl::TypedObjectPtr, pddl::TypedObjectPtr, pddl::ConditionPtr>> forced;
		CheckInstFuncReturn result = checkInstFunc(condition, forced, m_init, staticFunctions, data.freeArgs, reachable, nextCandidates, data);

		if (result == True)
		{
			data.checked.insert(condition->getCopy<pddl::Condition>());
//			LOG_INFO("Add " << condition->hashWithInstance()<< ", this: " << static_cast<void*>(condition.get()) << " " << condition->getCopy<pddl::Condition>()->hash());
			next = pddl::DefaultTypedObjects::__returnObjectTrue();
			nextval.reset();
			return;
		}
		else if (result == False)
		{
			next.reset();
			nextval.reset();
			return;
		}

		if (!forced.empty())
		{
			auto& tuple = forced[0];
			data.checked.insert(std::get<2>(tuple)->getCopy<pddl::Condition>());
//			LOG_INFO("Add " << std::get<2>(tuple)->hashWithInstance()<< ", this: " << static_cast<void*>(std::get<2>(tuple).get()) << " " << std::get<2>(tuple)->getCopy<pddl::Condition>()->hash());
			next = std::get<0>(tuple);
			nextval = std::get<1>(tuple);
			return;
		}

		if (!nextCandidates.empty())
		{
			int minLength = std::numeric_limits<int>::max();
			std::vector<pddl::ParameterPtr> best;
			for (auto& l: nextCandidates)
			{
				if (l.size() < minLength)
				{
					best = l;
					minLength = l.size();
				}
			}
			next = best[0];
			nextval.reset();
			return;
		}

		next = pddl::DefaultTypedObjects::__returnObjectTrue();
		nextval.reset();
	};
}

} /* namespace goal_planner_gui */
