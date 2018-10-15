/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 11, 2018
 *      Author: kuhnerd
 * 	  Filename: helpers.h
 */

#ifndef HD441B845_5714_4105_858F_9CB58586878E
#define HD441B845_5714_4105_858F_9CB58586878E
#include <goal_planner_gui/pddl/state/state.h>
#include <goal_planner_gui/pddl/terms.h>

namespace pddl
{
inline void instantiateArgs(const std::vector<std::shared_ptr<Term>>& args,
		std::vector<std::shared_ptr<TypedObject>>& result,
		const std::shared_ptr<State>& state = std::shared_ptr<State>())
{
	if (state)
	{
		result.reserve(args.size());
		for (auto& arg : args)
		{
			result.push_back(state->evaluateTerm(arg));
		}
		return;
	}

	for (auto& arg : args)
	{
		if (isInstanceSharedCast(arg, a, VariableTerm))
		{
			ASSERT(a->isInstantiated());
			result.push_back(a->getInstance());
		}
		else if (isInstanceSharedCast(arg, a, ConstantTerm ))
		{
			result.push_back(a->getObj());
		}
		else
		{
			throw Exception("couldn't create state variable, " + arg->str() + " is a function term and no state was supplied.");
		}
	}
}
}

#endif /* HD441B845_5714_4105_858F_9CB58586878E */
