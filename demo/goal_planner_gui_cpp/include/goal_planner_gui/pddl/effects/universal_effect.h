/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: universal_effect.h
 */

#ifndef HEC1C81CF_023C_423D_A67D_01C58B53D6A6
#define HEC1C81CF_023C_423D_A67D_01C58B53D6A6
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/types.h>

namespace pddl
{

class Parameter;

class UniversalEffect: public Scope, public Effect
{
public:
	UniversalEffect(const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& args,
			std::shared_ptr<Effect> effect,
			std::shared_ptr<Scope> parentScope);
	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "forall";
	}

	static std::shared_ptr<Effect> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Effect");
		Effect::getParents(parents);
	}

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& getArgs() const;
	const std::shared_ptr<Effect>& getEffect() const;

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;

	virtual std::shared_ptr<BaseElement> copy();

private:
	std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher> m_args;
	std::shared_ptr<Effect> m_effect;
};
}

#endif /* HEC1C81CF_023C_423D_A67D_01C58B53D6A6 */
