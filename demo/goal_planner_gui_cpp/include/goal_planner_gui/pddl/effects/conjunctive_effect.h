/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: conjunctive_effect.h
 */

#ifndef HA59F44C1_E4F3_4C3B_AEB0_2A5C3B75E90E
#define HA59F44C1_E4F3_4C3B_AEB0_2A5C3B75E90E
#include <goal_planner_gui/pddl/effects/effect.h>

namespace pddl
{
class ConjunctiveEffect: public Effect
{
public:
	ConjunctiveEffect(const std::vector<std::shared_ptr<Effect>>& effects,
			const std::shared_ptr<Scope>& scope = std::shared_ptr<Scope>());
	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "and";
	}

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;

	static std::shared_ptr<Effect> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Effect");
		Effect::getParents(parents);
	}

	virtual std::shared_ptr<BaseElement> copy();
	const std::vector<std::shared_ptr<Effect> >& getEffects() const;

private:
	std::vector<std::shared_ptr<Effect>> m_effects;
};
}

#endif /* HA59F44C1_E4F3_4C3B_AEB0_2A5C3B75E90E */
