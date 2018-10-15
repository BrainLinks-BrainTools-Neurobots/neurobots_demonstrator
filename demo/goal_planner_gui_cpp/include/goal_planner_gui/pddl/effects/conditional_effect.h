/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: conditional_effect.h
 */

#ifndef H30D60363_04E4_41C9_8376_49646DACC6BE
#define H30D60363_04E4_41C9_8376_49646DACC6BE
#include <goal_planner_gui/pddl/effects/effect.h>

namespace pddl
{
class ConditionalEffect: public Effect
{
public:
	ConditionalEffect();

	virtual inline std::string getTag() const
	{
		return tag();
	}

	static inline std::string tag()
	{
		return "when";
	}

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;

	static std::shared_ptr<Effect> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	virtual void collectFreeVars(std::vector<std::shared_ptr<Parameter>>& res);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Effect");
		Effect::getParents(parents);
	}

	virtual std::shared_ptr<BaseElement> copy();
};
}

#endif /* H30D60363_04E4_41C9_8376_49646DACC6BE */
