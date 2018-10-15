/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: simple_effect.h
 */

#ifndef H7EE26A97_B685_4209_83B1_78E2D716352B
#define H7EE26A97_B685_4209_83B1_78E2D716352B
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/literal.h>

namespace pddl
{
class SimpleEffect: public Literal, public Effect
{
public:
	SimpleEffect(std::shared_ptr<Predicate> predicate,
			const std::vector<std::shared_ptr<Term>>& args,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>(),
			bool negated = false);
	virtual ~SimpleEffect();
	virtual size_t hash() const;
	virtual bool operator==(const Effect& rhs) const;
	virtual bool operator==(const Literal& rhs) const;

	static std::shared_ptr<Effect> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	virtual std::string str() const;

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);
	virtual void collectEffects(std::vector<std::shared_ptr<SimpleEffect>>& res);

	virtual void setScope(const std::shared_ptr<Scope>& scope);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("Effect");
		Effect::getParents(parents);
	}

	virtual std::shared_ptr<BaseElement> copy();
};

VECTOR_DEF(SimpleEffect);

}

#endif /* H7EE26A97_B685_4209_83B1_78E2D716352B */
