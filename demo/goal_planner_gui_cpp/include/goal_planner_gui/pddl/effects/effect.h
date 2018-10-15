/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 5, 2017
 *      Author: kuhnerd
 * 	  Filename: effect.h
 */

#ifndef H1D345B8C_0BB0_44D3_8376_1A83F845E64C
#define H1D345B8C_0BB0_44D3_8376_1A83F845E64C
#include <goal_planner_gui/pddl/base_element.h>
#include <goal_planner_gui/pddl/types.h>

namespace pddl
{

class Element;
class Parameter;
class SimpleEffect;

class Effect: public BaseElement
{
public:
	Effect();
	virtual ~Effect();

	static std::shared_ptr<Effect> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	virtual std::string str() const;

	virtual void collectEffects(std::vector<std::shared_ptr<SimpleEffect>>& res);
	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("BaseElement");
		BaseElement::getParents(parents);
	}
};

VECTOR_DEF(Effect);
POINTER_DEF(Effect);

}

#endif /* H1D345B8C_0BB0_44D3_8376_1A83F845E64C */
