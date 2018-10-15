/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 6, 2017
 *      Author: kuhnerd
 * 	  Filename: condition.h
 */

#ifndef HFA7CAF3F_C071_4C0E_8D00_058B1B34A586
#define HFA7CAF3F_C071_4C0E_8D00_058B1B34A586
#include <goal_planner_gui/pddl/base_element.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/types.h>

namespace pddl
{

class Element;
class Scope;
class LiteralCondition;

class Condition: public BaseElement
{
public:
	Condition();
	virtual ~Condition();

	virtual std::shared_ptr<Condition> negated() = 0;

	static std::shared_ptr<Condition> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope);

	virtual void collectConditions(std::vector<std::shared_ptr<LiteralCondition>>& res);
	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("BaseElement");
		BaseElement::getParents(parents);
	}

	virtual std::string str() const;
};

VECTOR_DEF(Condition);
POINTER_DEF(Condition);
typedef std::unordered_set<std::shared_ptr<Condition>, BaseElementHasher> ConditionUnorderedSet;

} /* namespace pddl */

#endif /* HFA7CAF3F_C071_4C0E_8D00_058B1B34A586 */
