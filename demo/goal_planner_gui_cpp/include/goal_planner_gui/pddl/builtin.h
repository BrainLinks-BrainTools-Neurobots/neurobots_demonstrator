/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 29, 2017
 *      Author: kuhnerd
 * 	  Filename: builtin.h
 */

#ifndef H698080C8_7429_4832_85C5_BB609B20C23D
#define H698080C8_7429_4832_85C5_BB609B20C23D
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/types.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pddl
{

class Builtin
{
public:
	static std::vector<std::shared_ptr<Type>> defaultTypes();
	static std::unordered_map<std::string, std::shared_ptr<Type>> defaultTypesAsMap();
	static std::shared_ptr<Predicate> equals();
	static std::shared_ptr<Predicate> assign();
	static std::shared_ptr<Predicate> equalAssign();
	static std::shared_ptr<Predicate> numAssign();
	static std::shared_ptr<Predicate> numEqualAssign();
	static std::shared_ptr<Predicate> change();
	static std::shared_ptr<Predicate> numChange();
	static std::shared_ptr<Predicate> scaleUp();
	static std::shared_ptr<Predicate> scaleDown();
	static std::shared_ptr<Predicate> increase();
	static std::shared_ptr<Predicate> decrease();

	static std::shared_ptr<Predicate> gt();
	static std::shared_ptr<Predicate> lt();
	static std::shared_ptr<Predicate> eq();
	static std::shared_ptr<Predicate> ge();
	static std::shared_ptr<Predicate> le();

	static std::shared_ptr<Function> minus();
	static std::shared_ptr<Function> plus();
	static std::shared_ptr<Function> mult();
	static std::shared_ptr<Function> div();
	static std::shared_ptr<Function> neg();
	static std::shared_ptr<Function> totalCost();

	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> numericOps();
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> numericComparators();
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> assignmentOps();
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> functionalOps();
	static std::unordered_set<std::shared_ptr<Function>, FunctionHasher> numericFunctions();
};

} /* namespace pddl */

#endif /* H698080C8_7429_4832_85C5_BB609B20C23D */
