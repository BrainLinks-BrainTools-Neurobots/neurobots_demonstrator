/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 29, 2017
 *      Author: kuhnerd
 * 	  Filename: builtin.cpp
 */

#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/constants.h>

namespace pddl
{

std::vector<std::shared_ptr<Type>> Builtin::defaultTypes()
{
	static std::vector<std::shared_ptr<Type>> types { DefaultTypes::objectType(), DefaultTypes::booleanType(), DefaultTypes::anyType() };
	return types;
}

std::unordered_map<std::string, std::shared_ptr<Type>> Builtin::defaultTypesAsMap()
{
	static std::unordered_map<std::string, std::shared_ptr<Type>> types;
	if (types.empty())
	{
		std::vector<std::shared_ptr<Type>> t = defaultTypes();
		for (auto& it : t)
			types[it->getName()] = it;
	}
	return types;
}

//equals = Predicate("=", [Parameter("?o1", t_object), Parameter("?o2", t_object)], builtin=True, function_scope=SCOPE_CONDITION)
std::shared_ptr<Predicate> pddl::Builtin::equals()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?o1", DefaultTypes::objectType())),
			std::shared_ptr<Parameter>(new Parameter("?o2", DefaultTypes::objectType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("=", args, true, SCOPE_CONDITION));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::assign()
{
	static std::shared_ptr<Parameter> p(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::objectType()))));
	static std::shared_ptr<Predicate> predicate(new Predicate("assign", {
			p,
			std::shared_ptr<Parameter>(new Parameter("?v", std::shared_ptr<ProxyType>(new ProxyType(p))))
	}, true, SCOPE_EFFECT));
	return predicate;
}

//equal_assign = Predicate("=", [Parameter("?f", FunctionType(t_object)), Parameter("?v", t_object)], builtin=True, function_scope=SCOPE_INIT)
std::shared_ptr<Predicate> Builtin::equalAssign()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::objectType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::objectType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("=", args, true, SCOPE_INIT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::numAssign()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("change", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::numEqualAssign()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("=", args, true, SCOPE_INIT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::change()
{
	static std::shared_ptr<Parameter> p(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::objectType()))));
	static std::vector<std::shared_ptr<Parameter>> args {
			p,
			std::shared_ptr<Parameter>(new Parameter("?v", std::shared_ptr<ProxyType>(new ProxyType(p))))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("change", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::numChange()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("change", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::scaleUp()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("scale-up", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::scaleDown()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("scale-down", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::increase()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("increase", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::decrease()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?f", std::shared_ptr<FunctionType>(new FunctionType(DefaultTypes::numberType())))),
			std::shared_ptr<Parameter>(new Parameter("?v", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("decrease", args, true, SCOPE_EFFECT));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::gt()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate(">", args, true, SCOPE_CONDITION));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::lt()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("<", args, true, SCOPE_CONDITION));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::eq()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("=", args, true, SCOPE_CONDITION));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::ge()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate(">=", args, true, SCOPE_CONDITION));
	return predicate;
}

std::shared_ptr<Predicate> Builtin::le()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType()))
	};
	static std::shared_ptr<Predicate> predicate(new Predicate("<=", args, true, SCOPE_CONDITION));
	return predicate;
}

std::shared_ptr<Function> Builtin::minus()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType())),
	};
	static std::shared_ptr<Function> func(new Function("-", args, DefaultTypes::numberType(), true));
	return func;
}

std::shared_ptr<Function> Builtin::plus()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType())),
	};
	static std::shared_ptr<Function> func(new Function("+", args, DefaultTypes::numberType(), true));
	return func;
}

std::shared_ptr<Function> Builtin::mult()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType())),
	};
	static std::shared_ptr<Function> func(new Function("*", args, DefaultTypes::numberType(), true));
	return func;
}

std::shared_ptr<Function> Builtin::div()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n1", DefaultTypes::numberType())),
			std::shared_ptr<Parameter>(new Parameter("?n2", DefaultTypes::numberType())),
	};
	static std::shared_ptr<Function> func(new Function("/", args, DefaultTypes::numberType(), true));
	return func;
}

std::shared_ptr<Function> Builtin::neg()
{
	static std::vector<std::shared_ptr<Parameter>> args {
			std::shared_ptr<Parameter>(new Parameter("?n", DefaultTypes::numberType())),
	};
	static std::shared_ptr<Function> func(new Function("-", args, DefaultTypes::numberType(), true));
	return func;
}

std::shared_ptr<Function> Builtin::totalCost()
{
	static std::shared_ptr<Function> function(new Function("total-cost", { }, DefaultTypes::numberType(), true));
	return function;
}

std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> Builtin::numericOps()
{
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> preds { numAssign(), scaleUp(), scaleDown(), increase(), decrease() };
	return preds;
}

std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> Builtin::assignmentOps()
{
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> preds { assign(), numAssign(), equalAssign(), numEqualAssign(), change(),
			numChange() };
	return preds;
}

std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> Builtin::numericComparators()
{
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> preds { gt(), lt(), eq(), ge(), le() };
	return preds;
}

std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> Builtin::functionalOps()
{
	static std::unordered_set<std::shared_ptr<Predicate>, FunctionHasher> preds { equals(), eq(), assign(), numAssign(), numEqualAssign() };
	return preds;
}

std::unordered_set<std::shared_ptr<Function>, FunctionHasher> Builtin::numericFunctions()
{
	static std::unordered_set<std::shared_ptr<Function>, FunctionHasher> funcs {
			minus(), plus(), mult(), div(), neg()
	};
	return funcs;
}

} /* namespace pddl */

