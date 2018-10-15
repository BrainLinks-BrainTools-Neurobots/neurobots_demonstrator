/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: functions.h
 */

#ifndef H8F179EA7_3F26_4BA5_82E8_02ACA210098E
#define H8F179EA7_3F26_4BA5_82E8_02ACA210098E

#include <boost/algorithm/string/join.hpp>
#include <boost/any.hpp>
#include <goal_planner_gui/pddl/base_visitor.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/constants.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/types.h>
#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace pddl
{

class TypedObject;
class Parameter;
class Scope;
class Literal;

/**
 * A Function object represents any type of PDDL function or
 * predicate.
 */
class Function
{
public:
	/**
	 * Arguments:
	 * name -- the name of the function
	 * args -- list of Parameter objects of this function
	 * type -- Type object of this function
	 * builtin -- True if this function is a builtin PDDL/MAPL/whateverDDL function
	 */
	Function(const std::string& name,
			const std::vector<std::shared_ptr<Parameter>>& parameters,
			const std::shared_ptr<Type>& type,
			bool builtin = false,
			int function_scope = SCOPE_ALL);
	virtual ~Function();

	bool isModal() const;
	virtual bool operator==(const Function& rhs) const;
	virtual bool operator!=(const Function& rhs) const;
	virtual std::shared_ptr<Function> copy();
	template<class T>
	std::shared_ptr<T> getCopy()
	{
		return std::static_pointer_cast<T>(copy());
	}

	virtual size_t hash() const;
	virtual std::string str() const;
	const std::vector<std::shared_ptr<Parameter> >& getArgs() const;
	const std::vector<std::shared_ptr<Type> >& getArgTypes() const;
	const std::string& getName() const;
	std::shared_ptr<Type> getType() const;
	int getFunctionScope() const;
	bool isBuiltin() const;

	static std::shared_ptr<Function> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Type> type,
			std::unordered_map<std::string, std::shared_ptr<Type>>& types);

	static void parseArgumentList(std::shared_ptr<Element> it,
			std::unordered_map<std::string, std::shared_ptr<Type>> typeDict,
			std::vector<std::shared_ptr<Parameter>>& result,
			std::shared_ptr<Scope> parentScope = std::shared_ptr<Scope>(),
			std::vector<std::shared_ptr<Parameter>> previousParams = std::vector<std::shared_ptr<Parameter>>(),
			bool requiredType = false);

	static std::shared_ptr<Function> getFunction(const std::shared_ptr<Literal>& lit);
	static bool isFunctional(const std::shared_ptr<Literal>& lit);

protected:
	std::string m_name;
	std::vector<std::shared_ptr<Parameter>> m_args;
	std::shared_ptr<Type> m_type;
	const int m_arity;
	const int m_functionScope;
	const bool m_builtin;
	mutable size_t m_hash;
	mutable std::vector<std::shared_ptr<Type>> m_argTypes;
};

HASH_AND_COMPARISON_OPERATOR(Function);
STREAM_OPERATOR(Function)

} /* namespace pddl */

#endif /* H8F179EA7_3F26_4BA5_82E8_02ACA210098E */
