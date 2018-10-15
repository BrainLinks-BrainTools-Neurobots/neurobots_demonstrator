/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 28, 2017
 *      Author: kuhnerd
 * 	  Filename: scope.h
 */

#ifndef HFDAB9B71_A8FB_4572_AE36_E276D36357B7
#define HFDAB9B71_A8FB_4572_AE36_E276D36357B7
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/types.h>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pddl
{

class Term;
class TypedObject;
class FunctionTable;
class Type;
class Parameter;
class ParseContext;

/**
 * This class represents any PDDL object that can define variables
 or constants. It implements a lookup table to get the associated
 Parameters or TypedObjects given their name. Most methods of this
 class can take strings, TypedObjects or Terms as keys. For the
 latter two, the name of the objects will be used as a key.

 Scopes can be nested; when a symbol is not found in one scope, it
 will be looked up in it's parent Scope.

 Usually, a Scope object will not be used directly but be inherited
 by classes representing PDDL elements, e.g. domains, actions,
 quantified conditions/effects.
 */
class Scope: public std::enable_shared_from_this<Scope>
{

public:
	typedef std::function<void(std::unordered_map<std::shared_ptr<TypedObject>, std::shared_ptr<TypedObject>, TypedObjectHasher>& mapping,
			const std::vector<std::shared_ptr<TypedObject>>& params,
			std::shared_ptr<TypedObject>& next,
			std::shared_ptr<TypedObject>& nextval)> SmartInstantiateFunction;

	struct SmartInstantiater
	{
		struct LoopData
		{
			pddl::TypedObjectVector params;
			std::shared_ptr<TypedObject> next;
			std::shared_ptr<TypedObject> nextVal;
		};

		std::unordered_map<TypedObjectPtr, TypedObjectVector> values;
		std::unordered_map<TypedObjectPtr, TypedObjectPtr, TypedObjectHasher> mapping;
		std::list<std::pair<pddl::TypedObjectPtr, int>> stack;
		std::unordered_set<TypedObjectPtr, TypedObjectHasher> remaining;
		SmartInstantiateFunction func;
		std::vector<std::shared_ptr<Parameter> > args;
		std::vector<TypedObjectUnorderedSet> argList;
		std::shared_ptr<Scope> parent;
		std::shared_ptr<TypedObject> currentArg;
		int currentIndex;
		int i;
		LoopData loopData;
		bool continueLoop;
	};

	Scope(const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& constants,
			std::shared_ptr<Scope> parent);
	virtual ~Scope();

	virtual std::string str() const;

	void lookup(const std::vector<std::shared_ptr<Term> >& input,
			std::vector<std::shared_ptr<Term> >& output);

	void setParent(std::shared_ptr<Scope> parent);

	void add(std::shared_ptr<TypedObject> object,
			bool check = false);
	std::shared_ptr<TypedObject> get(const std::string& key);
	std::shared_ptr<TypedObject> get(const std::shared_ptr<Term>& key);
	bool contains(const std::string& key) const;

	std::shared_ptr<ParseContext> getParseContext();

	bool smartInstantiateBegin(const SmartInstantiateFunction& func,
			const std::vector<std::shared_ptr<Parameter>>& args,
			const std::vector<TypedObjectUnorderedSet>& argList,
			const std::shared_ptr<Scope>& parent);
	bool smartInstantiateNext(std::unordered_map<TypedObjectPtr, TypedObjectPtr, TypedObjectHasher>& result);

	virtual void instantiate(const std::unordered_map<pddl::ParameterPtr, pddl::TypedObjectPtr, pddl::TypedObjectHasher>& mapping,
			const std::shared_ptr<Scope>& parent = std::shared_ptr<Scope>());

	void uninstantiate();

private:
	bool smartInstantiateNext2();

public:
	std::shared_ptr<Scope> m_parent;
	std::shared_ptr<Scope> m_originalParent;
	std::shared_ptr<FunctionTable> m_functions;
	std::shared_ptr<FunctionTable> m_predicates;
	std::unordered_map<std::string, std::shared_ptr<Type>> m_types;
	std::unordered_set<std::string> m_requirements;

	std::unordered_map<std::string, std::shared_ptr<TypedObject>> m_contentTypedObjects;

protected:
	std::shared_ptr<ParseContext> m_parseContext;
	std::unordered_map<std::shared_ptr<TypedObject>, std::shared_ptr<Term>, TypedObjectHasher> m_termCache;

private:
	std::shared_ptr<SmartInstantiater> m_smartInstantiater;
};

} /* namespace pddl */

#endif /* HFDAB9B71_A8FB_4572_AE36_E276D36357B7 */
