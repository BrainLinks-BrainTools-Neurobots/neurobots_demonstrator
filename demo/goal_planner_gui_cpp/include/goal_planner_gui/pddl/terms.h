/*
 * terms.h
 *
 *  Created on: Oct 3, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_TERMS_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_TERMS_H_
#include <boost/any.hpp>
#include <goal_planner_gui/pddl/constants.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <memory>
#include <string>

namespace pddl
{

namespace TermTypes
{
enum T
{
	ConstantTerm,
	FunctionTerm,
	VariableTerm,
	FunctionVariableTerm
};
}

class Type;
class TypedObject;
class Scope;
class BaseVisitor;
class Element;
class Function;
class Parameter;

/**
 * This is an abstract superclass for all terms in a PDDL
 * functional expression.
 */
class Term: public std::enable_shared_from_this<Term>
{
public:
	Term();
	virtual ~Term();

	virtual boost::any visit(BaseVisitor* fn);
	virtual std::string pddlString(bool instantiated = true);
	virtual std::shared_ptr<Type> getType() const = 0;
	virtual bool isInstanceOf(const std::shared_ptr<Type>& type) const;
	virtual bool operator==(const Term& rhs) const = 0;
	virtual bool operator!=(const Term& rhs) const;
	virtual std::string str() const = 0;
	virtual TermTypes::T t() const = 0;
	virtual size_t hash(bool hashAll = false) const = 0;
	virtual std::shared_ptr<Term> copy() = 0;

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	static std::shared_ptr<Term> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope,
			int maxNesting = 999,
			int functionScope = SCOPE_ALL);

protected:
	mutable size_t m_hash;
	mutable size_t m_hashAll;
};

/**
 * This class represents a constant term. It is always bound to a
 * TypedObject, which can be accessed by the "object" member.
 */
class ConstantTerm: public Term
{
public:
	ConstantTerm(std::shared_ptr<TypedObject> obj);
	virtual ~ConstantTerm();
	std::shared_ptr<TypedObject> getObj() const;
	std::shared_ptr<TypedObject> getInstance() const;
	virtual std::shared_ptr<Type> getType() const;
	virtual bool operator==(const Term& rhs) const;
	virtual std::string str() const;
	virtual size_t hash(bool hashAll = false) const;
	virtual std::shared_ptr<Term> copy();

	virtual inline TermTypes::T t() const
	{
		return TermTypes::ConstantTerm;
	}

protected:
	std::shared_ptr<TypedObject> m_obj;
};

/**
 * This class represents a function term. It is similar to a
 * Literal, but it can be nested inside other Terms or Literals.
 */
class FunctionTerm: public Term
{
public:
	FunctionTerm(std::shared_ptr<Function> function,
			const std::vector<std::shared_ptr<Term>>& args,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>());
	virtual ~FunctionTerm();

	std::shared_ptr<Function> getFunction() const;
	const std::vector<std::shared_ptr<Term>>& getArgs() const;
	std::shared_ptr<Scope> getScope() const;
	virtual std::shared_ptr<Term> copy();

	virtual boost::any visit(BaseVisitor* fn);
	virtual std::shared_ptr<Type> getType() const;
	virtual bool operator==(const Term& rhs) const;
	virtual std::string str() const;
	virtual size_t hash(bool hashAll = false) const;
	virtual inline TermTypes::T t() const
	{
		return TermTypes::FunctionTerm;
	}

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

	static std::shared_ptr<FunctionTerm> parse(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> scope,
			int maxNesting = 999,
			int functionScope = SCOPE_ALL);

protected:
	FunctionTerm();

protected:
	std::shared_ptr<Function> m_function;
	std::vector<std::shared_ptr<Term>> m_args;
	std::shared_ptr<Scope> m_scope;
};

/**
 * This class represents a variable term. It is always bound to a
 * Parameter object, which can be accessed by the "object" member.
 */
class VariableTerm: public Term
{
public:
	VariableTerm(std::shared_ptr<Parameter> parameter);
	virtual ~VariableTerm();
	std::shared_ptr<Parameter> getObject() const;
	bool isInstantiated() const;
	std::shared_ptr<TypedObject> getInstance() const;
	virtual std::shared_ptr<Term> copy();

	virtual std::shared_ptr<Type> getType() const;
	virtual bool operator==(const Term& rhs) const;
	virtual std::string str() const;
	virtual size_t hash(bool hashAll = false) const;
	virtual inline TermTypes::T t() const
	{
		return TermTypes::VariableTerm;
	}

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res);

protected:
	std::shared_ptr<Parameter> m_object;
};

/**
 * This class represents a variable term with a function type. It
 * is always bound to a Parameter object, which can be accessed by
 * the "object" member.
 *
 * If the parameter is instantiated with a FunctionTerm, it will
 * behave like a FunctionTerm.
 */
class FunctionVariableTerm: public Term
{
public:
	FunctionVariableTerm(std::shared_ptr<Parameter> parameter);
	virtual ~FunctionVariableTerm();

	virtual boost::any visit(BaseVisitor* fn);

	virtual std::shared_ptr<Type> getType() const;
	virtual bool operator==(const Term& rhs) const;
	virtual std::string str() const;
	virtual size_t hash(bool hashAll = false) const;
	virtual bool isInstantiated() const;
	virtual std::shared_ptr<Term> copy();

	virtual inline TermTypes::T t() const
	{
		return TermTypes::FunctionVariableTerm;
	}

protected:
	FunctionTerm* m_functionTerm;
	VariableTerm* m_variableTerm;
};

class TermGenerator
{
	static std::shared_ptr<Term> create(std::shared_ptr<ConstantTerm> term);
	static std::shared_ptr<Term> create(std::shared_ptr<VariableTerm> term);
	static std::shared_ptr<Term> create(std::shared_ptr<FunctionTerm> term);
	static std::shared_ptr<Term> create(std::shared_ptr<TypedObject> obj);
	static std::shared_ptr<Term> create(int number);
	static std::shared_ptr<Term> create(double number);
	static std::shared_ptr<Term> create(std::shared_ptr<Function> func,
			const std::vector<std::shared_ptr<ConstantTerm>>& args);
	static std::shared_ptr<Term> create(std::shared_ptr<Function> func,
			const std::vector<std::shared_ptr<VariableTerm>>& args);
	static std::shared_ptr<Term> create(std::shared_ptr<Function> func,
			const std::vector<std::shared_ptr<TypedObject>>& args);
	static std::shared_ptr<Term> create(std::shared_ptr<Function> func,
			const std::vector<int>& args);
	static std::shared_ptr<Term> create(std::shared_ptr<Function> func,
			const std::vector<double>& args);
};

HASH_AND_COMPARISON_OPERATOR(Term);
STREAM_OPERATOR(Term);

} /* namespace pddl */

std::ostream& operator<<(std::ostream& stream,
		const pddl::Term& token);

namespace std
{

template<>
class hash<pddl::TermTypes::T>
{
public:
	size_t operator()(const pddl::TermTypes::T& t) const
			{
		return std::hash<int>()((int)t);
	}
};

}

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_TERMS_H_ */
