/*
 * terms.cpp
 *
 *  Created on: Oct 3, 2017
 *      Author: kuhnerd
 */

#include <boost/algorithm/string/trim.hpp>
#include <boost/any.hpp>
#include <goal_planner_gui/pddl/base_visitor.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/visitors.h>

namespace pddl
{

Term::Term() :
				m_hash(0),
				m_hashAll(0)
{
}

Term::~Term()
{
}

boost::any Term::visit(BaseVisitor* fn)
{
	std::vector<boost::any> l;
	return (*fn)(boost::any(shared_from_this()), l);
}

std::string Term::pddlString(bool instantiated)
{
	TermPrintVisitor* visitor = new TermPrintVisitor(instantiated);
	std::string result = boost::any_cast<std::string>(visit(visitor));
	delete visitor;
	return result;
}

bool Term::isInstanceOf(const std::shared_ptr<Type>& type) const
		{
	return getType()->equalOrSubtypeOf(type);
}

bool Term::operator !=(const Term& rhs) const
		{
	return !(*this == rhs);
}

std::shared_ptr<Term> Term::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope,
		int maxNesting,
		int functionScope)
{
	std::shared_ptr<Element> term = it->get(Element::ExpectedTypeNone, "function term, variable or constant");
	if (term->isTerminal())
	{
		std::shared_ptr<TypedObject> obj;

//		LOG_INFO(it->token.string)
//		for (auto& it: scope->m_contentTypedObjects)
//			LOG_INFO(" " << it.first);

		if (scope->contains(term->token.string))
		{
			obj = scope->get(term->token.string);
		}
		else
		{
			try
			{
				double value = std::stod(term->token.string);
				obj.reset(new TypedNumber(value));
			}
			catch (std::invalid_argument& e)
			{
				throw ParseError(term->token, "Unknown identifier: '" + term->token.string + "'");
			}
		}

		if (obj->t() == TypedObjectTypes::Parameter)
		{
			if (obj->getType()->t() == Types::FunctionType)
				return std::shared_ptr<FunctionVariableTerm>(new FunctionVariableTerm(std::static_pointer_cast<Parameter>(obj)));
			return std::shared_ptr<VariableTerm>(new VariableTerm(std::static_pointer_cast<Parameter>(obj)));
		}
		return std::shared_ptr<ConstantTerm>(new ConstantTerm(obj));
	}

	return FunctionTerm::parse(term, scope, maxNesting - 1, functionScope);
}

void Term::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
}

ConstantTerm::ConstantTerm(std::shared_ptr<TypedObject> obj) :
				m_obj(obj)
{
}

ConstantTerm::~ConstantTerm()
{
}

std::shared_ptr<TypedObject> ConstantTerm::getObj() const
{
	return m_obj;
}

std::shared_ptr<TypedObject> ConstantTerm::getInstance() const
{
	return m_obj;
}

std::shared_ptr<Type> ConstantTerm::getType() const
{
	return m_obj->getType();
}

bool ConstantTerm::operator ==(const Term& rhs) const
		{
	return hash() == rhs.hash();
}

std::string ConstantTerm::str() const
{
	return "ConstantTerm " + m_obj->getName();
}

size_t ConstantTerm::hash(bool hashAll) const
		{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_obj->hash());
		m_hash = h;
	}
	return m_hash;
}

std::shared_ptr<Term> ConstantTerm::copy()
{
	return std::shared_ptr<Term>(new ConstantTerm(m_obj->copy()));
}

FunctionTerm::FunctionTerm(std::shared_ptr<Function> function,
		const std::vector<std::shared_ptr<Term> >& args,
		std::shared_ptr<Scope> scope) :
				m_function(function)
{
	if (scope)
		scope->lookup(args, m_args);
	else
		m_args = args;
}

std::shared_ptr<FunctionTerm> FunctionTerm::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope,
		int maxNesting,
		int functionScope)
{
	std::shared_ptr<Element> name = it->get(Element::ExpectedTypeNone, "function or predicate identifier");

	std::shared_ptr<FunctionTable> table;

	if (scope->m_functions->contains(name->token.string))
		table = scope->m_functions;
	else if (scope->m_predicates->contains(name->token.string))
		table = scope->m_predicates;
	else
		throw ParseError(name->token, "Unknown function or predicate: '" + name->token.string + "'");

	std::vector<std::shared_ptr<Term>> args;
	int i = 0;

	while (true)
	{
		try
		{
			std::shared_ptr<Term> term = Term::parse(it, scope, maxNesting);
			if (maxNesting <= 0 && term->t() == TermTypes::FunctionTerm)
				throw ParseError(name->token,
						"Error in Argument " + std::to_string(i + 1)
								+ ": Maximum nesting depth for functions exceeded or no functions allowed here.");
			args.push_back(term);
			++i;
		}
		catch (EndOfListException& e)
		{
			break;
		}
	}

	std::shared_ptr<Function> func = table->get(name->token.string, args, functionScope);
	if (!func)
	{
		std::string typeStr;
		for (auto& a : args)
			typeStr += a->getType()->str() + " ";
		const auto& candidates = table->operator [](name->token.string);
		std::string cStr;
		for (auto& p : candidates)
			if (p->getFunctionScope() & functionScope)
				cStr += p->str() + " ";

		throw ParseError(name->token,
				"no matching function or predicate found for (" + name->token.string + " " + typeStr + "). Candidates are:\n " + cStr);
	}

	if (func->isModal())
		throw ParseError(name->token, "nested modal predicates are not allowed.");

	return std::shared_ptr<FunctionTerm>(new FunctionTerm(func, args));
}

size_t FunctionTerm::hash(bool hashAll) const
		{
	if (hashAll)
	{
		if (m_hashAll == 0)
		{
			std::size_t h = 0;
			for (auto& k : m_args)
				hash_combine(h, k->hash(true));

			hash_combine(h, m_function->hash());
			m_hashAll = h;
		}
		return m_hashAll;
	}
	else
	{
		if (m_hash)
		{
			std::size_t h = 0;
			for (auto& k : m_args)
				hash_combine(h, k->hash(false));

			hash_combine(h, m_function->hash());
			m_hash = h;
		}
		return m_hash;
	}
}

FunctionTerm::FunctionTerm()
{
}

FunctionTerm::~FunctionTerm()
{
}

std::string FunctionTerm::str() const
{
	std::string args;
	for (auto& k : m_args)
		args += k->str() + " ";
	boost::algorithm::trim(args);
	return "FunctionTerm: " + m_function->getName() + "(" + args + ")";
}

boost::any FunctionTerm::visit(BaseVisitor* fn)
{
	std::vector<boost::any> l;
	for (auto& it : m_args)
		l.push_back(it->visit(fn));
	return (*fn)(boost::any(shared_from_this()), l);
}

std::shared_ptr<Type> FunctionTerm::getType() const
{
	return std::shared_ptr<Type>(new FunctionType(m_function->getType()));
}

bool FunctionTerm::operator ==(const Term& rhs) const
		{
	if (t() == rhs.t())
	{
		staticPointerCast(&rhs, other, FunctionTerm);

		if (m_function != other->m_function)
			return false;

		auto& otherArgs = other->getArgs();
		if (m_function == other->getFunction() && m_args.size() == otherArgs.size())
			for (size_t i = 0; i < m_args.size(); ++i)
				if (m_args[i] != otherArgs[i])
					return false;
		return true;
	}
	else
	{
		return false;
	}
}

std::shared_ptr<Function> FunctionTerm::getFunction() const
{
	return m_function;
}

void FunctionTerm::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	for (auto& it : m_args)
	{
		it->collectFreeVars(res);
	}
}

const std::vector<std::shared_ptr<Term> >& FunctionTerm::getArgs() const
{
	return m_args;
}

std::shared_ptr<Scope> FunctionTerm::getScope() const
{
	return m_scope;
}

std::shared_ptr<Term> FunctionTerm::copy()
{
	std::vector<std::shared_ptr<Term>> args;

	for (auto& it : m_args)
	{
		args.push_back(it->copy());
	}

	return std::shared_ptr<Term>(new FunctionTerm(m_function->copy(), args, m_scope));
}

VariableTerm::VariableTerm(std::shared_ptr<Parameter> parameter) :
				m_object(parameter)
{
}

VariableTerm::~VariableTerm()
{
}

std::shared_ptr<Parameter> VariableTerm::getObject() const
{
	return m_object;
}

std::shared_ptr<TypedObject> VariableTerm::getInstance() const
{
	if (m_object->isInstantiated())
		return m_object->getInstance();
	throw Exception("Term" + m_object->str() + " is not instantiated!");
}

bool VariableTerm::isInstantiated() const
{
	return m_object->isInstantiated();
}

std::shared_ptr<Type> VariableTerm::getType() const
{
	return m_object->getType();
}

bool VariableTerm::operator ==(const Term& rhs) const
		{
	if (isInstanceCast(&rhs, other, VariableTerm))
		return m_object == other->m_object;
//	else if (isInstanceCast(&rhs, other, Parameter))
//		return m_parameter == *other;
//	throw Exception("Check again");
	return false;
}

std::string VariableTerm::str() const
{
	return "VariableTerm: " + m_object->str();
//	if (isInstantiated())
//		return "VariableTerm: " + m_object->str() + " (instance: " + m_object->getInstance()->str() + ")";
//	else
//		return "VariableTerm: " + m_object->str() + " (no instance)";
}

size_t VariableTerm::hash(bool hashAll) const
		{
	if (hashAll)
	{
		if (m_hashAll == 0)
		{
			std::size_t h = 0;
			hash_combine(h, t(), m_object->hash());

			auto& instance = m_object->getInstance();
			hash_combine(h, instance ? instance->hash() : 0);
			m_hashAll = h;
		}
		return m_hashAll;
	}
	else
	{
		if (m_hashAll == 0)
		{
			std::size_t h = 0;
			hash_combine(h, t(), m_object->hash());
			m_hash = h;
		}
		return m_hash;
	}
}

std::shared_ptr<Term> VariableTerm::copy()
{
	return std::shared_ptr<Term>(new VariableTerm(m_object->getCopy<Parameter>()));
}

void VariableTerm::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	res.insert(m_object);
}

FunctionVariableTerm::FunctionVariableTerm(std::shared_ptr<Parameter> parameter)
{
	throw NotImplementedException(FILE_AND_LINE);
}

FunctionVariableTerm::~FunctionVariableTerm()
{
	delete m_functionTerm;
	delete m_variableTerm;
}

boost::any FunctionVariableTerm::visit(BaseVisitor* fn)
{
	throw NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Type> FunctionVariableTerm::getType() const
{
	throw NotImplementedException(FILE_AND_LINE);
}

bool FunctionVariableTerm::operator ==(const Term& rhs) const
		{
	throw NotImplementedException(FILE_AND_LINE);
}

std::string FunctionVariableTerm::str() const
{
	throw NotImplementedException(FILE_AND_LINE);
}

size_t FunctionVariableTerm::hash(bool hashAll) const
		{
	throw NotImplementedException(FILE_AND_LINE);
}

bool FunctionVariableTerm::isInstantiated() const
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Term> FunctionVariableTerm::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<ConstantTerm> term)
{
	return create(term->getObj());
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<VariableTerm> term)
{
	return create(term->getObject());
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<FunctionTerm> term)
{
	return std::shared_ptr<Term>(new FunctionTerm(term->getFunction(), term->getArgs()));
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<TypedObject> obj)
{
	if (obj->t() == TypedObjectTypes::Parameter)
		if (obj->getType()->t() == Types::FunctionType)
			return std::shared_ptr<Term>(new FunctionVariableTerm(std::dynamic_pointer_cast<Parameter>(obj)));
		else
			return std::shared_ptr<Term>(new VariableTerm(std::dynamic_pointer_cast<Parameter>(obj)));
	else
		return std::shared_ptr<Term>(new ConstantTerm(obj));
}

std::shared_ptr<Term> TermGenerator::create(int number)
{
	return std::shared_ptr<Term>(new ConstantTerm(std::shared_ptr<TypedNumber>(new TypedNumber(number))));
}

std::shared_ptr<Term> TermGenerator::create(double number)
{
	return std::shared_ptr<Term>(new ConstantTerm(std::shared_ptr<TypedNumber>(new TypedNumber(number))));
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<Function> func,
		const std::vector<std::shared_ptr<ConstantTerm>>& args)
{
	std::vector<std::shared_ptr<Term>> t;
	for (auto& it : args)
		t.push_back(create(it));
	return std::shared_ptr<Term>(new FunctionTerm(func, t));
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<Function> func,
		const std::vector<std::shared_ptr<VariableTerm>>& args)
{
	std::vector<std::shared_ptr<Term>> t;
	for (auto& it : args)
		t.push_back(create(it));
	return std::shared_ptr<Term>(new FunctionTerm(func, t));
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<Function> func,
		const std::vector<std::shared_ptr<TypedObject>>& args)
{
	std::vector<std::shared_ptr<Term>> t;
	for (auto& it : args)
		t.push_back(create(it));
	return std::shared_ptr<Term>(new FunctionTerm(func, t));
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<Function> func,
		const std::vector<int>& args)
{
	std::vector<std::shared_ptr<Term>> t;
	for (auto& it : args)
		t.push_back(create(it));
	return std::shared_ptr<Term>(new FunctionTerm(func, t));
}

std::shared_ptr<Term> TermGenerator::create(std::shared_ptr<Function> func,
		const std::vector<double>& args)
{
	std::vector<std::shared_ptr<Term>> t;
	for (auto& it : args)
		t.push_back(create(it));
	return std::shared_ptr<Term>(new FunctionTerm(func, t));
}

} /* namespace pddl */

std::ostream& operator <<(std::ostream& stream,
		const pddl::Term& token)
{
	stream << token.str();
	return stream;
}

