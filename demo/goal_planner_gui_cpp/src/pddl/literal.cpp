/*
 * literal.cpp
 *
 *  Created on: Oct 3, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/literal.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/predicates.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/function_table.h>

namespace pddl
{

Literal::Literal(const Literal& other) :
				m_predicate(other.m_predicate->getCopy<Predicate>()),
				m_scope(other.m_scope),
				m_negated(other.m_negated)
{
	for (auto& it : other.m_args)
	{
		m_args.push_back(it->copy());
	}
}

Literal::Literal(std::shared_ptr<Predicate> predicate,
		const std::vector<std::shared_ptr<Term>>& args,
		std::shared_ptr<Scope> scope,
		bool negated) :
				m_predicate(predicate),
				m_scope(scope),
				m_negated(negated)
{
	if (scope)
	{
		m_scope->lookup(args, m_args);
	}
	else
	{
		m_args = args;
	}
}

Literal::~Literal()
{
}

std::string Literal::str() const
{
	std::string s = "(" + m_predicate->getName() + " ";
	for (auto& it : m_args)
		s += it->str() + " ";
	s += ")";
	if (m_negated)
		s = "(not " + s + ")";
	return s;
}

bool Literal::operator ==(const Literal& other) const
		{
	if (m_args.size() != other.m_args.size())
		return false;

	for (size_t i = 0; i < m_args.size(); ++i)
		if (m_args[i] != other.m_args[i])
			return false;

	return m_predicate == other.m_predicate and m_negated == other.m_negated;
}

bool Literal::operator !=(const Literal& other) const
		{
	return (*this != other);
}

size_t Literal::hash() const
{
	std::size_t h = 0;
	hash_combine(h, m_predicate->hash(), m_negated);
	for (auto& k : m_args)
		hash_combine(h, k->hash());
	return h;
}

size_t Literal::hashWithInstance() const
{
	std::size_t h = 0;
	hash_combine(h, m_predicate->hash(), m_negated);
	for (auto& k : m_args)
		hash_combine(h, k->hash(true));
	return h;
}

std::shared_ptr<Literal> Literal::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope,
		bool negate,
		int maxNesting,
		int functionScope)
{
	Token first = it->get(Element::ExpectedTypeTerminal, "predicate")->token;

	if (first.string == "not")
	{
		std::shared_ptr<Element> j = it->get(Element::ExpectedTypeList, "literal");
		it->checkNoMoreTokens();
		return Literal::parse(j, scope, !negate, maxNesting);
	}

	if (!scope->m_predicates->contains(first.string))
	{
		throw ParseError(first, "Unknown predicate: " + first.string);
	}

	std::vector<std::shared_ptr<Term>> args;

	while (true)
	{
		try
		{
			args.push_back(Term::parse(it, scope, maxNesting, functionScope));
		}
		catch (EndOfListException& e)
		{
			break;
		}
	}

	std::shared_ptr<Function> func = scope->m_predicates->get(first.string, args, functionScope);
	if (!func)
	{
		std::string typeStr;
		for (auto& a : args)
			typeStr += a->getType()->str() + " ";
		const auto& candidates = scope->m_predicates->operator [](first.string);
		std::string cStr;
		for (auto& p : candidates)
			if (p->getFunctionScope() & functionScope)
				cStr += "\t* " + p->str() + "\n";

		throw ParseError(first,
				"no matching function or predicate found for (" + first.string + " " + typeStr + "). Candidates are:\n " + cStr);
	}

	std::shared_ptr<Predicate> predicate = std::static_pointer_cast<Predicate>(func);

	//check type constraints for assignments
	const auto& assignementOps = Builtin::assignmentOps();
	if (CONTAINS(predicate, assignementOps))
	{
		const auto& term = std::static_pointer_cast<FunctionTerm>(args[0]);
		staticSharedPointerCast(term->getType(), functionType, FunctionType);
		auto& value = args[1];
//		LOG_INFO(value->getType() << " " << functionType->getType());
		if (!value->getType()->equalOrSubtypeOf(functionType->getType()))
		{
			throw ParseError(first, "Can't assign object of type " + value->getType()->str() + " to " + term->getFunction()->getName() + ".");
		}
	}

	//check nesting constraints
	if (maxNesting <= 0)
	{
		throw("Please check code! Predicate arguments are of type TypedObject and not of type Term (py code)");
//		auto& pargs = predicate->getArgs();
//		if (args.size() != pargs.size())
//		{
//			throw Exception("args.size() != pargs.size()");
//		}
//
//		for (size_t i = 0; i < args.size(); ++i)
//		{
//			auto& arg = args[i];
//			auto& parg = pargs[i];
//			if (arg->t() == TermTypes::FunctionTerm && parg->t() != TermTypes::FunctionTerm)
//				throw ParseError(first,
//						"Error in Argument " + std::to_string(i + 1)
//								+ ": Maximum nesting depth for functions exceeded or no functions allowed here.");
//		}
	}

	return std::shared_ptr<Literal>(new Literal(predicate, args, scope, negate));
}

bool Literal::isFunctional(const std::shared_ptr<Literal>& lit)
{
	static const pddl::FunctionUnorderedSet f { Builtin::equals(), Builtin::eq(), Builtin::assign(), Builtin::numAssign(), Builtin::equalAssign(),
			Builtin::numEqualAssign() };
	return CONTAINS(lit->m_predicate, f);
}

std::shared_ptr<Function> Literal::getFunction(const std::shared_ptr<Literal>& lit)
{
	if (isFunctional(lit))
	{
		if (isInstanceSharedCast(lit->m_args[0], fterm, FunctionTerm))
		{
			return fterm->getFunction();
		}
	}
	return lit->m_predicate;
}

void Literal::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	for (auto& t : m_args)
	{
		t->collectFreeVars(res);
	}
}

void Literal::setScope(const std::shared_ptr<Scope>& scope)
{
	m_scope = scope;
	if (m_scope)
	{
		std::vector<std::shared_ptr<Term>> args;
		m_scope->lookup(m_args, args);
		m_args = args;
	}
}

void Literal::getLiteralElements(const std::shared_ptr<Literal>& lit,
		std::shared_ptr<Function>& function,
		std::vector<std::shared_ptr<Term>>& litargs,
		std::shared_ptr<Function>& modality,
		std::vector<std::shared_ptr<Term>>& modalArgs,
		std::shared_ptr<Term>& value,
		bool& negated)
{
	function.reset();
	modality.reset();
	value.reset();
	negated = false;
	litargs.clear();
	modalArgs.clear();

	if (isFunctional(lit))
	{
		auto funcTerm = std::static_pointer_cast<pddl::FunctionTerm>(lit->m_args[0]);
		function = funcTerm->getFunction();
		litargs = funcTerm->getArgs();
		value = lit->m_args[1];
		negated = lit->m_negated;
	}
	else
	{
		auto& pargs = lit->m_predicate->getArgs();
		ASSERT(lit->m_args.size() == pargs.size());
		for (size_t i = 0; i < lit->m_args.size(); ++i)
		{
			auto& arg = lit->m_args[i];
			auto& parg = pargs[i];
			if (isInstanceSharedCast(parg, farg, FunctionTerm))
			{
				if (!function)
				{
					function = farg->getFunction();
					litargs = farg->getArgs();
				}
				else
				{
					throw Exception("A modal svar can only contain one function");
				}
			}
			else
			{
				modalArgs.push_back(arg);
			}
		}

		if (!function)
		{
			function = lit->m_predicate;
			litargs = lit->m_args;
			modalArgs.clear();
		}
		else
		{
			modality = lit->m_predicate;
		}

		value.reset(new ConstantTerm(lit->m_negated ? DefaultTypedObjects::falseObject() : DefaultTypedObjects::trueObject()));
	}
}

} /* namespace pddl */

