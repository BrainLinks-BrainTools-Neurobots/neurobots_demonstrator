/*
 * problem.cpp
 *
 *  Created on: Oct 2, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>

namespace pddl
{

std::string InitLiteral::str() const
{
	return "Effect: " + Literal::str();
}

size_t InitLiteral::hash() const
{
	std::size_t h = 0;
	hash_combine(h, Literal::hash(), BaseElement::hash());

	return h;
}

InitLiteral::InitLiteral(std::shared_ptr<Predicate> predicate,
		const std::vector<std::shared_ptr<Term> >& args,
		std::shared_ptr<Scope> scope,
		bool negated) :
				Literal(predicate, args, scope, negated)
{
	BaseElement::m_scope = scope;
}

bool InitLiteral::operator ==(const InitLiteral& other) const
		{
	return Literal::operator ==(other) && BaseElement::operator ==(other);
}

bool InitLiteral::operator !=(const InitLiteral& other) const
		{
	return !(*this == other);
}

std::shared_ptr<InitLiteral> InitLiteral::parse(std::shared_ptr<Element> elem,
		std::shared_ptr<Scope> scope)
{
	Token first = elem->get(Element::ExpectedTypeNone, "effect specification")->token;

	elem->reset();
	std::shared_ptr<Literal> literal = Literal::parse(elem, scope, false, 999, SCOPE_INIT);

	if (literal->m_negated && (CONTAINS(literal->m_predicate, Builtin::assignmentOps())
			|| CONTAINS(literal->m_predicate, Builtin::numericOps())))
	{
		throw ParseError(first, "Can't negate fluent assignments.");
	}

	if (literal->m_predicate == Builtin::assign() || literal->m_predicate == Builtin::numAssign())
	{
		throw ParseError(first, "Can't use 'assing' in :init, please use '=' instead.");
	}

	return std::shared_ptr<InitLiteral>(new InitLiteral(literal->m_predicate, literal->m_args, scope, literal->m_negated));
}

std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> removeConstants(
		const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& objects,
		const std::shared_ptr<Domain> domain)
{
	std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> result;
	std::shared_ptr<Type> defaultObjectType = DefaultTypes::objectType();
	for (auto& o : objects)
		if (CONTAINS_NOT(o, domain->m_constants))
			result.insert(o);
	return result;
}

Problem::Problem(const std::string& name,
		const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& objects,
		const std::shared_ptr<Domain> domain) :
				Scope(removeConstants(objects, domain), domain),
				m_name(name),
				m_objects(objects)
{
}

Problem::Problem(const std::string& name,
		const std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& objects,
		const std::shared_ptr<Domain> domain,
		const std::unordered_set<std::shared_ptr<pddl::InitLiteral>, InitLiteralHasher>& init) :
				Scope(removeConstants(objects, domain), domain),
				m_name(name),
				m_objects(objects),
				m_init(init)
{
}

Problem::~Problem()
{
}

std::shared_ptr<Domain> Problem::getDomain()
{
	return std::static_pointer_cast<Domain>(m_parent);
}

void Problem::getAllObjects(const std::shared_ptr<Type> type,
		std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>& result)
{
	if (isInstanceSharedCast(type, funcType, FunctionType))
	{
		throw NotImplementedException(FILE_AND_LINE);
	}
	else
	{
		std::shared_ptr<Type> t = type;
		if (isInstanceSharedCast(t, proxyType, ProxyType))
		{
			t = proxyType->effectiveType();
		}

		if (CONTAINS_NOT(type, m_objectsByType))
		{
			auto& set = m_objectsByType[type];
			for (auto& o: m_objects)
			{
				if (o->isInstanceOf(type) && o != DefaultTypedObjects::unknownObject())
				{
					set.insert(o);
				}
			}
		}

		getDomain()->getAllObjects(type, result);
		auto& set = m_objectsByType[type];
		result.insert(set.begin(), set.end());
	}
}

void InitLiteral::collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res)
{
	Literal::collectFreeVars(res);
}

void InitLiteral::setScope(const std::shared_ptr<Scope>& scope)
{
	BaseElement::setScope(scope);
	Literal::setScope(scope);
}

std::shared_ptr<BaseElement> InitLiteral::copy()
{
	throw pddl::NotImplementedException(FILE_AND_LINE);
}

std::shared_ptr<Problem> Problem::copy()
{
	return std::shared_ptr<Problem>(new Problem(m_name, m_objects, getDomain(), m_init));
}

}

/* namespace pddl */

