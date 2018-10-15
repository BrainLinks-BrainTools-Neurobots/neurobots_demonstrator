/*
 * base_element.cpp
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/pddl/base_element.h>
#include <goal_planner_gui/pddl/conditions/quantified_condition.h>
#include <goal_planner_gui/pddl/effects/conditional_effect.h>
#include <goal_planner_gui/pddl/effects/universal_effect.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/scope.h>
#include <string>

namespace pddl
{

//void collectFreeVars(std::shared_ptr<BaseElement>& elem,
//		const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& result)
//{
//	if (isInstanceSharedCast(elem, e, InitLiteral))
//	{
//		elem->visit(BaseElement::BaseElementVisitFunction(collectFreeVars), result);
//		throw pddl::NotImplementedException(FILE_AND_LINE);
//	}
//	else if(isInstanceSharedCast(elem, e, QuantifiedCondition))
//	{
//
//	}
//	else if(isInstanceSharedCast(elem, e, UniversalEffect))
//	{
//
//	}
//	else if(isInstanceSharedCast(elem, e, ConditionalEffect))
//	{
//
//	}
//}

BaseElement::BaseElement() :
				m_freeComputed(false),
				m_hash(0)
{
}

BaseElement::~BaseElement()
{
}

void BaseElement::getChildren(std::vector<std::shared_ptr<BaseElement> >& children) const
		{
	children.clear();
}

size_t BaseElement::hash() const
{
	std::vector<std::shared_ptr<BaseElement>> children;
	getChildren(children);
	std::size_t h = 0;
	hash_combine(h, typeid(this).name());
	for (auto& k : children)
		hash_combine(h, k->hash());

//	if (m_hash != 0 && m_hash != h) {
//		LOG_ERROR("Hash has changed!");
//	}
//
//	m_hash = h;

	return h;
}

size_t BaseElement::hashWithInstance() const
{
	std::vector<std::shared_ptr<BaseElement>> children;
	getChildren(children);
	std::size_t h = 0;
	hash_combine(h, typeid(this).name());
	for (auto& k : children)
		hash_combine(h, k->hashWithInstance());
	return h;
}

bool BaseElement::operator ==(const BaseElement& rhs) const
		{
	return hash() == rhs.hash();
}

bool BaseElement::operator !=(const BaseElement& rhs) const
		{
	return !(*this == rhs);
}

void BaseElement::setScope(const std::shared_ptr<Scope>& scope)
{
	m_scope = scope;
	std::vector<std::shared_ptr<BaseElement>> children;
	getChildren(children);
	for (auto& it : children)
		it->setScope(scope);
}

std::shared_ptr<Scope> BaseElement::getScope()
{
	return m_scope;
}

std::string BaseElement::str() const
{
	return "BaseElement";
}

bool BaseElement::compareToWithInstance(const BaseElement& rhs) const
		{
	return hashWithInstance() == rhs.hashWithInstance();
}

const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& BaseElement::free()
{
//	if (!m_freeComputed)
//	{
//		LOG_INFO(str());
	collectFreeVars(m_free);
//		LOG_INFO("Collect " << m_free.size());
//		m_freeComputed = true;
//	}
	return m_free;
}

} /* namespace pddl */

