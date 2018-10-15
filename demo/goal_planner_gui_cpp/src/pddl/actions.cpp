/*
 * actions.cpp
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/builtin.h>
#include <goal_planner_gui/pddl/conditions/condition.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/effects/effect.h>
#include <goal_planner_gui/pddl/effects/simple_effect.h>
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/modules.h>
#include <goal_planner_gui/pddl/terms.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/visitors.h>

using pddl::Visitors;

namespace pddl
{

Action::Action(const std::string& name,
		const std::vector<std::shared_ptr<Parameter>>& args,
		std::shared_ptr<Condition> precondition,
		std::shared_ptr<Effect> effect,
		std::shared_ptr<Scope> domain) :
				Scope(std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher>(args.begin(), args.end()), domain),
				m_name(name),
				m_args(args),
				m_precondition(precondition),
				m_effect(effect),
				m_hash(0)
{
	for (auto& it : args)
		if (it->t() != TypedObjectTypes::Parameter)
			throw Exception("Only provide Parameter objects!");

	if (m_precondition)
		m_precondition->setScope(shared_from_this());
	if (m_effect)
		m_effect->setScope(shared_from_this());
}

Action::~Action()
{
}

std::shared_ptr<Action> Action::parse(std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	it->get(":action");
	std::string name = it->get()->token.string;
	auto next = it->get();

	std::vector<std::shared_ptr<Parameter>> params;
	if (next && next->token.string == ":parameters")
	{
		Function::parseArgumentList(it->get(Element::ExpectedTypeList, "parameters"), scope->m_types, params);
		next = it->get();
	}

	std::shared_ptr<Action> action(new Action(name, params, std::shared_ptr<Condition>(), std::shared_ptr<Effect>(), scope));
//	for (auto& it3: it->m_children ) {
//		LOG_INFO(it3->token.string);
//	}
	while (true)
	{
		if (next->token.string == ":precondition")
		{
			if (action->m_precondition)
				throw ParseError(next->token, "precondition already defined");
			action->m_precondition = Condition::parse(it->get(Element::ExpectedTypeList, "condition"), action);
		}
		else if (next->token.string == ":effect")
		{
			if (action->m_effect)
				throw ParseError(next->token, "effects already defined");
			action->m_effect = Effect::parse(it->get(Element::ExpectedTypeList, "effect"), action);
		}
		else
		{
			throw UnexpectedTokenError(next->token, "precondition or effect");
		}
		try
		{
			next = it->get();
		}
		catch (pddl::EndOfListException& e)
		{
			break;
		}
	}

	return action;
}

bool Action::operator ==(const Action& rhs) const
		{
	return hash() == rhs.hash();
}

bool Action::operator !=(const Action& rhs) const
		{
	return !(*this == rhs);
}

size_t Action::hash() const
{
	if (m_hash == 0)
	{
		pddl::hash_combine(m_hash, m_name);
		for (auto& it : m_args)
		{
			pddl::hash_combine(m_hash, it->hash());
		}
	}
	return m_hash;
}

std::string Action::str() const
{
	std::string res;
	for (auto& it : m_args)
	{
		res += it->str() + ", ";
	}
	if (!res.empty())
	{
		res.pop_back();
		res.pop_back();
	}
	return "Action: " + m_name + "(" + res + ")";
}

double Action::getTotalCost()
{
	TermPtr tct(new FunctionTerm(Builtin::totalCost(), { }));
	std::unordered_set<std::shared_ptr<Term>, pddl::TermHasher> res;
	getEffects(tct, res);

	if (res.empty())
	{
		return 1;
	}
	else
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
}

void Action::getEffects(const std::shared_ptr<Term>& term,
		std::unordered_set<std::shared_ptr<Term>, pddl::TermHasher>& res)
{
	pddl::Visitors<pddl::Effect, pddl::Term, pddl::TermHasher>::VisitorFunction subcondVisitor = [&term](pddl::EffectPtr& eff,
			std::unordered_set<std::shared_ptr<Term>, pddl::TermHasher>& result)
	{
		if (isInstanceSharedCast(eff, seff, SimpleEffect))
		{
			if (CONTAINS(seff->m_predicate, Builtin::numericOps()) || CONTAINS(seff->m_predicate, Builtin::assignmentOps()))
			{
				if (seff->m_args[0] == term)
				{
					result.insert(seff->m_args[1]);
				}
			}
		}
	};

	Visitors<pddl::Effect, pddl::Term, pddl::TermHasher>::visit(m_effect, subcondVisitor, res);
}

} /* namespace pddl */

