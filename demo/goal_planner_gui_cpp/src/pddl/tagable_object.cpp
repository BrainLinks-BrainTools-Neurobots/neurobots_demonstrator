/*
 * tagable_object.cpp
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */
#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/tagable_object.h>
#include <goal_planner_gui/pddl/utils.h>

namespace pddl
{

TagableObjectRegistry* TagableObjectRegistry::s_reg = 0;

TagableObject::TagableObject()
{
}

TagableObject::~TagableObject()
{
}

void TagableObjectRegistry::reg(const std::string& name,
		const Parser parser,
		const TagNameGetter tagNameGetter)
{
	m_parsers[name] = parser;
	m_tagNameGetters[name] = tagNameGetter;
}

void TagableObjectRegistry::parse(const std::string& className,
		std::shared_ptr<Element> it,
		std::shared_ptr<Scope> scope)
{
	if (CONTAINS(className, m_parsers))
	{
		m_parsers[className](it, scope);
	}
	else
	{
		throw Exception("Please register the class " + className);
	}
}

TagableObjectRegistry* TagableObjectRegistry::getInstance()
{
	if (s_reg == NULL)
		s_reg = new TagableObjectRegistry;
	return s_reg;
}

std::string TagableObject::getTag() const
{
	return tag();
}

std::string TagableObject::tag()
{
	throw NotImplementedException(FILE_AND_LINE);
}

void TagableObject::getParents(std::unordered_set<std::string>& parents)
{
	//do nothing
}

bool TagableObjectRegistry::exists(const std::string& name)
{
	return CONTAINS(name, m_parsers);
}

std::string TagableObjectRegistry::tag(const std::string& name)
{
	if (CONTAINS(name, m_tagNameGetters))
		return m_tagNameGetters[name]();
	else
		throw Exception("Please register the class " + name);
}

} /* namespace pddl */

