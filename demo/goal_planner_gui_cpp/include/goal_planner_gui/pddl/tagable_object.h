/*
 * tagable_object.h
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_GOAL_PLANNER_GUI_CPP_INCLUDE_GOAL_PLANNER_GUI_TAGABLE_OBJECT_H_
#define DEMO_GOAL_PLANNER_GUI_CPP_INCLUDE_GOAL_PLANNER_GUI_TAGABLE_OBJECT_H_
#include <string>
#include <unordered_map>
#include <memory>
#include <unordered_set>

namespace pddl
{

class BaseElement;
class Element;
class Scope;

class TagableObject
{
public:
	TagableObject();
	virtual ~TagableObject();

	virtual std::string getTag() const;
	static std::string tag();

	static void getParents(std::unordered_set<std::string>& parents);
};

class TagableObjectRegistry
{
public:
	typedef std::function<std::shared_ptr<TagableObject>(std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action)> Parser;
	typedef std::function<std::string()> TagNameGetter;
	typedef std::unordered_map<std::string, Parser> Parsers;
	typedef std::unordered_map<std::string, TagNameGetter> TagNameGetters;

	static TagableObjectRegistry* getInstance();

	bool exists(const std::string& name);

	void reg(const std::string& name,
			const Parser parser,
			const TagNameGetter tagNameGetter);

	std::string tag(const std::string& name);

	void parse(const std::string& className,
			std::shared_ptr<Element> it,
			std::shared_ptr<Scope> action);

private:
	Parsers m_parsers;
	TagNameGetters m_tagNameGetters;
	static TagableObjectRegistry* s_reg;
};

} /* namespace pddl */

#endif /* DEMO_GOAL_PLANNER_GUI_CPP_INCLUDE_GOAL_PLANNER_GUI_TAGABLE_OBJECT_H_ */
