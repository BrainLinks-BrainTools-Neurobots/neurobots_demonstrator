/*
 * base_element.h
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_BASE_ELEMENT_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_BASE_ELEMENT_H_
#include <goal_planner_gui/pddl/tagable_object.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/pddl/types.h>
#include <memory>
#include <vector>

namespace pddl
{

class Scope;

class BaseElement: public TagableObject, public std::enable_shared_from_this<BaseElement>
{
public:
//	typedef std::function<void(std::shared_ptr<BaseElement>&,
//			std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>&)> BaseElementVisitFunction;

	BaseElement();
	virtual ~BaseElement();

	virtual void getChildren(std::vector<std::shared_ptr<BaseElement>>& children) const;
	template<class ReturnValue>
	void getChildrenAs(std::vector<std::shared_ptr<ReturnValue>>& children) const;
	virtual size_t hash() const;
	virtual size_t hashWithInstance() const;
	virtual std::string str() const;
	virtual bool compareToWithInstance(const BaseElement& rhs) const;
	virtual bool operator==(const BaseElement& rhs) const;
	virtual bool operator!=(const BaseElement& rhs) const;
	virtual std::shared_ptr<BaseElement> copy() = 0;

	virtual void setScope(const std::shared_ptr<Scope>& scope);
	std::shared_ptr<Scope> getScope();

	const std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& free();

	static void getParents(std::unordered_set<std::string>& parents)
	{
		parents.insert("TagableObject");
		TagableObject::getParents(parents);
	}

	virtual void collectFreeVars(std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher>& res) = 0;

	template<class T, class ResultValue, class ResultValueHasher>
	void visit(std::function<void(std::shared_ptr<T>&,
			std::unordered_set<std::shared_ptr<ResultValue>, ResultValueHasher>&)>& fn,
			std::unordered_set<std::shared_ptr<ResultValue>, ResultValueHasher>& result)
	{
		std::vector<std::shared_ptr<T>> children;
		getChildrenAs<T>(children);
		for (auto& it : children)
		{
			fn(it, result);
			it->visit(fn, result);
		}
		auto t = std::static_pointer_cast<T>(shared_from_this());
		fn(t, result);
	}

	template<class T>
	std::shared_ptr<T> getCopy()
	{
		return std::static_pointer_cast<T>(copy());
	}

protected:
	std::shared_ptr<Scope> m_scope;
	std::unordered_set<std::shared_ptr<Parameter>, TypedObjectHasher> m_free;
	bool m_freeComputed;
	mutable size_t m_hash;
};

HASH_AND_COMPARISON_OPERATOR(BaseElement);
//HASH_AND_COMPARISON_STRUCT(BaseElement, hashWithInstance, compareToWithInstance, BaseElementWithInstance);
class BaseElementWithInstanceHasher
{
public:\

	std::size_t operator()(const std::shared_ptr<BaseElement>& obj) const noexcept\

	{\

		if (!obj)
			return 0;\

		else
			return obj->hashWithInstance();\

	}\

};\

class BaseElementWithInstanceEqualTo
{
public:\

	std::size_t operator()(const std::shared_ptr<BaseElement>& obj1,
			const std::shared_ptr<BaseElement>& obj2) const noexcept\

			{\

		return (!obj1 && !obj2) || (obj1 && obj2 && obj1->compareToWithInstance(*obj2));\

	}\

};\

typedef std::unordered_set<std::shared_ptr<BaseElement>, BaseElementWithInstanceHasher, BaseElementWithInstanceEqualTo> BaseElementWithInstanceUnorderedSet;\

} /* namespace pddl */

template<class ReturnValue>
inline void pddl::BaseElement::getChildrenAs(std::vector<std::shared_ptr<ReturnValue> >& children) const
		{
	std::vector<std::shared_ptr<BaseElement>> res;
	getChildren(res);
	children.reserve(res.size());
	for (auto& it : res)
	{
		children.push_back(std::static_pointer_cast<ReturnValue>(it));
	}
}

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_BASE_ELEMENT_H_ */
