/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 28, 2017
 *      Author: kuhnerd
 * 	  Filename: visitors.h
 */

#ifndef H8B0C30CE_1C64_44A9_94E7_5718136F3412
#define H8B0C30CE_1C64_44A9_94E7_5718136F3412
#include <goal_planner_gui/pddl/base_visitor.h>
#include <memory>
#include <string>

namespace pddl
{

template<class T, class ResultValue, class ResultValueHasher>
struct Visitors
{
	typedef std::function<void(std::shared_ptr<T>&,
			std::unordered_set<std::shared_ptr<ResultValue>, ResultValueHasher>&)> VisitorFunction;

	static void visit(std::shared_ptr<T>& elem,
			std::function<void(std::shared_ptr<T>&,
					std::unordered_set<std::shared_ptr<ResultValue>, ResultValueHasher>&)>& visitor,
			std::unordered_set<std::shared_ptr<ResultValue>, ResultValueHasher>& result)
	{
		if (elem)
		{
			elem->visit(visitor, result);
		}
	}
};

class TermPrintVisitor: public BaseVisitor
{
public:
	TermPrintVisitor(bool instantiated);
	virtual ~TermPrintVisitor();

	virtual boost::any operator()(boost::any value,
			const std::vector<boost::any>& result);

private:
	bool m_instantiated;
};

} /* namespace pddl */

#endif /* H8B0C30CE_1C64_44A9_94E7_5718136F3412 */
