/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 19, 2017
 *      Author: kuhnerd
 * 	  Filename: proposition.h
 */

#ifndef HF2D4D85F_112D_4482_BB75_1CDBB47A3577
#define HF2D4D85F_112D_4482_BB75_1CDBB47A3577
#include <goal_planner_gui/relaxed_exploration/unary_op.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <goal_planner_gui/pddl/utils.h>
#include <vector>

namespace goal_planner_gui
{

class Proposition
{
public:
	Proposition(const pddl::FactPtr& fact,
			const std::vector<UnaryOpPtr>& children);
	virtual ~Proposition();

	virtual std::string str();
	void reset();
	void fire(std::vector<UnaryOpPtr>& ops);

	pddl::FactPtr m_fact;
	std::vector<UnaryOpPtr> m_children;
	double m_costs;
	UnaryOpPtr m_reachedBy;
	bool m_hasFired;
};

POINTER_DEF(Proposition);

} /* namespace goal_planner_gui */

#endif /* HF2D4D85F_112D_4482_BB75_1CDBB47A3577 */
