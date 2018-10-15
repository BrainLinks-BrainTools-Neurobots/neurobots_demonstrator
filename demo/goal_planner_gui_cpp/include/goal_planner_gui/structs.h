/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: structs.h
 */

#ifndef H7AC54F8D_7378_4812_92AE_61C82893136A
#define H7AC54F8D_7378_4812_92AE_61C82893136A
#include <goal_planner_gui/pddl/functions.h>

namespace goal_planner_gui
{
struct GraphContent
{
	double H;
	pddl::FunctionPtr f;
	size_t i, j;
};
}

#endif /* H7AC54F8D_7378_4812_92AE_61C82893136A */
