/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Feb 14, 2018
 *      Author: kuhnerd
 * 	  Filename: back_menu_item.h
 */

#ifndef H4053B1C2_ABA4_4220_98E7_6DC00CD3BFCE
#define H4053B1C2_ABA4_4220_98E7_6DC00CD3BFCE

#include <goal_planner_gui/gui/goal_menu_item.h>

namespace goal_planner_gui
{

class BackMenuItem: public GoalMenuItem
{
public:
	BackMenuItem(const QString& imagePath);
	virtual ~BackMenuItem();

	virtual bool initial() const;
	virtual QString getAction() const;
	virtual bool back() const;
};

} /* namespace goal_planner_gui */

#endif /* H4053B1C2_ABA4_4220_98E7_6DC00CD3BFCE */
