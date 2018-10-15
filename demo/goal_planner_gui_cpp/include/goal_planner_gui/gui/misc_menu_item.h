/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: mist_menu_item.h
 */

#ifndef H6996ABBE_F981_4DF0_A88A_018CBAEDB3AD
#define H6996ABBE_F981_4DF0_A88A_018CBAEDB3AD

#include <goal_planner_gui/gui/goal_menu_item.h>

namespace goal_planner_gui
{

class MiscMenuItem: public GoalMenuItem
{
public:
	MiscMenuItem(const QString& text,
			const QString& image,
			const QString& action,
			bool more);
	virtual ~MiscMenuItem();

	virtual bool initial() const;
	virtual QString getAction() const;
	virtual bool more() const;

private:
	QString m_action;
	bool m_more;
};

} /* namespace goal_planner_gui */

#endif /* H6996ABBE_F981_4DF0_A88A_018CBAEDB3AD */
