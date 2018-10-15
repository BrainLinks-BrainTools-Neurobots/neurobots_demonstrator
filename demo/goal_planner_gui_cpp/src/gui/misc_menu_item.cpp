/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: misc_menu_item.cpp
 */

#include <goal_planner_gui/gui/misc_menu_item.h>

namespace goal_planner_gui
{

MiscMenuItem::MiscMenuItem(const QString& text,
		const QString& image,
		const QString& action,
		bool more) :
				m_action(action),
				m_more(more)
{
	m_arg.push_back(new ImageTextItem(text, image));
}

MiscMenuItem::~MiscMenuItem()
{
}

bool MiscMenuItem::initial() const
{
	return true;
}

QString MiscMenuItem::getAction() const
{
	return m_action;
}

bool MiscMenuItem::more() const
{
	return m_more;
}

} /* namespace goal_planner_gui */
