/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Feb 14, 2018
 *      Author: kuhnerd
 * 	  Filename: back_menu_item.cpp
 */

#include <goal_planner_gui/gui/back_menu_item.h>

namespace goal_planner_gui
{

BackMenuItem::BackMenuItem(const QString& imagePath) :
				GoalMenuItem(std::shared_ptr<GoalSpec>(), 0)
{
	m_arg.push_back(new ImageTextItem(QString("back"), imagePath + "images/back.png"));
}

BackMenuItem::~BackMenuItem()
{
}

bool BackMenuItem::initial() const
{
	return false;
}

QString BackMenuItem::getAction() const
{
	return "back";
}

bool BackMenuItem::back() const
{
	return true;
}

} /* namespace goal_planner_gui */
