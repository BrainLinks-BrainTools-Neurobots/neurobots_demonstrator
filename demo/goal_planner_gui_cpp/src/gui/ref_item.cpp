/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: ref_item.cpp
 */

#include <goal_planner_gui/gui/ref_item.h>
#include <goal_planner_gui/pddl/utils.h>

namespace goal_planner_gui
{

RefItem::RefItem() :
				m_nameItem(NULL)
{
}

RefItem::RefItem(const RefItem& other) :
				m_nameItem(other.m_nameItem),
				m_args(other.m_args)
{
}

RefItem::~RefItem()
{
}

QVariant RefItem::getName() const
{
	return QVariant::fromValue(m_nameItem);
}

QList<QVariant> RefItem::getArguments() const
{
	return m_args;
}

} /* namespace goal_planner_gui */

