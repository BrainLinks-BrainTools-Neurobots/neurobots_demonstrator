/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: goal_menu_item.h
 */

#ifndef H0E07CBC5_0F61_441B_8EE3_A6D7E85F1444
#define H0E07CBC5_0F61_441B_8EE3_A6D7E85F1444

#include <goal_planner_gui/gui/image_text_item.h>
#include <QObject>
#include <qvariant.h>
#include <memory>

namespace goal_planner_gui
{

class GoalSpec;

class GoalMenuItem: public QObject
{
Q_OBJECT

public:
	GoalMenuItem();
	GoalMenuItem(const std::shared_ptr<GoalSpec>& goal,
			int currentIndex);
	GoalMenuItem(const GoalMenuItem& other);
	virtual ~GoalMenuItem();

	Q_PROPERTY(bool initial READ initial CONSTANT)
	Q_PROPERTY(QString action READ getAction CONSTANT)
	Q_PROPERTY(QVariant argument READ getArgument CONSTANT)
	Q_PROPERTY(bool more READ more CONSTANT)
	Q_PROPERTY(bool back READ back CONSTANT)

	virtual QString getAction() const;
	virtual QVariant getArgument() const;
	virtual bool initial() const;
	virtual bool more() const;
	virtual bool back() const;

protected:
	QList<QObject*> m_arg;

	std::shared_ptr<GoalSpec> m_goal;
};

} /* namespace goal_planner_gui */

Q_DECLARE_METATYPE(goal_planner_gui::GoalMenuItem);

#endif /* H0E07CBC5_0F61_441B_8EE3_A6D7E85F1444 */
