/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: gui.h
 */

#ifndef H3B4F9EC8_EF95_4F73_8626_D5FD19C8D86E
#define H3B4F9EC8_EF95_4F73_8626_D5FD19C8D86E
#include <database_msgs/database_change.h>
#include <neurobots_database_gui/painter.h>
#include <qwidget.h>
#include <qmutex.h>
#include <ros/ros.h>

namespace neurobots_database_gui
{

class Tree;

class GUI: public QWidget
{
	Q_OBJECT

public:
	GUI();
	virtual ~GUI();

	void callbackChange(const database_msgs::database_change::ConstPtr& msg);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
	void getWorld();

private:
	QMutex m_mutex;
	ros::Subscriber m_subChange;
	ros::ServiceClient m_srvGetWorld;
	Tree* m_tree;
	Painter m_painter;
};

} /* namespace neurobots_database_gui */

#endif /* H3B4F9EC8_EF95_4F73_8626_D5FD19C8D86E */
