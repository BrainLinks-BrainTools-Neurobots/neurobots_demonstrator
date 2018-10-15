/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: gui.cpp
 */

#include <neurobots_database_gui/gui.h>
#include <database_msgs/get_world.h>
#include <database_conversions/database_conversions.h>
#include <neurobots_database_gui/tree.h>
#include <qpainter.h>

namespace neurobots_database_gui
{

GUI::GUI() :
				m_tree(NULL)
{
	ros::NodeHandle n;
	m_subChange = n.subscribe("/neurobots_database/database_change", 10, &GUI::callbackChange, this);
	m_srvGetWorld = n.serviceClient<database_msgs::get_world>("/neurobots_database/get_world");

	getWorld();

	setStyleSheet("background-color:white;");
}

GUI::~GUI()
{
	if (m_tree != NULL)
		delete m_tree;
}

void GUI::callbackChange(const database_msgs::database_change::ConstPtr& msg)
{
	ROS_INFO("Got database change");
	getWorld();
	update();
}

void GUI::getWorld()
{
	QMutexLocker locker(&m_mutex);
	database_msgs::get_world srv;

	if (m_tree != NULL)
		delete m_tree;

	ros::Rate r(10);
	int i = 0;
	while (!m_srvGetWorld.call(srv))
	{
		r.sleep();
		if (i++ > 30)
			return;
	}

	std::unordered_map<std::string, Element*> elements;
	bool foundRoom = false;

	for (size_t i = 0; i < srv.response.world.objects.size(); ++i)
	{
		std::string& name = srv.response.world.object_names[i];
		Object obj;
		database_conversions::convertMsgToObject(srv.response.world.objects[i], obj);
		std::vector<std::string>& attributeNames = srv.response.world.objects[i].attribute_names;

		std::unordered_map<std::string, std::vector<std::string>> stringAttributes;
		std::unordered_map<std::string, bool> boolAttributes;
		std::string type;

//		ROS_INFO_STREAM("name: " << name);
		for (size_t j = 0; j < obj.size(); ++j)
		{
//			ROS_INFO_STREAM(attributeNames[j] << ": " << obj[attributeNames[j]][0].which());

			if (attributeNames[j] == "name" || attributeNames[j] == "locatable" || attributeNames[j] == "perceptible")
			{
				continue;
			}
			else if (attributeNames[j] == "type")
			{
				type = CONVERT_TO_STRING(obj[attributeNames[j]]);
				if (type == "room")
					foundRoom = true;
			}
			//bool
			else if (obj[attributeNames[j]][0].which() == 2)
			{
				boolAttributes[attributeNames[j]] = CONVERT_TO_BOOL(obj[attributeNames[j]]);
			}
			//string
			else if (obj[attributeNames[j]][0].which() == 3)
			{
				for (auto& it : obj[attributeNames[j]])
				{
					stringAttributes[attributeNames[j]].push_back(boost::get<std::string>(it));
				}
			}
		}
//		ROS_INFO(" ");

		elements[name] = new Element(type, name, stringAttributes, boolAttributes);
	}

	if (!foundRoom)
	{
		elements["root_room"] = new Element("room", "root_room", { }, { });
	}

	m_tree = new Tree(elements);
}

void GUI::paintEvent(QPaintEvent* event)
{
	QMutexLocker locker(&m_mutex);
	QSize s = size();
	QPainter painter;
	painter.begin(this);
	painter.setRenderHint(QPainter::Antialiasing);
	if (m_tree != NULL)
		m_painter.paint(m_tree, &painter, s);
	painter.end();
}

} /* namespace neurobots_database_gui */

