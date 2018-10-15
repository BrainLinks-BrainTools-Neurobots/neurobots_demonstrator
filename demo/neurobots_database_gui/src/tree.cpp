/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: tree.cpp
 */

#include <neurobots_database_gui/tree.h>
#include <ros/ros.h>

#define FIND(e, c) c.find(e) != c.end()

namespace neurobots_database_gui
{

Tree::Tree(std::unordered_map<std::string, Element*>& elements) :
				m_all(elements)
{
	//get 'room' elements, we use them as root elements
	for (auto& it : m_all)
	{
//		ROS_INFO_STREAM("el: " << it.first);
		if (it.second->m_type == "room")
		{
//			ROS_INFO_STREAM(it.first);
			m_roots[it.first] = it.second;
			for (auto& it2 : it.second->m_stringAttributes["connected"])
			{
				if (it2 != it.first)
				{
//					ROS_INFO_STREAM("-->" << it2);
					it.second->m_siblings[it2] = m_all[it2];
				}
			}
		}
		else if (FIND("position", it.second->m_stringAttributes))
		{
			std::string parent = it.second->m_stringAttributes["position"][0];
//			ROS_INFO_STREAM("position: " << it.first << " -> " << parent);
			m_all[parent]->m_children[it.first] = it.second;
			it.second->m_parents[parent] = m_all[parent];
		}
		else if (FIND("in", it.second->m_stringAttributes))
		{
			std::string parent = it.second->m_stringAttributes["in"][0];
//			ROS_INFO_STREAM("in: " << it.first << " -> " << parent);
			m_all[parent]->m_children[it.first] = it.second;
			it.second->m_parents[parent] = m_all[parent];
		}
//		else if (FIND("at", it.second->m_stringAttributes))
//		{
//			std::string parent = it.second->m_stringAttributes["at"][0];
//			ROS_INFO_STREAM("at: " << it.first << " -> " << parent);
//			m_all[parent]->m_children[it.first] = it.second;
//			it.second->m_parents[parent] = m_all[parent];
//		}
	}
}

Tree::~Tree()
{
	for (auto& it : m_all)
	{
		delete it.second;
	}
	m_all.clear();
}

} /* namespace neurobots_database_gui */
