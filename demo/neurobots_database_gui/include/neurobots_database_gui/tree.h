/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: tree.h
 */

#ifndef H143AA451_4E05_47F5_8282_EF29DCCFE741
#define H143AA451_4E05_47F5_8282_EF29DCCFE741

#include <neurobots_database_gui/element.h>
#include <unordered_map>
#include <vector>

namespace neurobots_database_gui
{

class Tree
{
public:
	Tree(std::unordered_map<std::string, Element*>& elements);
	virtual ~Tree();

	std::unordered_map<std::string, Element*> m_roots;
	std::unordered_map<std::string, Element*> m_all;
};

} /* namespace neurobots_database_gui */

#endif /* H143AA451_4E05_47F5_8282_EF29DCCFE741 */
