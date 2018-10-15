/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: element.h
 */

#ifndef H17EDD3D6_6EC2_412A_BCFB_26B4009454D7
#define H17EDD3D6_6EC2_412A_BCFB_26B4009454D7
#include <string>
#include <unordered_map>
#include <vector>

namespace neurobots_database_gui
{

class Element
{
public:
	Element(const std::string& type,
			const std::string& name,
			const std::unordered_map<std::string, std::vector<std::string>>& otherStringAttributes,
			const std::unordered_map<std::string, bool>& otherBoolAttributes);
	Element();
	virtual ~Element();

	void setParent(Element* parent);

	std::string m_type;
	std::string m_name;
	std::unordered_map<std::string, std::vector<std::string>> m_stringAttributes;
	std::unordered_map<std::string, bool> m_boolAttributes;
	std::unordered_map<std::string, Element*> m_parents;
	std::unordered_map<std::string, Element*> m_siblings;
	std::unordered_map<std::string, Element*> m_children;
};

} /* namespace neurobots_database_gui */

#endif /* H17EDD3D6_6EC2_412A_BCFB_26B4009454D7 */
