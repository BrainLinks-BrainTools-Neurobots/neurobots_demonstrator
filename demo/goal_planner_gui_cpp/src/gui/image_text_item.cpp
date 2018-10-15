/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: image_text_item.cpp
 */

#include <goal_planner_gui/gui/image_text_item.h>

namespace goal_planner_gui
{

ImageTextItem::ImageTextItem() :
				m_text("-----"),
				m_image("")
{
}

ImageTextItem::ImageTextItem(const QString& text,
		const QString& image) :
				m_image(image),
				m_text(text)
{
}

ImageTextItem::ImageTextItem(const std::string& text,
		const std::string& image) :
				m_image(QString::fromStdString(image)),
				m_text(QString::fromStdString(text))
{
}

ImageTextItem::ImageTextItem(const ImageTextItem& other) :
				m_text(other.m_text),
				m_image(other.m_image)
{
}

ImageTextItem::~ImageTextItem()
{
}

} /* namespace goal_planner_gui */

