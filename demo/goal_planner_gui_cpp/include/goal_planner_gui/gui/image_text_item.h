/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: image_text_item.h
 */

#ifndef HF37BC311_5867_4C0C_B197_2A87463C699B
#define HF37BC311_5867_4C0C_B197_2A87463C699B

#include <QObject>
#include <QString>
#include <string>

namespace goal_planner_gui
{

class ImageTextItem: public QObject
{
	Q_OBJECT

public:
	ImageTextItem();
	ImageTextItem(const QString& text,
			const QString& image);
	ImageTextItem(const std::string& text,
				const std::string& image);
	ImageTextItem(const ImageTextItem& other);
	virtual ~ImageTextItem();

Q_PROPERTY(QString text MEMBER m_text CONSTANT)
Q_PROPERTY(QString image MEMBER m_image CONSTANT)

private:
	QString m_text;
	QString m_image;
};

} /* namespace goal_planner_gui */

Q_DECLARE_METATYPE(goal_planner_gui::ImageTextItem);

#endif /* HF37BC311_5867_4C0C_B197_2A87463C699B */
