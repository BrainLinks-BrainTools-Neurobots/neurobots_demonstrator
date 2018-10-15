/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2017
 *      Author: kuhnerd
 * 	  Filename: ref_item.h
 */

#ifndef H8E82B9CD_2F43_4853_9741_3CFAE62C8DC7
#define H8E82B9CD_2F43_4853_9741_3CFAE62C8DC7

#include <goal_planner_gui/gui/image_text_item.h>
#include <QList>
#include <QObject>
#include <QVariantList>

namespace goal_planner_gui
{

class RefItem: public QObject
{
Q_OBJECT

	Q_PROPERTY(QVariant name READ getName CONSTANT)
	Q_PROPERTY(QList<QVariant> arguments READ getArguments CONSTANT)

public:
	RefItem();
	RefItem(const RefItem& other);
	virtual ~RefItem();

	virtual QVariant getName() const;
	virtual QList<QVariant> getArguments() const;

protected:
	ImageTextItem* m_nameItem;
	QList<QVariant> m_args;
};

} /* namespace goal_planner_gui */

#endif /* H8E82B9CD_2F43_4853_9741_3CFAE62C8DC7 */
