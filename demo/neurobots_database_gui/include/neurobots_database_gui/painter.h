/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, 2017
 *      Author: kuhnerd
 * 	  Filename: painter.h
 */

#ifndef H31CFA093_434F_4760_949F_63EE67C7C6D8
#define H31CFA093_434F_4760_949F_63EE67C7C6D8
#include <qfont.h>
#include <qpainter.h>
#include <qpen.h>
#include <qsize.h>
#include <unordered_map>

namespace neurobots_database_gui
{

class Tree;
class Element;

class Painter
{
public:
	Painter();
	virtual ~Painter();

public:
	void paint(Tree* tree,
			QPainter *painter,
			QSize& size);

private:
	void draw(Element* element,
			int children);

	//rooms
	void drawRooms(std::unordered_map<std::string, Element*>& rooms);
	void drawRoom(int x,
			int y,
			double width,
			double height,
			std::string last,
			Element* element);
	void getNumberOfRoomsInXandY(Element* room,
			std::string last,
			int x,
			int y);

	void drawShelf(Element* shelf);
	void drawFlowerbed(Element* flowerbed);
	void drawTable(Element* table);
	void drawRobot(Element* robot);
	void drawHuman(Element* human);
	void drawCup(Element* vessel,
			int numberOfObjects);
	void drawGlass(Element* vessel,
			int numberOfObjects);
	void drawBottle(Element* vessel,
			int numberOfObjects);
	void drawVase(Element* vase,
			int numberOfObjects);
	void drawFlower(Element* flower,
			int numberOfObjects);

	QImage& getColoredImage(const std::string& file,
			int width,
			const std::string& color,
			const std::string maskFile = "",
			const bool drawBlackPartOfMask = true);
	int getHue(const std::string& color);

private:
	QPainter* m_painter;
	QSize m_size;
	Tree* m_tree;

	QFont textFont;
	QPen blackPen;
	QPen boldBlackPen;
	QPen boldWhitePen;
	QPen boldBrownPen;
	QPen boldBlackPen5Pen;
	QPen textPen;

	QImage m_imageOmnirob;
	QImage m_imageHumanFriend;
	QImage m_imageHumanMe;
	QImage m_imageFlowerbed;

	QString m_imagePath;

	std::unordered_map<std::string, std::unordered_map<std::string, std::unordered_map<int, QImage>>>m_coloredImages;
	std::unordered_map<std::string, QImage> m_imagesRooms;

	//rooms
	std::vector<std::vector<bool>> m_roomAvailable;
	std::unordered_map<std::string, QRect> m_elementSizes;
	std::unordered_map<std::string, QLine> m_elementPositions;
	std::unordered_map<std::string, int> m_elementObjectPosition;
	std::unordered_map<std::string, std::string> m_imageFiles;
};

}
/* namespace neurobots_database_gui */

#endif /* H31CFA093_434F_4760_949F_63EE67C7C6D8 */
