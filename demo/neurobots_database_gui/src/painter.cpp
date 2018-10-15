/*
 * Copyright (c) ROOM_DIST17 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 27, ROOM_DIST17
 *      Author: kuhnerd
 * 	  Filename: painter.cpp
 */

#include <neurobots_database_gui/element.h>
#include <neurobots_database_gui/painter.h>
#include <neurobots_database_gui/tree.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <qrgb.h>

//room
#define ROOM_DIST 30
#define ROOM_BORDER_WIDTH 10
#define DOOR_WIDTH 0.2

//shelf
#define SHELF_BORDER_WIDTH 5
#define BORDER_WIDTH_2 5

#define BROWN QColor::fromRgb(139, 69, 19)

namespace neurobots_database_gui
{

Painter::Painter() :
				m_painter(NULL),
				m_tree(NULL),
				m_imageFiles( {
						{ "cup", "redcup.png" },
						{ "bottle", "bottle.png" },
						{ "ballon", "ballon.png" },
						{ "cylinder", "glass.png" },
						{ "wwineglas", "wwineglas.png" },
						{ "beerstein", "beerstein.png" },
						{ "vase", "vase2.png" },
						{ "red-wine", "red_grapes.png" },
						{ "white-wine", "white_grapes.png" },
						{ "water", "water.png" },
						{ "lemonade", "lemonade.png" },
						{ "apple-juice", "apple.png" },
						{ "orange-juice", "orange.png" },
						{ "beer", "beer.png" },
						{ "tulip", "tulip.png" },
						{ "sunflower", "sunflower.png" },
						{ "rose", "rose.png" },
						{ "tulip_mask", "tulip_mask.png" },
						{ "sunflower_mask", "sunflower_mask.png" },
						{ "rose_mask", "rose_mask.png" },
						{ "kitchen", "kitchen.png" },
						{ "garden", "garden.png" },
						{ "bathroom", "bathroom.png" },
						{ "living-room", "living-room.png" },
						{ "pantry", "pantry.png" } })
{
	blackPen = QPen(Qt::black);
	blackPen.setWidth(1);
	boldBlackPen = QPen(Qt::black);
	boldBlackPen.setWidth(ROOM_BORDER_WIDTH);
	boldWhitePen = QPen(Qt::white);
	boldWhitePen.setWidth(ROOM_BORDER_WIDTH);
	boldBrownPen = QPen(BROWN);
	boldBrownPen.setWidth(SHELF_BORDER_WIDTH);
	boldBlackPen5Pen = QPen(Qt::black);
	boldBlackPen5Pen.setWidth(BORDER_WIDTH_2);
	textPen = QPen(Qt::black);
	textFont.setPixelSize(10);

	//images
	m_imagePath = QString::fromStdString(ros::package::getPath("neurobots_scenario") + "/images/");
	if (!m_imageOmnirob.load(m_imagePath + "omnirob_2.png"))
		ROS_ERROR("omnirob image not available");
	if (!m_imageHumanFriend.load(m_imagePath + "woman.png"))
		ROS_ERROR("friend image not available");
	if (!m_imageHumanMe.load(m_imagePath + "man.png"))
		ROS_ERROR("me image not available");
	if (!m_imageFlowerbed.load(m_imagePath + "flowerbed.jpg"))
		ROS_ERROR("flowerbed image not available");
}

Painter::~Painter()
{
}

void Painter::paint(Tree* tree,
		QPainter* painter,
		QSize& size)
{
	m_painter = painter;
	m_size = size;
	m_tree = tree;

	if (m_tree->m_roots.empty())
		return;

	static std::vector<std::string> drawFirst = { "shelf", "table", "furniture", "flowerbed" };

	//we have rooms at the top level
	if (m_tree->m_roots.begin()->second->m_type == "room")
	{
		drawRooms(m_tree->m_roots);
		std::vector<std::string> first;
		std::vector<std::string> second;

		//sort elements
		for (auto& it : m_tree->m_roots)
			for (auto& it2 : it.second->m_children)
				if (std::find(drawFirst.begin(), drawFirst.end(), it2.second->m_type) != drawFirst.end())
					first.push_back(it2.first);
				else
					second.push_back(it2.first);

		//draw
		for (auto& it2 : first)
			draw(m_tree->m_all[it2], m_tree->m_all[it2]->m_children.size());

		for (auto& it2 : second)
			draw(m_tree->m_all[it2], m_tree->m_all[it2]->m_children.size());
	}
//	else
//	{
//		for (auto& it : m_tree->m_roots)
//			draw(it.second, it.second->m_children.size());
//	}
}

void Painter::draw(Element* element,
		int children)
{
	if (element->m_type == "shelf")
	{
		drawShelf(element);
	}
	else if (element->m_type == "flowerbed")
	{
		drawFlowerbed(element);
	}
	else if (element->m_type == "table")
	{
		drawTable(element);
	}
	else if (element->m_type == "robot")
	{
		drawRobot(element);
	}
	else if (element->m_type == "human")
	{
		drawHuman(element);
	}
	else if (element->m_type == "cup")
	{
		drawCup(element, children);
	}
	else if (element->m_type == "glass")
	{
		drawGlass(element, children);
	}
	else if (element->m_type == "bottle")
	{
		drawBottle(element, children);
	}
	else if (element->m_type == "vase")
	{
		drawVase(element, children);
	}
	else if (element->m_type == "tulip"
			|| element->m_type == "rose"
			|| element->m_type == "sunflower")
	{
		drawFlower(element, children);
	}
	else
	{
		ROS_WARN_STREAM("Unknown element of type: " << element->m_type << " and first parent " << element->m_parents.begin()->first);
	}

	for (auto& it : element->m_children)
	{
		draw(it.second, children);
	}
}

void Painter::drawRooms(std::unordered_map<std::string, Element*>& rooms)
{
	int width;
	int height;
	std::string maxConnectionRoom;
	if (rooms.size() == 1)
	{
		width = 1;
		height = 1;
		m_roomAvailable = std::vector<std::vector<bool>>(width, std::vector<bool>(height, true));
		maxConnectionRoom = rooms.begin()->first;
	}
	else
	{
		int maxConnections = -1;

		for (auto& it : rooms)
		{
			if ((int) it.second->m_siblings.size() > maxConnections)
			{
				maxConnections = it.second->m_siblings.size();
				maxConnectionRoom = it.first;
			}
		}

		//get size needed to draw all rooms by overestimating the room number
		m_roomAvailable = std::vector<std::vector<bool>>(2 * rooms.size(), std::vector<bool>(2 * rooms.size(), true));
		getNumberOfRoomsInXandY(rooms[maxConnectionRoom], "", 0, 0);
		int xMin = m_roomAvailable.size();
		int xMax = 0;
		int yMin = m_roomAvailable[0].size();
		int yMax = 0;
		for (size_t x = 0; x < m_roomAvailable.size(); ++x)
		{
			for (size_t y = 0; y < m_roomAvailable[0].size(); ++y)
			{
				if (!m_roomAvailable[x][y])
				{
					if (x < xMin)
						xMin = x;
					if (x > xMax)
						xMax = x;
					if (y < yMin)
						yMin = y;
					if (y > yMax)
						yMax = y;
				}
			}
		}

		width = xMax - xMin + 1;
		height = yMax - yMin + 1;
		m_roomAvailable = std::vector<std::vector<bool>>(width, std::vector<bool>(height, true));
	}

	double screenWidth = m_size.width() - ROOM_DIST;
	double screenHeight = m_size.height() - ROOM_DIST;

	double roomWidth = screenWidth / width - ROOM_DIST * width;
	double roomHeight = screenHeight / height - ROOM_DIST * height;

//	ROS_INFO("Needed width and height: %d (%f px/room)), %d (%f px/room)", width, roomWidth, height, roomHeight);

	m_roomAvailable[width / 2][height / 2] = false;
	drawRoom(0, 0, roomWidth, roomHeight, "", rooms[maxConnectionRoom]);
}

void Painter::drawRoom(int x,
		int y,
		double width,
		double height,
		std::string last,
		Element* room)
{
	double topLeftX = m_size.width() / 2 - width / 2 + x * width + x * ROOM_DIST;
	if (m_roomAvailable.size() % 2 == 0)
		topLeftX += width / 2;
	double topLeftY = m_size.height() / 2 - height / 2 + y * height + y * ROOM_DIST;
	if (m_roomAvailable[0].size() % 2 == 0)
		topLeftY += height / 2;
	int gridX = x + m_roomAvailable.size() / 2;
	int gridY = y + m_roomAvailable[0].size() / 2;
	double doorWidth = width * DOOR_WIDTH;

	QRect roomSize(topLeftX + ROOM_BORDER_WIDTH / 2, topLeftY + ROOM_BORDER_WIDTH / 2, width - + ROOM_BORDER_WIDTH, height - + ROOM_BORDER_WIDTH / 2);
	m_elementSizes[room->m_name] = roomSize;

	//draw room
	m_painter->setPen(boldBlackPen);
	if (room->m_name == "garden")
	{
		m_painter->fillRect(roomSize, Qt::green);
	}
	else
	{
		m_painter->fillRect(roomSize, QColor("#ffe5cc"));
	}
	m_painter->drawRect(topLeftX, topLeftY, width, height);

	for (auto& it : room->m_siblings)
	{
		//only go down
		if (it.first != last)
		{
			if (gridX - 1 >= 0 && m_roomAvailable[gridX - 1][gridY])
			{
				m_roomAvailable[gridX - 1][gridY] = false;
				drawRoom(x - 1, y, width, height, room->m_name, it.second);
				m_painter->setPen(boldBlackPen);
				m_painter->drawRect(topLeftX - ROOM_DIST, topLeftY + height / 2 - doorWidth / 2, ROOM_DIST, doorWidth);
				m_painter->setPen(boldWhitePen);
				m_painter->fillRect(topLeftX - ROOM_DIST - ROOM_BORDER_WIDTH / 2 - 1, topLeftY + height / 2 - doorWidth / 2 + ROOM_BORDER_WIDTH / 2, ROOM_DIST + ROOM_BORDER_WIDTH + 2,
						doorWidth - ROOM_BORDER_WIDTH, QColor("#ffe5cc"));
			}
			else if (gridX + 1 < m_roomAvailable.size() && m_roomAvailable[gridX + 1][gridY])
			{
				m_roomAvailable[gridX + 1][gridY] = false;
				drawRoom(x + 1, y, width, height, room->m_name, it.second);
				m_painter->setPen(boldBlackPen);
				m_painter->drawRect(topLeftX + width, topLeftY + height / 2 - doorWidth / 2, ROOM_DIST, doorWidth);
				m_painter->setPen(boldWhitePen);
				m_painter->fillRect(topLeftX + width - ROOM_BORDER_WIDTH / 2 - 1, topLeftY + height / 2 - doorWidth / 2 + ROOM_BORDER_WIDTH / 2, ROOM_DIST + ROOM_BORDER_WIDTH + 2,
						doorWidth - ROOM_BORDER_WIDTH, QColor("#ffe5cc"));
			}
			else if (gridY - 1 >= 0 && m_roomAvailable[gridX][gridY - 1])
			{
				m_roomAvailable[gridX][gridY - 1] = false;
				drawRoom(x, y - 1, width, height, room->m_name, it.second);
				m_painter->setPen(boldBlackPen);
				m_painter->drawRect(topLeftX + width / 2 - doorWidth / 2, topLeftY - ROOM_DIST, doorWidth, ROOM_DIST);
				m_painter->setPen(boldWhitePen);
				m_painter->fillRect(topLeftX + width / 2 - doorWidth / 2 + ROOM_BORDER_WIDTH / 2, topLeftY - ROOM_DIST - ROOM_BORDER_WIDTH / 2 - 1, doorWidth - ROOM_BORDER_WIDTH,
				ROOM_DIST + ROOM_BORDER_WIDTH + 2, QColor("#ffe5cc"));
			}
			else if (gridY + 1 < m_roomAvailable[0].size() && m_roomAvailable[gridX][gridY + 1])
			{
				m_roomAvailable[gridX][gridY + 1] = false;
				drawRoom(x, y + 1, width, height, room->m_name, it.second);
				m_painter->setPen(boldBlackPen);
				m_painter->drawRect(topLeftX + width / 2 - doorWidth / 2, topLeftY + height, doorWidth, ROOM_DIST);
				m_painter->setPen(boldWhitePen);
				m_painter->fillRect(topLeftX + width / 2 - doorWidth / 2 + ROOM_BORDER_WIDTH / 2, topLeftY + height - ROOM_BORDER_WIDTH / 2 - 1, doorWidth - ROOM_BORDER_WIDTH,
				ROOM_DIST + ROOM_BORDER_WIDTH + 2, QColor("#ffe5cc"));
			}
		}
	}

	if (m_imagesRooms.find(room->m_name) == m_imagesRooms.end())
	{
		if (!m_imagesRooms[room->m_name].load(m_imagePath + m_imageFiles[room->m_name].c_str()))
		{
			ROS_ERROR("%s image not available", m_imageFiles[room->m_name].c_str());
			exit(12);
		}
	}

	//draw icon
	QImage& image = m_imagesRooms[room->m_name];
	const QSize imageSize = image.size().scaled(roomSize.height() * 0.15 - 10, roomSize.height() * 0.15 - 10, Qt::KeepAspectRatio);

	QPoint atPoint = roomSize.bottomLeft() + QPoint(10, -10 - imageSize.height());

	QRect imageRect(atPoint, imageSize);
	m_painter->drawImage(imageRect, image);

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(topLeftX + 20 + imageSize.width(), topLeftY + height - 10, QString::fromStdString(room->m_name));
}

void Painter::getNumberOfRoomsInXandY(Element* room,
		std::string last,
		int x,
		int y)
{
	int gridX = x + m_roomAvailable.size() / 2;
	int gridY = y + m_roomAvailable[0].size() / 2;

	m_roomAvailable[gridX][gridY] = false;

	for (auto& it : room->m_siblings)
	{
		//only go down
		if (it.first != last)
		{
			if (gridX - 1 >= 0 && m_roomAvailable[gridX - 1][gridY])
			{
				m_roomAvailable[gridX - 1][gridY] = false;
				getNumberOfRoomsInXandY(it.second, room->m_name, x - 1, y);
			}
			else if (gridX + 1 < m_roomAvailable.size() && m_roomAvailable[gridX + 1][gridY])
			{
				m_roomAvailable[gridX + 1][gridY] = false;
				getNumberOfRoomsInXandY(it.second, room->m_name, x + 1, y);
			}
			else if (gridY - 1 >= 0 && m_roomAvailable[gridX][gridY - 1])
			{
				m_roomAvailable[gridX][gridY - 1] = false;
				getNumberOfRoomsInXandY(it.second, room->m_name, x, y - 1);
			}
			else if (gridY + 1 < m_roomAvailable[0].size() && m_roomAvailable[gridX][gridY + 1])
			{
				m_roomAvailable[gridX][gridY + 1] = false;
				getNumberOfRoomsInXandY(it.second, room->m_name, x, y + 1);
			}
			else
			{
				ROS_WARN("Cannot draw room because there is no place available: %s", it.first.c_str());
			}
		}
	}
}

void Painter::drawShelf(Element* shelf)
{
	const std::string& in = shelf->m_stringAttributes["in"][0];
	std::string aligned = "left";
	if (shelf->m_stringAttributes.find("aligned") != shelf->m_stringAttributes.end())
	{
		aligned = shelf->m_stringAttributes["aligned"][0];
	}

	const QRect& roomSize = m_elementSizes[in];
	QRect shelfSize;

	if (aligned == "left")
	{
		shelfSize = QRect(roomSize.x() + 10, roomSize.y() + 10, roomSize.width() / 2 - roomSize.width() * 0.05, roomSize.height() * 0.7);
	}
	else
	{
		shelfSize = QRect(roomSize.topRight().x() - roomSize.width() / 2 + roomSize.width() * 0.05 - 10, roomSize.y() + 10, roomSize.width() / 2 - roomSize.width() * 0.05, roomSize.height() * 0.7);
	}

	m_elementSizes[shelf->m_name] = shelfSize;
	m_elementPositions[shelf->m_name] = QLine(shelfSize.topLeft() + QPoint(1, shelfSize.height() / 2), shelfSize.topRight() + QPoint(-1, shelfSize.height() / 2));
	m_elementObjectPosition[shelf->m_name] = 0;

	m_painter->setPen(boldBrownPen);
	m_painter->drawLine(shelfSize.topLeft(), shelfSize.bottomLeft());
	m_painter->drawLine(shelfSize.topRight(), shelfSize.bottomRight());
	m_painter->drawLine(shelfSize.bottomLeft() - QPoint(0, 15), shelfSize.bottomRight() - QPoint(0, 15));
	m_painter->drawLine(shelfSize.topLeft() + QPoint(0, 10), shelfSize.topRight() + QPoint(0, 10));
	m_painter->drawLine(shelfSize.topLeft() + QPoint(1, shelfSize.height() / 2), shelfSize.topRight() + QPoint(-1, shelfSize.height() / 2));

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(shelfSize.bottomLeft().x() + 10, shelfSize.bottomLeft().y(), QString::fromStdString(shelf->m_name));
}

void Painter::drawFlowerbed(Element* flowerbed)
{
	const std::string& in = flowerbed->m_stringAttributes["in"][0];

	const QRect& roomSize = m_elementSizes[in];
	QRect flowerbedSize(roomSize.x() + 10, roomSize.y() + 10, roomSize.width() - 20, roomSize.height() * 0.7);
	m_elementSizes[flowerbed->m_name] = flowerbedSize;
	m_elementPositions[flowerbed->m_name] = QLine(flowerbedSize.topLeft() + QPoint(0, flowerbedSize.height() / 2), flowerbedSize.topRight() + QPoint(0, flowerbedSize.height() / 2));
	m_elementObjectPosition[flowerbed->m_name] = 0;

	m_painter->fillRect(flowerbedSize, BROWN);
	m_painter->setPen(boldBlackPen5Pen);
	m_painter->drawRect(flowerbedSize);

	//draw icon
	const QSize imageSize = m_imageFlowerbed.size().scaled(flowerbedSize.height() * 0.25 - 10, flowerbedSize.height() * 0.25 - 10, Qt::KeepAspectRatio);

	QPoint atPoint = flowerbedSize.bottomLeft() + QPoint(10, -10 - imageSize.height());

	QRect imageRect(atPoint, imageSize);
	m_painter->drawImage(imageRect, m_imageFlowerbed);

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(flowerbedSize.bottomLeft().x() + 20 + imageSize.width(), flowerbedSize.bottomLeft().y() - 10, QString::fromStdString(flowerbed->m_name));
}

void Painter::drawTable(Element* table)
{
	const std::string& in = table->m_stringAttributes["in"][0];
	const QRect& roomSize = m_elementSizes[in];
	QRect tableSize(roomSize.x() + 0.2 * roomSize.width(), roomSize.y() + 0.4 * roomSize.height(), roomSize.width() * 0.6, roomSize.height() * 0.3);
	m_elementSizes[table->m_name] = tableSize;
	m_elementPositions[table->m_name] = QLine(tableSize.topLeft(), tableSize.topRight());
	m_elementObjectPosition[table->m_name] = 0;

	m_painter->setPen(boldBrownPen);
	m_painter->drawLine(tableSize.topLeft(), tableSize.topRight());
	m_painter->drawLine(tableSize.topLeft() + QPoint(tableSize.width() * 0.1, 0), tableSize.bottomLeft() + QPoint(tableSize.width() * 0.1, 0));
	m_painter->drawLine(tableSize.topRight() - QPoint(tableSize.width() * 0.1, 0), tableSize.bottomRight() - QPoint(tableSize.width() * 0.1, 0));

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(tableSize.bottomLeft().x() + tableSize.width() * 0.1 + 10, tableSize.bottomLeft().y() - 10, QString::fromStdString(table->m_name));
}

void Painter::drawRobot(Element* robot)
{
	const std::string& in = robot->m_stringAttributes["in"][0];
	const std::string& at = robot->m_stringAttributes["at"][0];

	const QRect& roomSize = m_elementSizes[in];

	const QSize imageSize = m_imageOmnirob.size().scaled(roomSize.height() * 0.3 - 10, roomSize.height() * 0.3 - 10, Qt::KeepAspectRatio);
	QPoint atPoint;

	if (at == "nowhere")
	{
		atPoint = roomSize.bottomLeft() + QPoint(roomSize.width() / 2 - imageSize.width() / 2, -roomSize.height() * 0.3 + 10);
	}
	else
	{
		const QRect& elementSize = m_elementSizes[at];
		atPoint = elementSize.bottomLeft() + QPoint(elementSize.width() / 2 - imageSize.width() / 2, -imageSize.height());
	}

	QRect imageRect(atPoint, imageSize);
	m_elementSizes[robot->m_name] = imageRect;
	m_elementPositions[robot->m_name] = QLine(imageRect.topLeft() + QPoint(0, imageRect.height() / 2), imageRect.topRight() + QPoint(0, imageRect.height() / 2));
	m_elementObjectPosition[robot->m_name] = 0;
	m_painter->drawImage(imageRect, m_imageOmnirob);
}

void Painter::drawHuman(Element* human)
{
	const std::string& in = human->m_stringAttributes["in"][0];

	const QRect& roomSize = m_elementSizes[in];

	QImage& image = human->m_name == "me" ? m_imageHumanMe : m_imageHumanFriend;
	const QSize imageSize = image.size().scaled(roomSize.height() * 0.2 - 10, roomSize.height() * 0.2 - 10, Qt::KeepAspectRatio);

	QPoint delta =
			human->m_name == "me" ? QPoint(roomSize.width() - 10 - 2 * imageSize.width(), -imageSize.height() - 20) : QPoint(roomSize.width() - 10 - imageSize.width(), -imageSize.height() - 20);
	QPoint atPoint;

	atPoint = roomSize.bottomLeft() + delta;

	QRect imageRect(atPoint, imageSize);
	m_elementSizes[human->m_name] = imageRect;
	m_elementPositions[human->m_name] = QLine(imageRect.topLeft() + QPoint(0, imageRect.height() / 2), imageRect.topRight() + QPoint(0, imageRect.height() / 2));
	m_elementObjectPosition[human->m_name] = 0;
	m_painter->drawImage(imageRect, image);
}

void Painter::drawCup(Element* vessel,
		int numberOfObjects)
{
	const std::string& position = vessel->m_stringAttributes["position"][0];
	const std::string& color = vessel->m_stringAttributes["colored"][0];
	std::string contains = "empty";
	if (vessel->m_stringAttributes.find("contains") != vessel->m_stringAttributes.end())
	{
		contains = vessel->m_stringAttributes["contains"][0];
	}

	const QLine& line = m_elementPositions[position];
	int pos = m_elementObjectPosition[position];
	++m_elementObjectPosition[position];
	const int numberOfObjects2 = std::max(numberOfObjects, 5);
	const double lineLength = fabs(line.x2() - line.x1());
	const double width = lineLength / numberOfObjects2;

	std::string imageFile = m_imageFiles[vessel->m_type];

	QImage& img = getColoredImage((m_imagePath + imageFile.c_str()).toStdString(), width, color);

	QPoint p(line.p1().x() + pos * width, line.p1().y() - img.height());
	m_painter->drawImage(p, img);

	//draw content
	if (contains != "empty")
	{
		const double widthContent = width * 0.5;
		std::string imageFileContent = m_imageFiles[contains];
		QImage& imgContent = getColoredImage((m_imagePath + imageFileContent.c_str()).toStdString(), widthContent, "");
		m_painter->drawImage(p + QPoint(width / 2 - widthContent / 2, img.height() / 2 - imgContent.height() / 2), imgContent);
	}

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(line.p1() + QPoint(10 + pos * width, 15), QString::fromStdString(vessel->m_name));
}

void Painter::drawGlass(Element* vessel,
		int numberOfObjects)
{
	const std::string& position = vessel->m_stringAttributes["position"][0];
	const std::string& shape = vessel->m_stringAttributes["shaped"][0];
	std::string contains = "empty";
	if (vessel->m_stringAttributes.find("contains") != vessel->m_stringAttributes.end())
	{
		contains = vessel->m_stringAttributes["contains"][0];
	}

	const QLine& line = m_elementPositions[position];
	int pos = m_elementObjectPosition[position];
	++m_elementObjectPosition[position];
	const int numberOfObjects2 = std::max(numberOfObjects, 5);
	const double lineLength = fabs(line.x2() - line.x1());
	const double width = lineLength / numberOfObjects2;

	std::string imageFile = m_imageFiles[shape];

	QImage& img = getColoredImage((m_imagePath + imageFile.c_str()).toStdString(), width, "");

	QPoint p(line.p1().x() + pos * width, line.p1().y() - img.height());
	m_painter->drawImage(p, img);

	//draw content
	if (contains != "empty")
	{
		const double widthContent = width * 0.5;
		std::string imageFileContent = m_imageFiles[contains];
		QImage& imgContent = getColoredImage((m_imagePath + imageFileContent.c_str()).toStdString(), widthContent, "");
		m_painter->drawImage(p + QPoint(width / 2 - widthContent / 2, img.height() * 0.2 - imgContent.height() / 2), imgContent);
	}

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(line.p1() + QPoint(10 + pos * width, 15), QString::fromStdString(vessel->m_name));
}

void Painter::drawVase(Element* vase,
		int numberOfObjects)
{
	const std::string& position = vase->m_stringAttributes["position"][0];
	const std::string& color = vase->m_stringAttributes["colored"][0];

	const QLine& line = m_elementPositions[position];
	int pos = m_elementObjectPosition[position];
	++m_elementObjectPosition[position];
	const int numberOfObjects2 = std::max(numberOfObjects, 5);
	const double lineLength = fabs(line.x2() - line.x1());
	const double width = lineLength / numberOfObjects2;

	std::string imageFile = m_imageFiles[vase->m_type];

	QImage& img = getColoredImage((m_imagePath + imageFile.c_str()).toStdString(), width, color);

	QRect rect(line.p1().x() + pos * width, line.p1().y() - img.height(), img.width(), img.height());
	m_elementSizes[vase->m_name] = rect;

	m_painter->drawImage(rect, img);

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(line.p1() + QPoint(10 + pos * width, 15), QString::fromStdString(vase->m_name));
}

void Painter::drawBottle(Element* vessel,
		int numberOfObjects)
{
	const std::string& position = vessel->m_stringAttributes["position"][0];
	std::string contains = "empty";
	if (vessel->m_stringAttributes.find("contains") != vessel->m_stringAttributes.end())
	{
		contains = vessel->m_stringAttributes["contains"][0];
	}

	const QLine& line = m_elementPositions[position];
	int pos = m_elementObjectPosition[position];
	++m_elementObjectPosition[position];
	const int numberOfObjects2 = std::max(numberOfObjects, 5);
	const double lineLength = fabs(line.x2() - line.x1());
	const double width = lineLength / numberOfObjects2;

	std::string imageFile = m_imageFiles[vessel->m_type];

	QImage& img = getColoredImage((m_imagePath + imageFile.c_str()).toStdString(), width, "");

	QPoint p(line.p1().x() + pos * width, line.p1().y() - img.height());
	m_painter->drawImage(p, img);

	//draw content
	if (contains != "empty")
	{
		const double widthContent = width * 0.9;
		std::string imageFileContent = m_imageFiles[contains];
		QImage& imgContent = getColoredImage((m_imagePath + imageFileContent.c_str()).toStdString(), widthContent, "");
		m_painter->drawImage(p + QPoint(width / 2 - widthContent / 2, img.height() / 2 - imgContent.height() / 2), imgContent);
	}

	//draw text
	m_painter->setPen(blackPen);
	m_painter->setFont(textFont);
	m_painter->drawText(line.p1() + QPoint(10 + pos * width, 15), QString::fromStdString(vessel->m_name));
}

void Painter::drawFlower(Element* flower,
		int numberOfObjects)
{
	const std::string& position = flower->m_stringAttributes["position"][0];
	const std::string& color = flower->m_stringAttributes["colored"][0];
	const std::string& parentType = flower->m_parents.begin()->second->m_type;
	if (parentType == "vase" || parentType == "robot" || parentType == "human")
	{
		const QRect& vase = m_elementSizes[position];

		std::string imageFile = m_imageFiles[flower->m_type];
		std::string maskFile = m_imageFiles[flower->m_type + "_mask"];

		QImage& img = getColoredImage((m_imagePath + imageFile.c_str()).toStdString(), vase.width(), color, (m_imagePath + maskFile.c_str()).toStdString(), false);

		m_painter->drawImage(vase.bottomLeft().x(), vase.bottomLeft().y() - vase.height() * 1.3, img);

		//draw text
		m_painter->setPen(blackPen);
		m_painter->setFont(textFont);
		m_painter->drawText(vase.bottomLeft().x() + vase.width() / 2, vase.bottomLeft().y() - vase.height() * 1.3, QString::fromStdString(flower->m_name));
	}
	else
	{
		int flowersMax = 5;
		if (parentType == "flowerbed")
			flowersMax = 8;
		const QLine& line = m_elementPositions[position];
		int pos = m_elementObjectPosition[position];
		++m_elementObjectPosition[position];
		const int numberOfObjects2 = std::max(numberOfObjects, flowersMax);
		const double lineLength = fabs(line.x2() - line.x1());
		const double width = lineLength / numberOfObjects2;

		std::string imageFile = m_imageFiles[flower->m_type];
		std::string maskFile = m_imageFiles[flower->m_type + "_mask"];

		QImage& img = getColoredImage((m_imagePath + imageFile.c_str()).toStdString(), width, color, (m_imagePath + maskFile.c_str()).toStdString());

		m_painter->drawImage(line.p1().x() + pos * width, line.p1().y() - img.height(), img);

		//draw text
		m_painter->setPen(blackPen);
		m_painter->setFont(textFont);
		m_painter->drawText(line.p1() + QPoint(10 + pos * width, 15), QString::fromStdString(flower->m_name));
	}
}

QImage& Painter::getColoredImage(const std::string& file,
		int width,
		const std::string& color,
		const std::string maskFile,
		const bool drawBlackPartOfMask)
{
	const auto& it1 = m_coloredImages.find(file);
	if (it1 != m_coloredImages.end())
	{
		const auto& it2 = it1->second.find(color);
		if (it2 != it1->second.end())
		{
			const auto& it3 = it2->second.find(width);
			if (it3 != it2->second.end())
			{
				return it3->second;
			}
		}
	}

	QImage& img = m_coloredImages[file][color][width];
	if (!img.load(file.c_str()))
	{
		ROS_ERROR("image not available: %s", file.c_str());
		return img;
	}

	QImage mask;
	if (!maskFile.empty())
	{
		if (!mask.load(maskFile.c_str()))
		{
			ROS_ERROR("image not available: %s", maskFile.c_str());
			return img;
		}
	}

	if (!color.empty())
	{
		QImage alpha = img.alphaChannel();
		for (size_t x = 0; x < img.width(); ++x)
		{
			for (size_t y = 0; y < img.height(); ++y)
			{
				QColor col(img.pixel(x, y));
				if (maskFile.empty() || (!maskFile.empty() && qRed(mask.pixel(x, y)) != 0))
				{
					if (color == "white" || color == "black")
					{
						col.setHsv(col.hue(), QColor(color.c_str()).saturation(), col.value());
					}
					else
					{
						col.setHsv(getHue(color), col.saturation(), col.value());
					}
					img.setPixel(x, y, col.rgb());
				}

				if (!drawBlackPartOfMask && qRed(mask.pixel(x, y)) == 0)
				{
					alpha.setPixel(x, y, 0);
				}
			}
		}
		img.setAlphaChannel(alpha);
	}
	img = img.scaledToWidth(width);

	return img;
}

int Painter::getHue(const std::string& color)
{
	return QColor(color.c_str()).hue();
}

} /* namespace neurobots_database_gui */
