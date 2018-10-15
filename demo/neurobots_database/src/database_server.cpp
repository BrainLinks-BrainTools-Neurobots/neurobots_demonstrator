/*
 * database_server.h
 *
 *  Created on: May 2, 2016
 *      Author: Felix Burget
 */

#include <neurobots_database/database_server.h>

namespace neurobots_database
{

//Constructor
DatabaseServer::DatabaseServer(std::string domain) : m_database(Database(domain))
{
	ros::NodeHandle nh("~");

	//Bind Services, get
	m_getObject = nh.advertiseService("get_object", &Database::getObject, &m_database);
	m_getObjects = nh.advertiseService("get_objects", &Database::getObjects, &m_database);
	m_getWorld = nh.advertiseService("get_world", &Database::getWorld, &m_database);

	//set
	m_addObject = nh.advertiseService("add_object", &Database::addObject, &m_database);
	m_updateObject = nh.advertiseService("update_object", &Database::updateObject, &m_database);
	m_removeObject = nh.advertiseService("remove_object", &Database::removeObject, &m_database);

	//debug services (no checks, just sets an attribute)
	m_addEmptyObject = nh.advertiseService("debug/add_empty_object", &Database::addEmptyObject, &m_database);
	m_setBoolAttribute = nh.advertiseService("debug/set_bool", &Database::updateObjectBoolAttribute, &m_database);
	m_setStringAttribute = nh.advertiseService("debug/set_string", &Database::updateObjectStringAttribute, &m_database);
}

//Destructor
DatabaseServer::~DatabaseServer()
{
}

} /* namespace neurobots_database */

