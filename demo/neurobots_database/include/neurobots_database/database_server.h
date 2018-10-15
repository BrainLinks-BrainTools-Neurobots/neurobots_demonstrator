/*
 * database_server.h
 *
 *  Created on: May 2, 2016
 *      Author: Felix Burget
 */

#ifndef DATABASE_SERVER_H_
#define DATABASE_SERVER_H_

#include <neurobots_database/database.h>
#include <ros/ros.h>


namespace neurobots_database
{
//* NeurobotsDatabase
/**
* Class providing services to query the state of the world
*/
class DatabaseServer
{
public:
    /*!
    * \brief DatabaseServer Constructor
    */
	DatabaseServer(std::string domain);
    /*!
    * \brief DatabaseServer Destructor
    */
	~DatabaseServer();

protected:
	//World Database Object
    Database m_database;    /**< Database representing the state of the world */

	//Services, getters
	ros::ServiceServer m_getObject; /**< Service returning an object */
	ros::ServiceServer m_getObjects; /**< Service returning all objects */
	ros::ServiceServer m_getWorld; /**< Service returning complete world */

	//Services, setters
	ros::ServiceServer m_addObject; /**< Service to add an object (update and add call the same method) */
	ros::ServiceServer m_updateObject; /**< Service to update an existing object (update and add call the same method) */
	ros::ServiceServer m_removeObject; /**< Service to remove an existing object */

	//debug services (no checks, just sets an attribute)
	ros::ServiceServer m_addEmptyObject; /**< Inserts an object based on name and type */
	ros::ServiceServer m_setStringAttribute; /**< Sets an string attribute (first value of vector) of an given object*/
	ros::ServiceServer m_setBoolAttribute; /**< Sets an bool attribute (first value of vector) of an given object*/
};

}
#endif /* DATABASE_SERVER_H_ */
