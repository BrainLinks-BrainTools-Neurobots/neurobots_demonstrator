/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: simtrack_objects_to_database_mapper.cpp
 */

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <database_conversions/database_conversions.h>
#include <database_msgs/update_object.h>
#include <database_msgs/remove_object.h>
#include <database_msgs/get_object.h>

#include <database_msgs/database_change.h>

using namespace std;

bool init_database_triggered = false;

void database_callback(const database_msgs::database_change::ConstPtr& db_msgs)
{
	init_database_triggered = db_msgs->init;

	ROS_INFO_STREAM("Neurobots database has been restarted ...");
	//ROS_INFO_STREAM(init_database_triggered);
}



int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "simtrack_objects_to_database_mapper");

	//Node Handle
	ros::NodeHandle nh;

	//Neurobots Database Service Clients
	ros::ServiceClient clientUpdateObject = nh.serviceClient<database_msgs::update_object>("neurobots_database/update_object"); //adds object if not already present in database
	ros::ServiceClient clientRemoveObject = nh.serviceClient<database_msgs::remove_object>("neurobots_database/remove_object");
	ros::ServiceClient clientGetObject = nh.serviceClient<database_msgs::get_object>("neurobots_database/get_object");

	//Subscriber for database changes
	ros::Subscriber database_change_sub = nh.subscribe<database_msgs::database_change>("neurobots_database/database_change", 100, database_callback);

	//Get Names of Simtrack Models used
	// Note: since neurobots_objects.yaml is loaded for both cameras the "/shelf_camera/simtrack/model_names" and "/shelf_camera/simtrack/model_names"
	//       parameters should contain the same objects / object names
	std::vector<std::string> object_model_names;
	std::vector<std::string> object_model_types;

	//TF Listener to get transform_map_to_object for all simtrack objects
	tf::TransformListener listener;
	tf::StampedTransform transform_map_to_object;

	ros::Rate r(20);

	//Get names of objects to be detected
	ros::param::get("/shelf_right_camera/simtrack/model_names", object_model_names);

	//Get type of objects
	ros::param::get("/shelf_right_camera/simtrack/model_types", object_model_types);

	//Number of iterations after which a object is deleted from the database
	int max_object_not_seen_iter = 5;

	//Vector counting the number of iterations a object has not been seen
	vector<int> object_not_seen_counter(object_model_names.size());
	//Flag indicating whether object has already been removed from database 
	vector<bool> object_already_removed(object_model_names.size()); 
	//Flag indicating whether object has already been added to database
	vector<bool> object_already_added(object_model_names.size());  
	
	for (int i = 0 ; i < object_model_names.size() ; i++)
	{
		object_not_seen_counter[i] = 0;
		object_already_removed[i] = true;
		object_already_added[i] = false;
	}	
		

	while (ros::ok())
	{
		
		//If database is reinitialized we need to reset all parameters
		//ROS_INFO_STREAM("Checking for database initialization ...");
		//ROS_INFO_STREAM(init_database_triggered);
		if(init_database_triggered)
		{
			for (int i = 0 ; i < object_model_names.size() ; i++)
			{
				object_not_seen_counter[i] = 0;
				object_already_removed[i] = true;
				object_already_added[i] = false;
			}

			ROS_INFO_STREAM("Reinitializing simtrack parameters ...");
			
			//Reset performed -> reset flag to false
			init_database_triggered = false;
		}

		//Get Transforms from /map link to /shelf_camera/simtrack/model_name or /table_camera/simtrack/model_name
		int type_counter = 0;
		for (auto &it : object_model_names)
		{

			//ROS_INFO_STREAM(it);
			//Get object pose in map using the shelf camera
			bool transform_found = false;
			std::string position = "";

			try
			{
				listener.lookupTransform("/map", "/shelf_right_camera/simtrack/" + it, ros::Time(0), transform_map_to_object);
				transform_found = true;
				position = "shelfright";
				//ROS_INFO_STREAM("Transform /map to /shelf_right_camera/simtrack/" <<it<< " available ...");
			}
			catch (tf::TransformException& ex)
			{
				//ROS_ERROR("%s",ex.what());
				//No transform available
				//ROS_INFO_STREAM("Transform /map to /shelf_right_camera/simtrack/" <<it<< " not available!");
				//return false;
			}

			//Get object pose in map using the table camera
			if (!transform_found)
			{
				try
				{
					listener.lookupTransform("/map", "/table_camera/simtrack/" + it, ros::Time(0), transform_map_to_object);
					transform_found = true;
					position = "table1";
					//ROS_INFO_STREAM("Transform /map to /table_camera/simtrack/" <<it<< " available ...");
				}
				catch (tf::TransformException& ex)
				{
					//ROS_ERROR("%s",ex.what());
					//No transform available
					//ROS_INFO_STREAM("Transform /map to /table_camera/simtrack/" <<it<< " not available!");
					//return false;
				}
			}

			//Get object pose in map using the table camera
			if (!transform_found)
			{
				try
				{
					listener.lookupTransform("/map", "/shelf_left_camera/simtrack/" + it, ros::Time(0), transform_map_to_object);
					transform_found = true;
					position = "shelfleft";
					//ROS_INFO_STREAM("Transform /map to /shelf_left_camera/simtrack/" <<it<< " available ...");
				}
				catch (tf::TransformException& ex)
				{
					//ROS_ERROR("%s",ex.what());
					//No transform available
					//ROS_INFO_STREAM("Transform /map to /shelf_left_camera/simtrack/" <<it<< " not available!");
					//return false;
				}
			}

			if (!transform_found)
			{
				try
				{
					listener.lookupTransform("/map", "/omnirob_camera/simtrack/" + it, ros::Time(0), transform_map_to_object);
					transform_found = true; //not in omnirob
					position = "omnirob";
					//ROS_INFO_STREAM("Transform /map to /omnirob_camera/simtrack/" <<it<< " available ...");
				}
				catch (tf::TransformException& ex)
				{
					//ROS_ERROR("%s",ex.what());
					//No transform available
					//ROS_INFO_STREAM("Transform /map to /omnirob_camera/simtrack/" <<it<< " not available!");
					//return false;
				}
			}


			//ask database if object is already there
			database_msgs::get_object getObject;
			getObject.request.name = it;
			clientGetObject.call(getObject);

			//Get robot object from database
			database_msgs::get_object getRobot;
			getRobot.request.name = "omnirob";
			clientGetObject.call(getRobot);
			

			//"transform_map_to_object" has been found (i.e. one of the cameras has detected the object)
			if (transform_found && position != "omnirob")
			{
				//Reset object not seen counter
				object_not_seen_counter[type_counter] = 0;	

				//New object attributes and object already in database			
				Object object, databaseObject;
				database_msgs::update_object srv;
				//Set object to be updated
				srv.request.name = it;

				//Write object pose w.r.t /map frame in neurobots_database
				//Set object pose as Affine3d
				Eigen::Affine3d pose = Eigen::Affine3d::Identity();

				//Converts a tf Transform into an Eigen Affine3d.
				tf::transformTFToEigen(transform_map_to_object, pose);

				//get type of object
				std::string type = object_model_types[type_counter];

				//fill object container
				object["name"].push_back(it);
				object["type"].push_back(type);
				object["perceptible"].push_back(bool(true));
				object["pose"].push_back(pose);
				object["position"].push_back(position);

				//add special attributes
				if (type == "cup")
				{
					object["is-open"].push_back(true);
					object["contains"].push_back(std::string("empty"));
				}
				else if (type == "bottle")
				{
					object["is-open"].push_back(true);
					object["contains"].push_back(std::string("water"));
				}
				else
				{
				
				}
		
				

				if (getObject.response.found)
				{
					database_conversions::convertMsgToObject(getObject.response.object, databaseObject);

					//we send 'send_changes=true' if the position has changed
					if (HAS_ATTRIBUTE(databaseObject, "position"))
					{
						//Old object position from database
						std::string oldPosition = CONVERT_TO_STRING(databaseObject["position"]);

						//Robot object from database
						Object robotObject;
						//Current Robot location from database
						std::string robot_location;
						if (getRobot.response.found)
						{
							//Determine current robot location 
							database_conversions::convertMsgToObject(getRobot.response.object, robotObject);
			
							if (HAS_ATTRIBUTE(robotObject, "at"))
							{
								robot_location = CONVERT_TO_STRING(robotObject["at"]);
							}
							//shouldn't happen
							else
							{
								ROS_INFO_STREAM("Attribute 'at' of omnirob not found!!!");
							}
						}
						else
						{
							//ROS_INFO_STREAM("Object omnirob not found in database!!!");
						}

						//If object has been detected at a new location and the robot is neither moving, i.e. robot_location != nowhere nor at the location where the object
						//has been detected, robot_location != position
						//if (oldPosition != position && oldPosition != "omnirob") 
						if (oldPosition != position && robot_location != position && robot_location != "nowhere")   
						{
							//Set object attributes
							database_conversions::convertObjectToMsg(object, srv.request.object);
							
							//Update object data
							srv.request.send_changes = true;
							clientUpdateObject.call(srv);
							ROS_INFO_STREAM("Object " <<it<< " updated with new position in database");


							//Tell omnirob that it's arm is empty
							if(oldPosition == "omnirob"){
								database_msgs::update_object srv2;
								//Set robot to be updated
								srv2.request.name = "omnirob";

								//Set robot arm empty		
								if(robotObject["arm-empty"].empty())
									robotObject["arm-empty"].push_back(true);
								else
									robotObject["arm-empty"][0] = true;						
								
								srv2.request.send_changes = true;
								database_conversions::convertObjectToMsg(robotObject, srv2.request.object);
								clientUpdateObject.call(srv2);
								ROS_INFO_STREAM("Updated arm-empty attribute of robot!");						
							}

						}
					}
					//shouldn't happen
					else
					{
						ROS_INFO_STREAM("Object " <<it<< " does not have a position attribute!");
						srv.request.send_changes = true;
					}

					
				}
				//unknown object => send changes
				else 
				{
					srv.request.send_changes = true;

					if(!object_already_added[type_counter])
					{
						//Set object attributes
						database_conversions::convertObjectToMsg(object, srv.request.object);

						clientUpdateObject.call(srv);
						
						//Set flag indicating that object has already been added
						object_already_added[type_counter] = true;

						//Flag to see whether object has already been removed
						object_already_removed[type_counter] = false;	
						
						ROS_INFO_STREAM("Object " <<it<< " added to database");
					}
				}

				
			}
			//Object not detected in either camera-> remove it from the database (if it is not located at omnirob)
			else
			{
				//Increment object not seen counter
				object_not_seen_counter[type_counter]++;

				//Delete object from database if it has not been seen for max_object_not_seen_iter
				if(max_object_not_seen_iter < object_not_seen_counter[type_counter])
				{
					//Robot object from database
					Object robotObject;
					std::string robot_location;
					if (getRobot.response.found)
					{
						//Determine current robot location 
						database_conversions::convertMsgToObject(getRobot.response.object, robotObject);
			
						if (HAS_ATTRIBUTE(robotObject, "at"))
						{
							robot_location = CONVERT_TO_STRING(robotObject["at"]);
						}
						//shouldn't happen
						else
						{
							ROS_INFO_STREAM("Attribute 'at' of omnirob not found!!!");
						}
					}
					else
					{
						//ROS_INFO_STREAM("Object omnirob not found in database!!!");
					}


					//Manipulable Object previously added to database
					Object databaseObject;
					if (getObject.response.found)
					{
						database_conversions::convertMsgToObject(getObject.response.object, databaseObject);

						//we send 'send_changes=true' if the position has changed
						if (HAS_ATTRIBUTE(databaseObject, "position"))
						{
							std::string currentPosition = CONVERT_TO_STRING(databaseObject["position"]);
							//Check from database whether object is at an nonvisible location (no camera at that location)							
							if (currentPosition != "omnirob" && currentPosition != "table1" && robot_location != currentPosition)
							{
								//If object has not already been removed
								if(!object_already_removed[type_counter])
								{
									database_msgs::remove_object srv;
									srv.request.name = it;
									clientRemoveObject.call(srv);
									object_already_removed[type_counter] = true;
									//Set flag indicating that object has not been added
									object_already_added[type_counter] = false;

									ROS_INFO_STREAM("Object " <<it<< " removed to database");
								}
							}
						}
						//shouldn't happen
						else
						{
							ROS_INFO_STREAM("Object " <<it<< " does not have a position attribute!");
						}
					}	
						
				}
				
			}

			type_counter++;
		}
		
		ros::spinOnce();

		r.sleep();
	}

	//ros::waitForShutdown();

	return 0;
}



