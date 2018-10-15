/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: robot_interface_sim.cpp
 */


#include <ros/ros.h>
#include <database_msgs/get_object.h>
#include <database_msgs/update_object.h>
#include <database_conversions/database_conversions.h>


using namespace std;


ros::ServiceClient clientGetObject, clientUpdateObject;
ObjectMap objects;

vector<string> world_locs = {"kitchen", "livin-room", "bedroom", "floor", "garden", "dinner-room"};

void getSubjects()
{
    database_msgs::get_object srv_omnirob, srv_human;
    srv_omnirob.request.name = "omnirob";
    if (!clientGetObject.call(srv_omnirob) || !clientGetObject.call(srv_human))
    {
        ROS_ERROR("No omnirob robot / Human present in scene");
    }

    database_conversions::convertMsgToObject(srv_omnirob.response.object, objects["omnirob"]);
    database_conversions::convertMsgToObject(srv_human.response.object, objects["me"]);
}

void init()
{
    //Get Scene Subjects
    getSubjects();

    Object init_loc_robot, init_loc_human;
    init_loc_robot["location"].push_back("kitchen");
    init_loc_human["location"].push_back("living-room");

    database_msgs::update_object srv;
    srv.request.name = "omnirob";
    srv.request.send_changes = true;
    database_conversions::convertObjectToMsg(init_loc_robot, srv.request.object);
    clientUpdateObject.call(srv);

    srv.request.name = "me";
    srv.request.send_changes = true;
    database_conversions::convertObjectToMsg(init_loc_human, srv.request.object);
    clientUpdateObject.call(srv);
}


void updateRobotLocation()
{
    //Random location
    int loc_idx = rand() % world_locs.size();

    Object update_loc_robot;
    update_loc_robot["location"].push_back(world_locs[loc_idx]);

    database_msgs::update_object srv;
    srv.request.name = "omnirob";
    srv.request.send_changes = true;
    database_conversions::convertObjectToMsg(update_loc_robot, srv.request.object);
    clientUpdateObject.call(srv);
}

void updateHumanLocation()
{
    //Random location
    int loc_idx = rand() % world_locs.size();

    Object update_loc_human;
    update_loc_human["location"].push_back(world_locs[loc_idx]);

    database_msgs::update_object srv;
    srv.request.name = "me";
    srv.request.send_changes = true;
    database_conversions::convertObjectToMsg(update_loc_human, srv.request.object);
    clientUpdateObject.call(srv);
}



int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "robot_interface_sim");

    //Node Handle
    ros::NodeHandle nh("neurobots_database");

    clientGetObject = nh.serviceClient<database_msgs::get_object>("get_object");
    clientUpdateObject = nh.serviceClient<database_msgs::update_object>("update_object");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    init();

    ros::Rate r(0.25);

    Object updated_object;

    while (ros::ok())
    {
        //send new object
        if (rand() % 5 == 0)
        {
            updateRobotLocation();

            database_msgs::get_object srv_omnirob;
            srv_omnirob.request.name = "omnirob";
            clientGetObject.call(srv_omnirob);
            database_conversions::convertMsgToObject(srv_omnirob.response.object, updated_object);
            std::cout << "New robot location is "<<CONVERT_TO_STRING(updated_object["location"])<< std::endl;
        }

        //remove object
        if (rand() % 5 == 0)
        {
            updateHumanLocation();

            database_msgs::get_object srv_human;
            srv_human.request.name = "me";
            clientGetObject.call(srv_human);
            database_conversions::convertMsgToObject(srv_human.response.object, updated_object);
            std::cout << "New human location is "<<CONVERT_TO_STRING(updated_object["location"])<< std::endl;
        }

        r.sleep();
    }


    return 0;
}
