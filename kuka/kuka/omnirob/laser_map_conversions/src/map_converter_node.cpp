/*
 *map_converter.cpp
 *
 *  Created on: June 26, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <laser_map_conversions/map_converter.h>

#include <iostream>
#include <fstream>


//--------------------- ROS Node ---------------------------


int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "map_converter_node");

    //Node Handle
    ros::NodeHandle nh;

    //probability of cell (being an obstacle)
    double obs_threshold = 50.0;
    bool unknown_obstacles = false;
    double desired_extrusion = 1.0; //in [m]
    double map_padding = 2; //in number of cells
    std::string planning_scene_prefix = "";

    //Get desired map extrusion from arg input
    if(argc > 1){
        std::stringstream s(argv[1]);
        s >> desired_extrusion;
    }
    //Get desired map extrusion from arg input
    if(argc > 2){
        std::stringstream s(argv[2]);
        s >> planning_scene_prefix;
    }


    //Reference frame for octomap
    std::string octomap_ref_frame = "map";

    //Converts nav_msgs::OccupancyGrid to octomap_msgs::Octomap
    map_converter::LaserMapConverter lmc(obs_threshold,unknown_obstacles,desired_extrusion,map_padding,planning_scene_prefix);

    ros::Rate rate(1);
    int counter = 0;
    while(ros::ok()){

        //Publish the generated Octomap on topic "/octomap_demo"
        bool success = lmc.publish_extruded_map(octomap_ref_frame);

        //Spin to receive callbacks
        ros::spinOnce();

        if(success)
        {
            if(counter == 6)
             break;

            counter++;
        }

        rate.sleep();

    }


    //Shutdown node
    ros::shutdown();

    return 0;
}

