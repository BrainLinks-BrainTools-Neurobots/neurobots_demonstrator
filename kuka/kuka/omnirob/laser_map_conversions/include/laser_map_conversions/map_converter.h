/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: June 25, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: map_converter.h
 */

#ifndef MAP_CONVERTER_H_
#define MAP_CONVERTER_H_


#include <laser_map_conversions/msgs_utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace map_converter
{

class LaserMapConverter
{


	public:
    LaserMapConverter(int8_t obs_threshold, bool unknown_obstacles, double extrusion , int padding = 0, std::string planning_scene_prefix = "");
	~LaserMapConverter();

    //To query whether map has been generated
    bool is_map_generated(){return m_map_received;}

    //To publish the generated octomap
    bool publish_extruded_map(std::string octomap_ref_frame);

	
	private:
	
	//Node Handle
    ros::NodeHandle m_nh;
	
	int m_obs_threshold; //probability of cell (being an obstacle) 
	bool m_unknown_obstacles;
	double m_desired_extrusion;
    int m_map_padding; //in number of cells
	//Map converted flag
	bool m_map_received;
	
	moveit_msgs::OrientedBoundingBox get_bbx(const moveit_msgs::CollisionObject& collision_map);
	moveit_msgs::CollisionObject extrude_to_collision_map(const nav_msgs::OccupancyGrid& grid, double extrusion);
    moveit_msgs::CollisionObject extrude_to_collision_map_padded(const nav_msgs::OccupancyGrid& grid, double extrusion);
	void convert(const moveit_msgs::CollisionObject& cmap, octomap::Pointcloud& octomap_cloud);
	void log_octomap(const octomap::OcTree& octree);
	bool extrude(const nav_msgs::OccupancyGrid& grid, double extrusion, octomap_msgs::Octomap& map);

    //Map Callback
    void map_callback(const nav_msgs::OccupancyGridConstPtr& map_grid);

	
	//Resulting Octomap
	octomap_msgs::Octomap m_map_octo;

	
	//OccupancyGrid subscriber and Octomap publisher 
	ros::Subscriber m_map_sub;
	ros::Publisher  m_octomap_pub;

    //Planning Scene Publisher (for adding octomap to the planning scene)
    ros::Publisher m_ps_octo_pub;
	
	
	
	

};

} /* namespace neurobots_database */

#endif /* NEUROBOTS_DATABASE_DATABASE_H_ */

