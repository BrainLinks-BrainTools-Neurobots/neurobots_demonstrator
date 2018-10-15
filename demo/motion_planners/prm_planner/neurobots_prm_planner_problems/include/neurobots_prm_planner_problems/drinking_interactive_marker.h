/*
 * Copyright (c) 2016 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 23, 2016
 *      Author: kuhnerd
 * 	  Filename: drinking_interactive_marker.h
 */

#ifndef H1A043F48_5F91_4C4A_AF5B_BC5BBF7B1BF6
#define H1A043F48_5F91_4C4A_AF5B_BC5BBF7B1BF6

#ifdef FOUND_PRM_PLANNER

#include <prm_planner/visualization/interactive_marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ais_definitions/class.h>
#include <ros/ros.h>

FORWARD_DECLARE_N(prm_planner, PRMPlanner);
FORWARD_DECLARE_N(prm_planner, GraspableObject);

namespace neurobots_prm_planner_problems
{

class DrinkingInteractiveMarker: public prm_planner::InteractiveMarker
{
public:
	DrinkingInteractiveMarker(boost::shared_ptr<prm_planner::PRMPlanner> planner,
			boost::shared_ptr<prm_planner::GraspableObject> object);
	virtual ~DrinkingInteractiveMarker();

private:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void initMarkerServer();
	void initMenu();

private:
	boost::shared_ptr<prm_planner::GraspableObject> m_cup;

	ros::NodeHandle m_nodeHandle;
	interactive_markers::InteractiveMarkerServer* m_server;
	interactive_markers::MenuHandler m_menuHandler;
	visualization_msgs::InteractiveMarker m_interactiveMarker;

	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPlanToMouth;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPlanToInit;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPrintInfo;
};

} /* namespace neurobots_prm_planner_problems */

#endif

#endif /* H1A043F48_5F91_4C4A_AF5B_BC5BBF7B1BF6 */
