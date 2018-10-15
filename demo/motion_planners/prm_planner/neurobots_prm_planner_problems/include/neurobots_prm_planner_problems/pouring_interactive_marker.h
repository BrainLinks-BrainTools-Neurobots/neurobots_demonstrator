/*
 * pouring_interactive_marker.h
 *
 *  Created on: Sep 11, 2016
 *      Author: kuhnerd
 */

#ifndef H13DAAD31_2138_4959_9260_7429E9DE5C85
#define H13DAAD31_2138_4959_9260_7429E9DE5C85

#ifdef FOUND_PRM_PLANNER

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <prm_planner/visualization/interactive_marker.h>
#include <ros/ros.h>

FORWARD_DECLARE_N(prm_planner, PRMPlanner);
FORWARD_DECLARE_N(prm_planner, GraspableObject);

namespace neurobots_prm_planner_problems
{

class PouringInteractiveMarker: public prm_planner::InteractiveMarker
{
public:
	PouringInteractiveMarker(boost::shared_ptr<prm_planner::PRMPlanner> planner,
			boost::shared_ptr<prm_planner::GraspableObject> object);
	virtual ~PouringInteractiveMarker();

//	virtual void update();

private:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void initMarkerServer();
	void initMenu();

private:
	boost::shared_ptr<prm_planner::GraspableObject> m_bottle;

	ros::NodeHandle m_nodeHandle;
	interactive_markers::InteractiveMarkerServer* m_server;
	interactive_markers::MenuHandler m_menuHandler;
	visualization_msgs::InteractiveMarker m_interactiveMarker;

	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPlanToCup;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPlanToInit;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPrintInfo;
};

} /* namespace prm_planner */

#endif

#endif /* H13DAAD31_2138_4959_9260_7429E9DE5C85 */
