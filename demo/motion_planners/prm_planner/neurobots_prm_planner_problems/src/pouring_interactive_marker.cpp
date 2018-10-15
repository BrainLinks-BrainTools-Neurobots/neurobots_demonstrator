/*
 * pouring_interactive_marker.cpp
 *
 *  Created on: Sep 11, 2016
 *      Author: kuhnerd
 */

#ifdef FOUND_PRM_PLANNER

#include <ais_definitions/class.h>
#include <eigen_conversions/eigen_msg.h>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <neurobots_prm_planner_problems/pouring_problem_definition.h>
#include <neurobots_prm_planner_problems/pouring_interactive_marker.h>

namespace neurobots_prm_planner_problems
{

PouringInteractiveMarker::PouringInteractiveMarker(boost::shared_ptr<prm_planner::PRMPlanner> planner,
		boost::shared_ptr<prm_planner::GraspableObject> object) :
				prm_planner::InteractiveMarker(planner),
				m_bottle(object),
				m_nodeHandle("~"),
				m_server(new interactive_markers::InteractiveMarkerServer(m_nodeHandle.getNamespace() + "/goal_pose"))
{
	initMarkerServer();
}

PouringInteractiveMarker::~PouringInteractiveMarker()
{
	DELETE_VAR(m_server);
}

//void PouringInteractiveMarker::update()
//{
//	tf::poseEigenToMsg(m_bottle->getPose(), m_interactiveMarker.pose);
//	m_server->setPose(m_interactiveMarker.name, m_interactiveMarker.pose);
//	m_server->applyChanges();
//}

void PouringInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
	boost::shared_ptr<PouringProblemDefinition> pd = boost::static_pointer_cast<PouringProblemDefinition>(
			prm_planner::ProblemDefinitionManager::getInstance()->getProblemDefinition());

	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			if (feedback->menu_entry_id == m_menuEntryPlanToCup)
			{
				pd->setMode(PouringProblemDefinition::InitToPour);
				m_planner->planAndExecute(Eigen::Affine3d::Identity());
			}
			else if (feedback->menu_entry_id == m_menuEntryPlanToInit)
			{
				pd->setMode(PouringProblemDefinition::PourToInit);
				m_planner->planAndExecute(Eigen::Affine3d::Identity());
			}
			else if (feedback->menu_entry_id == m_menuEntryPrintInfo)
			{
				if (feedback->marker_name == m_bottle->c_params.name)
					m_bottle->print();
			}
	}
}

void PouringInteractiveMarker::initMarkerServer()
{
	// create an interactive marker for our server
	m_interactiveMarker.header.frame_id = m_bottle->c_params.objectFrameName; //we use the name, which is equal to the frame
	m_interactiveMarker.header.stamp = ros::Time(0);
	m_interactiveMarker.name = m_bottle->c_params.name;
	m_interactiveMarker.description = "Object";
	m_interactiveMarker.scale = 1.0;
	tf::poseEigenToMsg(Eigen::Affine3d::Identity(), m_interactiveMarker.pose);

	// create a red sphere marker
	visualization_msgs::Marker objectMarker;
	objectMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
	objectMarker.mesh_resource = "file://" + m_bottle->c_params.meshFilename;
	objectMarker.color = ais_util::Color::red().toROSMsg();
	objectMarker.scale.x = objectMarker.scale.y = objectMarker.scale.z = 1.0;
//	objectMarker.frame_locked = true;

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.name = m_bottle->c_params.name;
	control.orientation.w = 1.0;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
	control.markers.push_back(objectMarker);
	m_interactiveMarker.controls.push_back(control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	m_server->insert(m_interactiveMarker, boost::bind(&PouringInteractiveMarker::processFeedback, this, _1));

	initMenu();

	// 'commit' changes and send to all clients
	m_server->applyChanges();
}

void PouringInteractiveMarker::initMenu()
{
	m_menuEntryPlanToCup = m_menuHandler.insert("Plan & Execute: Init -> Pour", boost::bind(&PouringInteractiveMarker::processFeedback, this, _1));
	m_menuEntryPlanToInit = m_menuHandler.insert("Plan & Execute: Pour -> Init", boost::bind(&PouringInteractiveMarker::processFeedback, this, _1));
	m_menuEntryPrintInfo = m_menuHandler.insert("Print Info", boost::bind(&PouringInteractiveMarker::processFeedback, this, _1));
	m_menuHandler.apply(*m_server, m_interactiveMarker.name);
}

} /* namespace neurobots_prm_planner_problems */

#endif
