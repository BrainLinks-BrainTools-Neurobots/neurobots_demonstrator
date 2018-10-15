/*
 * Copyright (c) 2016 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 23, 2016
 *      Author: kuhnerd
 * 	  Filename: drinking_interactive_marker.cpp
 */

#ifdef FOUND_PRM_PLANNER

#include <ais_definitions/class.h>
#include <eigen_conversions/eigen_msg.h>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <neurobots_prm_planner_problems/drinking_interactive_marker.h>
#include <neurobots_prm_planner_problems/drinking_problem_definition.h>

namespace neurobots_prm_planner_problems
{

DrinkingInteractiveMarker::DrinkingInteractiveMarker(boost::shared_ptr<prm_planner::PRMPlanner> planner,
		boost::shared_ptr<prm_planner::GraspableObject> object) :
				prm_planner::InteractiveMarker(planner),
				m_cup(object),
				m_nodeHandle("~"),
				m_server(new interactive_markers::InteractiveMarkerServer(m_nodeHandle.getNamespace() + "/goal_pose"))
{
	initMarkerServer();
}

DrinkingInteractiveMarker::~DrinkingInteractiveMarker()
{
	DELETE_VAR(m_server);
}

void DrinkingInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
	boost::shared_ptr<DrinkingProblemDefinition> pd = boost::static_pointer_cast<DrinkingProblemDefinition>(
			prm_planner::ProblemDefinitionManager::getInstance()->getProblemDefinition());

	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			if (feedback->menu_entry_id == m_menuEntryPlanToMouth)
			{
				pd->setMode(DrinkingProblemDefinition::InitToMouth);
				m_planner->planAndExecute(Eigen::Affine3d::Identity());
			}
			else if (feedback->menu_entry_id == m_menuEntryPlanToInit)
			{
				pd->setMode(DrinkingProblemDefinition::MouthToInit);
				m_planner->planAndExecute(Eigen::Affine3d::Identity());
			}
			else if (feedback->menu_entry_id == m_menuEntryPrintInfo)
			{
				if (feedback->marker_name == m_cup->c_params.name)
					m_cup->print();
			}
	}
}

void DrinkingInteractiveMarker::initMarkerServer()
{
	// create an interactive marker for our server
	m_interactiveMarker.header.frame_id = m_cup->c_params.objectFrameName; //we use the name, which is equal to the frame
	m_interactiveMarker.header.stamp = ros::Time(0);
	m_interactiveMarker.name = m_cup->c_params.name;
	m_interactiveMarker.description = "Object";
	m_interactiveMarker.scale = 1.0;
	tf::poseEigenToMsg(Eigen::Affine3d::Identity(), m_interactiveMarker.pose);

	// create a red sphere marker
	visualization_msgs::Marker objectMarker;
	objectMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
	objectMarker.mesh_resource = "file://" + m_cup->c_params.meshFilename;
	objectMarker.color = ais_util::Color::red().toROSMsg();
	objectMarker.scale.x = objectMarker.scale.y = objectMarker.scale.z = 1.0;
	//	objectMarker.frame_locked = true;

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.name = m_cup->c_params.name;
	control.orientation.w = 1.0;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
	control.markers.push_back(objectMarker);
	m_interactiveMarker.controls.push_back(control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	m_server->insert(m_interactiveMarker, boost::bind(&DrinkingInteractiveMarker::processFeedback, this, _1));

	initMenu();

	// 'commit' changes and send to all clients
	m_server->applyChanges();
}

void DrinkingInteractiveMarker::initMenu()
{
	m_menuEntryPlanToMouth = m_menuHandler.insert("Plan & Execute: Init -> Mouth", boost::bind(&DrinkingInteractiveMarker::processFeedback, this, _1));
	m_menuEntryPlanToInit = m_menuHandler.insert("Plan & Execute: Mouth -> Init", boost::bind(&DrinkingInteractiveMarker::processFeedback, this, _1));
	m_menuEntryPrintInfo = m_menuHandler.insert("Print Info", boost::bind(&DrinkingInteractiveMarker::processFeedback, this, _1));
	m_menuHandler.apply(*m_server, m_interactiveMarker.name);
}

} /* namespace neurobots_prm_planner_problems */

#endif
