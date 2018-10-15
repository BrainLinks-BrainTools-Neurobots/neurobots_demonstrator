/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Mar 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: constraint_factory.cpp
 */

#include <omnirob_planning_interface/constraint_factory.h>

namespace omnirob_planning_interface
{

ConstraintFactory::ConstraintFactory()
{
}

ConstraintFactory::~ConstraintFactory()
{
}

void ConstraintFactory::createHorizontalConstraint(moveit_msgs::OrientationConstraint& constraint)
{
	constraint.link_name = "lbr_7_link";
	constraint.header.stamp = ros::Time::now();
	constraint.header.frame_id = "base_link";
	constraint.absolute_x_axis_tolerance = 0.02;
	constraint.absolute_y_axis_tolerance = M_PI;
	constraint.absolute_z_axis_tolerance = 0.02;
	constraint.orientation.x = -0.707107;
	constraint.orientation.y = 0;
	constraint.orientation.z = 0;
	constraint.orientation.w = 0.707107;
	constraint.weight = 1.0;
}

void ConstraintFactory::createDownwardConstraint(moveit_msgs::OrientationConstraint& constraint)
{
	constraint.link_name = "lbr_7_link";
	constraint.header.stamp = ros::Time::now();
	constraint.header.frame_id = "base_link";
	constraint.absolute_x_axis_tolerance = 0.02;
	constraint.absolute_y_axis_tolerance = 0.02;
	constraint.absolute_z_axis_tolerance = M_PI;
	constraint.orientation.x = 0.707107;
	constraint.orientation.y = 0.707107;
	constraint.orientation.z = 0;
	constraint.orientation.w = 0;
	constraint.weight = 1.0;
}

} /* namespace omnirob_planning_interface */
