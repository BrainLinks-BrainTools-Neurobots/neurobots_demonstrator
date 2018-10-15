/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Mar 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: constraint_factory.h
 */

#ifndef CONSTRAINT_FACTORY_H_
#define CONSTRAINT_FACTORY_H_
#include <moveit/move_group_interface/move_group.h>

namespace omnirob_planning_interface
{

class ConstraintFactory
{
public:
	ConstraintFactory();
	virtual ~ConstraintFactory();

	static void createHorizontalConstraint(moveit_msgs::OrientationConstraint& constraint);
	static void createDownwardConstraint(moveit_msgs::OrientationConstraint& constraint);
};

} /* namespace omnirob_planning_interface */

#endif /* CONSTRAINT_FACTORY_H_ */
