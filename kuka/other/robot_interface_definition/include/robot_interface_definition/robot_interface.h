/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 9, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_interface.h
 */

#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_
#include <Eigen/Geometry>
#include <vector>

namespace robot_interface_definition
{

class RobotInterface
{
public:
	RobotInterface()
	{
	}

	virtual ~RobotInterface()
	{
	}

	/**
	 * Implement this method in your planner.
	 * The frame of 'goal' is 'world'
	 */
	virtual bool plan(const Eigen::Affine3d& goal)
	{
		return true;
	}

	/**
	 * Can be used for, e.g., pouring
	 */
	virtual bool plan(const std::vector<std::string>& params)
	{
		return true;
	}

	virtual bool execute()
	{
		return true;
	}

	virtual bool grasp(const std::string& object)
	{
		return true;
	}

	virtual bool drop(const std::string& object,
			const Eigen::Affine3d& pose)
	{
		return true;
	}

	//Only used to activate/deactivate the prm planners
	virtual bool activate()
	{
		return true;
	}

	virtual bool deactivate()
	{
		return true;
	}
};

} /* namespace robot_interface */

#endif /* ROBOT_INTERFACE_H_ */
