/*
 * planner.cpp
 *
 *  Created on: Jan 3, 2016
 *      Author: daniel
 */

#include <boost/smart_ptr/shared_ptr.hpp>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <prm_planner/planners/path_planner.h>
#include <prm_planner/planners/prm/prm_node.h>

#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/environment/planning_scene.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameters.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner_robot/defines.h>
#include <random>

namespace prm_planner
{

PathPlanner::PathPlanner(boost::shared_ptr<ProblemDefinition> pd) :
				m_robot(pd->getRobot()),
				m_pd(pd),
				m_stopAllThreads(false)
{
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::update(const boost::shared_ptr<PlanningScene>& planningScene)
{
	PLANNER_WRITE_LOCK();

	m_planningScene = planningScene;
}

void PathPlanner::publish()
{
}

void PathPlanner::reset()
{
}

const boost::shared_ptr<ProblemDefinition> PathPlanner::getProblemDefinition() const
{
	return m_pd;
}

bool PathPlanner::plan(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const Eigen::Affine3d& goalTaskPose,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path,
		bool directConnectionRequired)
{
	if (ParameterServer::oneImageMode == parameters::OneImagePerPlan)
	{
		ParameterServer::setReceiveImage();
		ros::Rate r(1000);
		while (!ParameterServer::hasImageReceived())
			r.sleep();
	}

	//check collisions in current state
	if (cd.get() != NULL)
	{
		int i = 0;
		fcl_robot_model::RobotState state;
		for (auto& it2 : m_robot->getChainJointNames())
		{
			state[it2] = currentJointPose(i++);
		}
		cd->robot->setRobotState(state);

		if (cd->fcl->checkCollisions())
		{
			fcl_collision_detection::FCLWrapper::CollisionsVector col;
			cd->fcl->getCollisions(col);
			LOG_ERROR("Found collisions in start state between: ");
			for (auto& it : col)
			{
				LOG_INFO("   - " << it.first << " " << it.second);
			}

			return false;
		}
	}

	return true;
}

bool PathPlanner::planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const int startNodeId,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path)
{
	if (ParameterServer::oneImageMode == parameters::OneImagePerPlan)
	{
		ParameterServer::setReceiveImage();
		ros::Rate r(1000);
		while (!ParameterServer::hasImageReceived())
			r.sleep();
	}

	return true;
}

void PathPlanner::stopAllThreads()
{
	PLANNER_WRITE_LOCK();
	if (!m_stopAllThreads)
	m_stopAllThreads = true;
}

void PathPlanner::resetStopAllThreads()
{
	PLANNER_WRITE_LOCK();
	m_stopAllThreads = false;
}

}
/* namespace prm_planner */

