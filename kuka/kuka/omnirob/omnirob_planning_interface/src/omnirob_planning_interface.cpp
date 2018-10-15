/*
 * omnirob_planning_interface.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: kuhnerd
 */

#include <omnirob_planning_interface/omnirob_planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

const std::string c_controllerTopic = "/omnirob_lbr/follow_joint_trajectory";

namespace omnirob_planning_interface
{

OmnirobPlanningInterface::OmnirobPlanningInterface(const std::string& planningFrame,
		const std::string& endEffectorFrame,
		const std::string& planningGroup,
		const double maxPlanningTime,
		const int maxPlanningAttempts) :
				m_privateNodeHandle("~"),
				m_planningFrame(planningFrame),
				m_endEffectorFrame(endEffectorFrame),
				m_planningGroup(planningGroup),
				m_maxPlanningTime(maxPlanningTime),
				m_maxPlanningAttempts(maxPlanningAttempts),
				m_robotModelLoader("robot_description"),
				m_robotModel(m_robotModelLoader.getModel()),
				m_planningScene(new planning_scene::PlanningScene(m_robotModel)),
				m_planningPipeline(NULL),
				m_controllerConnected(false),
				m_planner(RRTConnect)
{
	ros::NodeHandle n("move_group");

	ROS_INFO("setting up publishers and service clients");
	m_pubTrajectory = m_privateNodeHandle.advertise<moveit_msgs::DisplayTrajectory>("planned_path", 1, true);
	m_pubPlanningScene = m_privateNodeHandle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
	m_serviceGetPlanningScene = m_privateNodeHandle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

	ROS_INFO("setting up planning pipeline");
	m_planningPipeline = new planning_pipeline::PlanningPipeline(m_robotModel, n,
			"planning_plugin", "request_adapters");

	ROS_INFO("setting up controller action client");
	m_controller.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(c_controllerTopic, false));

	ROS_INFO("waiting for action server");
	if (m_controller->waitForServer(ros::Duration(2.0)))
	{
		ROS_INFO("connected!");
		m_controllerConnected = true;
	}
	else
	{
		ROS_INFO("No controller was found on action topic %s", c_controllerTopic.c_str());
	}
}

OmnirobPlanningInterface::~OmnirobPlanningInterface()
{
	boost::mutex::scoped_lock lock(m_deleteMutex);
	if (m_planningPipeline != NULL)
	{
		delete m_planningPipeline;
		m_planningPipeline = NULL;
	}
}

bool OmnirobPlanningInterface::plan(const Eigen::Affine3d& goal,
		bool allowCollisions,
		Plan& result)
{
	std::vector<moveit_msgs::OrientationConstraint> empty;
	return planTrajectory(NULL, goal, empty, allowCollisions, result);
}

bool OmnirobPlanningInterface::plan(const moveit_msgs::RobotState& start,
		const Eigen::Affine3d& goal,
		bool allowCollisions,
		Plan& result)
{
	std::vector<moveit_msgs::OrientationConstraint> empty;
	return planTrajectory(&start, goal, empty, allowCollisions, result);
}

bool OmnirobPlanningInterface::plan(const Plan& other,
		const Eigen::Affine3d& goal,
		bool allowCollisions,
		Plan& result)
{
	std::vector<moveit_msgs::OrientationConstraint> empty;
	return plan(other, goal, empty, allowCollisions, result);
}

bool OmnirobPlanningInterface::plan(const Eigen::Affine3d& goal,
		std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
		bool allowCollisions, //between hand and environment
		Plan& result)
{
	return planTrajectory(NULL, goal, trajectoryConstraints, allowCollisions, result);
}

bool OmnirobPlanningInterface::plan(const moveit_msgs::RobotState& start,
		const Eigen::Affine3d& goal,
		std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
		bool allowCollisions, //between hand and environment
		Plan& result)
{
	return planTrajectory(&start, goal, trajectoryConstraints, allowCollisions, result);
}

bool OmnirobPlanningInterface::plan(const Plan& other,
		const Eigen::Affine3d& goal,
		std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
		bool allowCollisions, //between hand and environment
		Plan& result)
{
	moveit_msgs::RobotState state;
	Plan resultNew;

	other.getLastWaypoint(state);
	bool success = planTrajectory(&state, goal, trajectoryConstraints, allowCollisions, resultNew);

	if (!success)
	{
		return false;
	}

	result = other;
	result.appendPlan(resultNew);
	return true;
}

bool OmnirobPlanningInterface::execute(const Plan& plan)
{
	if (m_controllerConnected)
	{
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = plan.m_plan.trajectory.joint_trajectory;
		m_controller->sendGoal(goal);
		if (!m_controller->waitForResult())
		{
			control_msgs::FollowJointTrajectoryResultConstPtr result = m_controller->getResult();
			ROS_ERROR("%s", result->error_string.c_str());
			return false;
		}
	}

	return true;
}

bool OmnirobPlanningInterface::move(const Eigen::Affine3d& goal,
		bool allowCollisions)
{
	Plan result;
	return plan(goal, allowCollisions, result) && execute(result);
}

bool OmnirobPlanningInterface::move(const moveit_msgs::RobotState& start,
		const Eigen::Affine3d& goal,
		bool allowCollisions)
{
	Plan result;
	return plan(start, goal, allowCollisions, result) && execute(result);
}

bool OmnirobPlanningInterface::updatePlanningScene()
{
	static bool updated = false;
	if (!updated)
	{
		moveit_msgs::GetPlanningScene srv;
		srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
				+ moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING
				+ moveit_msgs::PlanningSceneComponents::OCTOMAP
				+ moveit_msgs::PlanningSceneComponents::ROBOT_STATE
				+ moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
				+ moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
				+ moveit_msgs::PlanningSceneComponents::TRANSFORMS
				+ moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
				+ moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

		if (!m_serviceGetPlanningScene.call(srv))
		{
			ROS_ERROR("cannot get planning scene!");
			return false;
		}

		updated = true;

		m_planningScene->setPlanningSceneMsg(srv.response.scene);
//
//		if (srv.response.scene.world.octomap.octomap.data.empty())
//		{
//			ROS_INFO("empty octomap");
//			updated = false;
//		}

		return true;
	}
	else
	{
		moveit_msgs::GetPlanningScene srv;
		srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
				+ moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING
				+ moveit_msgs::PlanningSceneComponents::ROBOT_STATE
				+ moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
				+ moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
				+ moveit_msgs::PlanningSceneComponents::TRANSFORMS
				+ moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
				+ moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

		if (!m_serviceGetPlanningScene.call(srv))
		{
			ROS_ERROR("cannot get planning scene!");
			return false;
		}

		updated = true;

		m_planningScene->setPlanningSceneDiffMsg(srv.response.scene);
		return true;

		ROS_INFO("Octomap of planning scene was not updated!!!!!!!!!!!!!!!!!!!!!!! omnirob_Planning_interface.cpp, 161");
		return true;
	}
}

void OmnirobPlanningInterface::setCollisionsAllowed()
{
	collision_detection::AllowedCollisionMatrix& acm = m_planningScene->getAllowedCollisionMatrixNonConst();
	acm.setEntry("<octomap>", true);
}

void OmnirobPlanningInterface::setCollisionsDisallowed()
{
	collision_detection::AllowedCollisionMatrix& acm = m_planningScene->getAllowedCollisionMatrixNonConst();
	acm.setEntry("<octomap>", false);
}

void OmnirobPlanningInterface::visualizePlan(const Plan& plan)
{
	moveit_msgs::DisplayTrajectory displayTrajectory;
	displayTrajectory.trajectory_start = plan.m_plan.trajectory_start;
	displayTrajectory.trajectory.push_back(plan.m_plan.trajectory);
	displayTrajectory.trajectory.back().joint_trajectory.header.stamp = ros::Time::now();
	m_pubTrajectory.publish(displayTrajectory);
}

bool OmnirobPlanningInterface::planTrajectory(const moveit_msgs::RobotState* start,
		const Eigen::Affine3d& goal,
		std::vector<moveit_msgs::OrientationConstraint>& trajectoryConstraints,
		bool allowCollisions, //between hand and environment
		Plan& result)
{
	if (!updatePlanningScene())
	{
		return false;
	}

	if (allowCollisions)
	{
		setCollisionsAllowed();
	}
	else
	{
		setCollisionsDisallowed();
	}

	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	Eigen::Vector3d pos = goal.translation();
	Eigen::Quaterniond rot = Eigen::Quaterniond(goal.rotation());

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = m_planningFrame;
	pose.pose.position.x = pos.x();
	pose.pose.position.y = pos.y();
	pose.pose.position.z = pos.z();
	pose.pose.orientation.x = rot.x();
	pose.pose.orientation.y = rot.y();
	pose.pose.orientation.z = rot.z();
	pose.pose.orientation.w = rot.w();

	moveit_msgs::Constraints poseGoal = kinematic_constraints::constructGoalConstraints(m_endEffectorFrame, pose, 0.01, 0.01);
	req.group_name = m_planningGroup;

	//goal and start
	req.goal_constraints.push_back(poseGoal);

	if (start != NULL)
	{
		req.start_state = *start;
	}

	req.allowed_planning_time = m_maxPlanningTime;
	req.num_planning_attempts = m_maxPlanningAttempts;
	req.workspace_parameters.header.frame_id = m_planningFrame;
	req.workspace_parameters.max_corner.x = 1.8;
	req.workspace_parameters.max_corner.y = 1.8;
	req.workspace_parameters.max_corner.z = 2;
	req.workspace_parameters.min_corner.x = -1.8;
	req.workspace_parameters.min_corner.y = -1.9;
	req.workspace_parameters.min_corner.z = -0.5;
	req.planner_id = getPlannerString();

	//trajectory constraints
	if (trajectoryConstraints.size() > 0)
	{
		req.path_constraints.orientation_constraints = trajectoryConstraints;
		moveit_msgs::Constraints c;
		c.orientation_constraints = trajectoryConstraints;
		req.trajectory_constraints.constraints.push_back(c);
	}

	m_planningPipeline->generatePlan(m_planningScene, req, res);

	if (m_pubPlanningScene.getNumSubscribers() > 0)
	{
		moveit_msgs::PlanningScene msg;
		m_planningScene->getPlanningSceneMsg(msg);
		m_pubPlanningScene.publish(msg);
	}

	/* Check that the planning was successful */
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully: %d", res.error_code_.val);
		return false;
	}

//	m_iptp.computeTimeStamps(*res.trajectory_);

	res.getMessage(result.m_plan);

	return true;
}

std::string OmnirobPlanningInterface::getPlannerString()
{
	switch (m_planner)
	{
		case RRT:
			return "RRTkConfigDefault";
		case RRTConnect:
			return "RRTConnectkConfigDefault";
		case RRTStar:
			return "RRTstarkConfigDefault";
		default:
			ROS_ERROR("Unknown planner!");
			return "unknown";
	}
}

Planner OmnirobPlanningInterface::getPlanner() const
{
	return m_planner;
}

void OmnirobPlanningInterface::setPlanner(Planner planner)
{
	m_planner = planner;
}

bool OmnirobPlanningInterface::isStateInCollision(const moveit_msgs::RobotState& state)
{
	return m_planningScene->isStateColliding(state, m_planningGroup, false);
}

} /* namespace omnirob_planning_interface */

