/*
 * pouring_problem_definition.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: kuhnerd
 */
#include <omp.h>
#include <prm_planner/robot/feasibility_checker.h>

#ifdef FOUND_PRM_PLANNER

#include <prm_planner/util/parameter_server.h>
#include <ais_ros/ros_base_interface.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/environment/planning_scene.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/planners/path_planner.h>
#include <prm_planner/planners/prm/prm.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/visualization/interactive_marker.h>
#include <prm_planner_constraints/constraint.h>
#include <neurobots_prm_planner_problems/pouring_problem_definition.h>
#include <neurobots_prm_planner_problems/pouring_interactive_marker.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <prm_planner/planners/prm/prma_star.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/robot_arm.h>
#include <ais_util/progress_bar.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <random>

using namespace prm_planner;

namespace neurobots_prm_planner_problems
{

PouringProblemDefinition::PouringProblemDefinition() :
				m_mode(InitToPour),
				m_tcp(Eigen::Affine3d::Identity()),
				m_prm(NULL),
				m_prmInitialized(false)
{
}

PouringProblemDefinition::~PouringProblemDefinition()
{
}

void PouringProblemDefinition::init(const boost::shared_ptr<prm_planner::Robot> robot,
		const boost::shared_ptr<prm_planner::Constraint> constraint,
		const boost::shared_ptr<prm_planner::PRMPlanner> planner,
		const prm_planner::parameters::ProblemConfig& config)
{
	ApproachingProblemDefinition::init(robot, constraint, planner, config);

	m_mode = InitToPour;
	m_tcp = Eigen::Affine3d::Identity();

	m_robotName = m_robot->getName();

	//get objects
	auto& objects = ObjectManager::getInstance()->getObjects();
	std::string bottleName = "";
	std::string cupName = "";
	for (auto& it : objects)
	{
		if (boost::contains(it.first, "bottle"))
		{
			bottleName = it.first;
			break;
		}
	}

	for (auto& it : objects)
	{
		if (boost::contains(it.first, "cup"))
		{
			cupName = it.first;
			break;
		}
	}

	if (bottleName.empty() || cupName.empty())
	{
		LOG_FATAL("Please add objects with names that contains 'bottle' and 'cup' respectively to the problem definition!");
		exit(-1);
	}

	LOG_INFO("Using " << bottleName << " as bottle");
	LOG_INFO("Using " << cupName << " as cup");

	m_bottle = ObjectManager::getInstance()->getObject(bottleName);
	m_cup = ObjectManager::getInstance()->getObject(cupName);

	ros::NodeHandle n("problem_definitions/planning/pouring");
	if (!n.getParam("slow_pouring", m_slowPouring))
	{
		m_slowPouring = false;
	}
}

void PouringProblemDefinition::initPlanner()
{
	if (c_config.plannerType != "prm_a_star")
	{
		LOG_ERROR("This problem definition needs to use the PRMA* planner!");
	}

	//right now, we initialize the planner with a NULL roadmap.
	//it will be initialized/updated in the planning step
	ApproachingProblemDefinition::initPlanner();

	m_prmPlanner = boost::dynamic_pointer_cast<PRMAStar>(m_planner);
}

void PouringProblemDefinition::update(const boost::shared_ptr<prm_planner::PlanningScene>& planningScene)
{
	ApproachingProblemDefinition::update(planningScene);

	//initialize the PRM here (mainly for visualization, since
	//every plan() call will generate a new PRM). We cannot generate
	//it in the init method, because ROS transformations are not
	//available (they are initialized later).
	if (!m_prmInitialized)
	{
		PRM* prm = createPouringPRM();

		if (prm == NULL)
			return;

		PLANNER_WRITE_LOCK();
		m_prm = prm;
		m_prmInitialized = true;

		//set the prm, otherwise we need to delete it manually
		//before creating a new PRM (planner takes ownership)
		m_prmPlanner->setPRM(m_prm);
	}
}

prm_planner::PRM* PouringProblemDefinition::createPouringPRM()
{
//	LOG_DEBUG("Creating new roadmap...");

	const double cupHeight = m_cup->getHeight();
	const double cupRadius = m_cup->getRadius();
	const double bottleHeight = m_bottle->getHeight();
	const double bottleRadius = m_bottle->getRadius();

	const double distBetweenCupAndBottlePrePosition = 0.1;

	boost::shared_ptr<ProblemDefinition> pd = shared_from_this();

	std::unordered_map<int, PRMNode*> nodes;
	std::unordered_map<int, PRMEdge*> edges;

	static std::uniform_real_distribution<double> dist(-M_PI, M_PI);
	static std::random_device gen;

	boost::atomic_int counterNodes(0), counterEdges(0);
	KDL::JntArray joints;

	Eigen::Affine3d toPlanningFrame;
//	LOG_INFO(m_cup->c_params.objectFrameName);
//	LOG_INFO(c_config.planningFrame);

//	ros::Rate r(10);
//	while (!ais_ros::RosBaseInterface::getRosTransformationWithResult(m_cup->c_params.objectFrameName, c_config.planningFrame, toPlanningFrame)
//			&& ros::ok())
//	{
////		LOG_ERROR("Cup not found!");
//		r.sleep();
//	}

	if (!ais_ros::RosBaseInterface::getRosTransformationWithResult(m_cup->c_params.objectFrameName, c_config.planningFrame, toPlanningFrame)
			&& ros::ok())
	{
		return NULL;
	}

//	LOG_INFO(m_cup->c_params.objectFrameName);
//	LOG_INFO(c_config.planningFrame);
//	LOG_INFO(toPlanningFrame.matrix());

	if (ParameterServer::connectedToVRep)
	{
		//compute trajectory in cup frame
		//we assume z is the upward direction of the cup and bottle
		for (int dir = -1; dir <= 1; dir += 2)
			for (int i = 0; i < 50; ++i)
			{
				Eigen::AngleAxisd rot(dist(gen), Eigen::Vector3d::UnitZ());
//				Eigen::AngleAxisd rot(0, Eigen::Vector3d::UnitZ());

				Eigen::Affine3d startPose = Eigen::Affine3d::Identity();
				startPose.translation() = Eigen::Vector3d(0, dir * (cupRadius + distBetweenCupAndBottlePrePosition + bottleRadius),
						bottleHeight + 0.05);

				Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
				pose1.linear() = Eigen::AngleAxisd(dir * M_PI_4, Eigen::Vector3d::UnitX()).matrix();
				pose1.translation() = Eigen::Vector3d(0, dir * cupRadius, cupHeight + 0.05);

				Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
				pose2.linear() = Eigen::AngleAxisd(dir * M_PI / 180 * 80, Eigen::Vector3d::UnitX()).matrix();
				pose2.translation() = Eigen::Vector3d(0, dir * (cupRadius - 0.01), cupHeight + 0.03);

				Eigen::Affine3d poseGoal = Eigen::Affine3d::Identity();
				poseGoal.linear() = Eigen::AngleAxisd(dir * M_PI / 180 * 130, Eigen::Vector3d::UnitX()).matrix();
				poseGoal.translation() = Eigen::Vector3d(0, dir * (cupRadius - 0.01), cupHeight + 0.01);

				//rotate randomly
				startPose = toPlanningFrame * rot * startPose;
				pose1 = toPlanningFrame * rot * pose1;
				pose2 = toPlanningFrame * rot * pose2;
				poseGoal = toPlanningFrame * rot * poseGoal;

				PRMNode* nodeStart = new PRMNode(startPose, counterNodes, PRMNode::StartNode);
				PRMNode* node1 = new PRMNode(pose1, counterNodes);
				PRMNode* node2 = new PRMNode(pose2, counterNodes);
				PRMNode* nodeGoal = new PRMNode(poseGoal, counterNodes, PRMNode::GoalNode);

				nodes[nodeStart->getId()] = nodeStart;
				nodes[node1->getId()] = node1;
				nodes[node2->getId()] = node2;
				nodes[nodeGoal->getId()] = nodeGoal;

				PRMEdge* edge1 = new PRMEdge(nodeStart, node1, counterEdges);
				PRMEdge* edge2 = new PRMEdge(node1, node2, counterEdges);
				PRMEdge* edgeGoal = new PRMEdge(node2, nodeGoal, counterEdges);

				nodeStart->addEdge(edge1);
				node1->addEdge(edge1);
				node1->addEdge(edge2);
				node2->addEdge(edge2);
				node2->addEdge(edgeGoal);
				nodeGoal->addEdge(edgeGoal);

				edges[edge1->getId()] = edge1;
				edges[edge2->getId()] = edge2;
				edges[edgeGoal->getId()] = edgeGoal;
			}
	}
	else
	{
		//compute trajectory in cup frame
		//we assume y is the upward direction of the cup and bottle

//		double angle1 = M_PI / 180 * 30;
//		double angle2 = M_PI / 180 * 60;
//		double angle3 = M_PI / 180 * 95;
//		double heightAboveTableWP1 = 0.05;
//		double heightOffset1 = 0.09;
//		double heightOffset2 = 0.07;
//		double heightOffset3 = 0.06;
//		double radiusOffset1 = 0.02;
//		double radiusOffset2 = -0.01;
//		double radiusOffset3 = -0.03;

//		double angle1 = M_PI / 180 * 30;
//		double angle2 = M_PI / 180 * 80;
//		double angle3 = M_PI / 180 * 120;
//		double heightAboveTableWP1 = 0.05;
//		double heightOffset1 = 0.01;
//		double heightOffset2 = 0.005;
//		double heightOffset3 = -0.005;
//		double radiusOffset1 = 0.00;
//		double radiusOffset2 = 0.00;
//		double radiusOffset3 = 0.00;

		double angle1 = M_PI / 180 * 30;
		double angle2 = M_PI / 180 * 60;
		double angle3 = M_PI / 180 * 110;
		double heightAboveTableWP1 = 0.05;
		double heightOffset1 = 0.075; //0.05
		double heightOffset2 = 0.065; //0.04
		double heightOffset3 = 0.065; //0.04
		double radiusOffset1 = 0.06;
		double radiusOffset2 = 0.01;
		double radiusOffset3 = -0.01;

		for (int dir = -1; dir <= 1; dir += 2)
			for (int i = 0; i < 15; ++i)
			{
				Eigen::AngleAxisd rot(dist(gen), Eigen::Vector3d::UnitY());
//				Eigen::AngleAxisd rot(0, Eigen::Vector3d::UnitY());

				Eigen::Matrix3d baseRotation;
				baseRotation.col(0) = Eigen::Vector3d(1, 0, 0);
				baseRotation.col(1) = Eigen::Vector3d(0, 0, -1);
				baseRotation.col(2) = Eigen::Vector3d(0, 1, 0);

				Eigen::Affine3d startPose = Eigen::Affine3d::Identity();
				startPose.linear() = baseRotation;
				startPose.translation() = Eigen::Vector3d(0, bottleHeight + 0.05,
						dir * (cupRadius + distBetweenCupAndBottlePrePosition + bottleRadius));

				Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
				pose1.linear() = baseRotation * Eigen::AngleAxisd(-dir * angle1, Eigen::Vector3d::UnitX()).matrix();
				pose1.translation() = Eigen::Vector3d(0, cupHeight + heightOffset1, dir * (cupRadius + radiusOffset1));

				Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
				pose2.linear() = baseRotation * Eigen::AngleAxisd(-dir * angle2, Eigen::Vector3d::UnitX()).matrix();
				pose2.translation() = Eigen::Vector3d(0, cupHeight + heightOffset2, dir * (cupRadius + radiusOffset2));

				Eigen::Affine3d poseGoal = Eigen::Affine3d::Identity();
				poseGoal.linear() = baseRotation * Eigen::AngleAxisd(-dir * angle3, Eigen::Vector3d::UnitX()).matrix();
				poseGoal.translation() = Eigen::Vector3d(0, cupHeight + heightOffset3, dir * (cupRadius + radiusOffset3));

				//rotate randomly
				startPose = toPlanningFrame * rot * startPose;
				pose1 = toPlanningFrame * rot * pose1;
				pose2 = toPlanningFrame * rot * pose2;
				poseGoal = toPlanningFrame * rot * poseGoal;

				PRMNode* nodeStart = new PRMNode(startPose, counterNodes, PRMNode::StartNode);
				PRMNode* node1 = new PRMNode(pose1, counterNodes);
				PRMNode* node2 = new PRMNode(pose2, counterNodes);
				PRMNode* nodeGoal = new PRMNode(poseGoal, counterNodes, PRMNode::GoalNode);

				nodes[nodeStart->getId()] = nodeStart;
				nodes[node1->getId()] = node1;
				nodes[node2->getId()] = node2;
				nodes[nodeGoal->getId()] = nodeGoal;

				PRMEdge* edge1 = new PRMEdge(nodeStart, node1, counterEdges);
				PRMEdge* edge2 = new PRMEdge(node1, node2, counterEdges);
				PRMEdge* edgeGoal = new PRMEdge(node2, nodeGoal, counterEdges);

				nodeStart->addEdge(edge1);
				node1->addEdge(edge1);
				node1->addEdge(edge2);
				node2->addEdge(edge2);
				node2->addEdge(edgeGoal);
				nodeGoal->addEdge(edgeGoal);

				edges[edge1->getId()] = edge1;
				edges[edge2->getId()] = edge2;
				edges[edgeGoal->getId()] = edgeGoal;
			}
	}
	return new prm_planner::PRM(m_robot,
			c_config.prmConfig.visibilityDistance,
			pd,
			nodes,
			edges);
}

bool PouringProblemDefinition::updateToolFrames()
{
	m_tcp = m_robot->getGripper()->getTGripperToObject(); //ais_ros::RosBaseInterface::getRosTransformation(m_bottle->c_params.objectFrameName, m_arm->getOriginalTipLinkFrame());

//	LOG_INFO("Using fixed tcp frame");
//	m_tcp.matrix() << 0.977357, 0.00497387, 0.211539, -0.00184605,
//			-0.0130002, 0.999247, 0.0365686, -0.110046,
//			-0.211197, -0.0384906, 0.976685, 0.0600404,
//			0, 0, 0, 1;

//	m_tcp.translation() = Eigen::Vector3d(-0.002, -0.110, 0.061);
//	Eigen::Quaterniond q(-0.022, 0.117, -0.009, 0.993);
//	m_tcp.linear() = q.toRotationMatrix();

	if (ParameterServer::connectedToVRep)
	{
		Eigen::Affine3d current;
		m_robot->getCurrentFK(current);

		Eigen::Matrix3d rotCurrent = current.linear();
		Eigen::Matrix3d rotTCP = m_tcp.linear();

		Eigen::Vector3d newX = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d tcpZ = rotTCP.col(2);
		Eigen::Vector3d newY = tcpZ.cross(newX).normalized();
		Eigen::Vector3d newZ = newX.cross(newY).normalized();

		Eigen::Matrix3d newRot;
		newRot.col(0) = newX;
		newRot.col(1) = newY;
		newRot.col(2) = newZ;

		m_tcp.translation() += tcpZ * m_bottle->getHeightAboveCenter();
		m_tcp.linear() = newRot;
	}
	else
	{
		Eigen::Affine3d current;
		m_robot->getCurrentFK(current);

		Eigen::Matrix3d rotCurrent = current.linear();
		Eigen::Matrix3d rotTCP = m_tcp.linear();

		Eigen::Vector3d currentZ = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d tcpY = rotTCP.col(1);
		Eigen::Vector3d newY = tcpY.cross(currentZ).normalized();
		Eigen::Vector3d newX = newY.cross(tcpY).normalized();

		Eigen::Matrix3d newRot;
		newRot.col(0) = newX;
		newRot.col(1) = newY;
		newRot.col(2) = tcpY;

		m_tcp.translation() += m_tcp.linear().col(1) * m_bottle->getHeightAboveCenter();
		m_tcp.linear() = newRot;
	}

//	LOG_INFO(m_tcp.matrix());

	Eigen::Matrix3d rot = m_tcp.linear();
	double det = rot.determinant();

	if (m_tcp.matrix().hasNaN() || rot.isApprox(Eigen::Matrix3d::Zero()) || !(det > 0.95 && det < 1.05))
	{
		LOG_ERROR("Cannot set tool frame because of nan's in the transformation (bottle not detected?!)");
		return false;
	}

	m_robot->setToolCenterPointTransformation(m_tcp);

	return true;
}

bool PouringProblemDefinition::plan(const Eigen::Affine3d& goal,
		boost::shared_ptr<Path>& path,
		const PlanningParameters& parameters)
{
	//should be command (pour/back), the name of the bottle and the name of the cup
	if (parameters.input.size() == 3)
	{
		LOG_INFO("Received mode via parameters. New mode: " << parameters.input[0]);
		PLANNER_WRITE_LOCK();
		m_mode = parameters.input[0] == "pour" ? InitToPour : PourToInit;

		//get objects
		auto& objects = ObjectManager::getInstance()->getObjects();
		std::string bottleName = parameters.input[1];
		std::string cupName = parameters.input[2];

		if (bottleName.empty() || cupName.empty())
		{
			LOG_FATAL("Please add objects with names that contains 'bottle' and 'cup' respectively to the problem definition!");
			exit(-1);
		}

		LOG_INFO("Using " << bottleName << " as bottle");
		LOG_INFO("Using " << cupName << " as cup");

		m_bottle = ObjectManager::getInstance()->getObject(bottleName);
		m_cup = ObjectManager::getInstance()->getObject(cupName);
	}

	if (m_mode == InitToPour)
	{
		PRM* prm = createPouringPRM();

		ros::Rate r(10);
		while (prm == NULL)
		{
			LOG_ERROR("PRM is NULL, maybe cup was not detected");
			prm = createPouringPRM();
			r.sleep();
		}

		{
			PLANNER_WRITE_LOCK();
			m_prm = prm;
			m_prmPlanner->setPRM(m_prm);
		}

		return pour(path);
	}
	else if (m_mode == PourToInit)
	{
		return toInit(path);
	}

	return false;
}

void PouringProblemDefinition::publish()
{
	static tf::TransformBroadcaster br;

	tf::Transform t;

	if (!m_tcp.matrix().hasNaN())
	{
		tf::transformEigenToTF(m_tcp, t);
		br.sendTransform(tf::StampedTransform(t, ros::Time::now(), m_robot->getOriginalTipLinkFrame(), "bottle_rim"));
	}

	ApproachingProblemDefinition::publish();
}

std::vector<boost::shared_ptr<prm_planner::InteractiveMarker>> PouringProblemDefinition::getInteractiveMarkers()
{
	if (m_interactiveMarker.empty())
	{
		m_interactiveMarker.push_back(boost::shared_ptr<prm_planner::InteractiveMarker>(new PouringInteractiveMarker(m_plannerInterface, m_bottle)));
	}
	return m_interactiveMarker;
}

PouringProblemDefinition::Mode PouringProblemDefinition::getMode() const
{
	return m_mode;
}

void PouringProblemDefinition::setMode(PouringProblemDefinition::Mode mode)
{
	m_mode = mode;
}

bool PouringProblemDefinition::pour(boost::shared_ptr<prm_planner::Path>& path)
{
	ObjectManager* om = ObjectManager::getInstance();

	if (!m_plannerInterface->activateToolFrameFromPD())
	{
		//try to re-read the transformation between gripper and object
		//and try again to activate the tool frame
		m_plannerInterface->initObjectStates();

		//create PRM again, because
		PRM* prm = createPouringPRM();

		if (prm == NULL)
		{
			LOG_ERROR("PRM is NULL, maybe cup was not detected");
			return false;
		}

		{
			PLANNER_WRITE_LOCK();
			m_prm = prm;
			m_prmPlanner->setPRM(m_prm);
		}

		if (!m_plannerInterface->activateToolFrameFromPD())
		{
			return false;
		}
	}

	auto& startNodes = m_prm->getStartNodes();
	KDL::JntArray joints = m_robot->getKDLChainJointState();
	m_startPose = getCurrentTaskPose();

	boost::shared_ptr<prm_planner::Path> bestPath;
	boost::shared_ptr<prm_planner::Path> bestPourPath;
	double bestLength = std::numeric_limits<double>::max();
	int found = 0;
	boost::shared_ptr<prm_planner::PathPlanner> armPlanner = m_approachingPathPlanner;

	//crate a vector of all start node indices
	//for open mp
	std::vector<int> startNodeIndices;
	startNodeIndices.reserve(startNodes.size());
	for (auto& it : startNodes)
		startNodeIndices.push_back(it.first);

	//create a feasibility checker
	FeasibilityChecker feasibilityChecker(m_robot);

	//will get a private copy due to firstprivate. firstprivate calls to copy constructor
	//for each thread during the first call.
	PD_READ_LOCK();
	CollisionDetector::OpenMPCollisionDetectorStruct cds(m_robot, m_planningScene, { m_cup });

	ais_util::ProgressBar progress("Planning", startNodes.size());

#pragma omp parallel for shared(found, bestLength, bestPath, bestPourPath) firstprivate(cds) schedule(dynamic)
	for (size_t i = 0; i < startNodeIndices.size(); ++i)
	{
		progress.increment();
		int index = startNodeIndices[i];
		auto& it = startNodes.at(index);

		boost::shared_ptr<prm_planner::Path> approachPath;

		//check if goal is reachable at all
		Eigen::Affine3d pose = it->getPose();
		if (!feasibilityChecker.check(pose, cds.cdWithObject))
		{
			continue;
		}

		if (armPlanner->plan(joints, m_startPose, it->getPose(), cds.cdWithObject, approachPath))
		{
			const prm_planner::Path::Waypoint& wp = approachPath->back();

			boost::shared_ptr<Path> pourPath;
			if (m_planner->planSingleStartMultipleGoal(wp.jointPose, wp.pose, index, cds.cdWithoutObject, pourPath))
			{
//				prm_planner::Path::Waypoint& back = approachPath->back();
//				back.maxAngularVel = 0.08;
//				back.maxTranslationalVel = 0.01;

				int i = 0;
				for (auto& pourWP : *pourPath)
				{
//					slow down with the third waypoint
					if (m_slowPouring && i++ > 1)
					{
						pourWP.maxAngularVel = 0.002; //0.002
						pourWP.maxTranslationalVel = 0.01;
					}
					else
					{
						pourWP.maxAngularVel = -1;
						pourWP.maxTranslationalVel = -1;
					}
					approachPath->append(pourWP);
				}

#pragma omp critical
				{
					++found;
					if (approachPath->getPathLength() < bestLength)
					{
						bestLength = approachPath->getPathLength();
						bestPath = approachPath;
						bestPourPath = pourPath;
					}
				}
			}
		}
	}
	progress.finish();

	path = bestPath;

	if (path.get() == NULL)
	{
		return false;
	}
	else
	{
		m_pourPath.reset(new Path(*bestPourPath));
		return true;
	}
}

bool PouringProblemDefinition::toInit(boost::shared_ptr<prm_planner::Path>& path)
{
	ObjectManager* om = ObjectManager::getInstance();

	if (m_pourPath.get() == NULL)
	{
		LOG_ERROR("PourToInit not possible");
		return false;
	}

	m_mutex.lock_shared();
	boost::shared_ptr<CollisionDetector> cdWithObjects(new CollisionDetector(m_robot, m_planningScene));
	m_mutex.unlock_shared();

	int id = getCurrentWaypointId();

	//make reverse pouring motion
	path = m_pourPath->getSubPath(0, id);
	path->reverse();

	for (auto& pourWP : *path)
	{
		pourWP.maxAngularVel = 0.2;
	}

	//run planner to get back to start pose
	KDL::JntArray joints = path->back().jointPose;
	Eigen::Affine3d pose = path->back().getTaskPose();

	boost::shared_ptr<prm_planner::PathPlanner> armPlanner = m_approachingPathPlanner;
	boost::shared_ptr<prm_planner::Path> reverseApproachPath;

	if (!armPlanner->plan(joints, pose, m_startPose, cdWithObjects, reverseApproachPath))
	{
		LOG_ERROR("Cannot find a way back to the initial pose");
		m_pourPath.reset();
		return false;
	}

//	path = m_pourPath;

	//set the velocity of the last pouring waypoint to default
	//to allow normal velocities again
	prm_planner::Path::Waypoint& back = path->back();
	back.maxAngularVel = -1;
	back.maxTranslationalVel = -1;

	path->append(*reverseApproachPath, true);

	m_pourPath.reset();

//		m_planner->changeObjectState(m_bottle->c_params.name, false);
	return true;

}

int PouringProblemDefinition::getCurrentWaypointId()
{
	Eigen::Affine3d pose;
	m_robot->getCurrentFK(pose);
	Eigen::Vector3d pos = pose.translation();

	double min = std::numeric_limits<double>::max();
	int nearest = 0;

	int i = 0;
	for (auto& it : *m_pourPath)
	{
		++i;
		double dist = (it.pose.translation() - pos).norm();
		if (dist < min)
		{
			min = dist;
			nearest = i;
		}
	}

	if (nearest > 0)
		nearest--;

//	if (min > 0.1)
//	{
//		LOG_WARNING("You send stop too early!");
//		return -1;
//	}

	LOG_INFO("neareast point: " << nearest << " " << m_pourPath->size());

	Path::Waypoint& wp1 = (*m_pourPath)[nearest];
	Path::Waypoint& wp2 = (*m_pourPath)[nearest + 1];
//	double dist1 = (wp1.pose.translation() - pos).norm();
	double dist2 = (wp2.pose.translation() - pos).norm();
	double dist3 = (wp2.pose.translation() - wp1.pose.translation()).norm();
	if (dist2 > dist3)
	{
		if (nearest == 0)
			return nearest;
		else
			return nearest - 1;
	}
	else
	{
		return nearest;
	}

	LOG_ERROR("Should not happen");
	return -1;
}

} /* namespace neurobots_prm_planner_problems */

PLUGINLIB_EXPORT_CLASS(neurobots_prm_planner_problems::PouringProblemDefinition, prm_planner::ProblemDefinition)

#endif
