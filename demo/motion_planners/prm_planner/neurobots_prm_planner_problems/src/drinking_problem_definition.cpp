/*
 * drinking_problem_definition.cpp
 *
 *  Created on: Sep 29, 2016
 *      Author: kuhnerd
 */

#ifdef FOUND_PRM_PLANNER

#include <prm_planner/util/parameter_server.h>
#include <neurobots_prm_planner_problems/drinking_problem_definition.h>
#include <neurobots_prm_planner_problems/drinking_interactive_marker.h>

#include <ais_ros/ros_base_interface.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/planners/path_planner.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/planners/prm/prm.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/planners/prm/prma_star.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/robot/feasibility_checker.h>
#include <prm_planner/visualization/interactive_marker.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner_constraints/constraint.h>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner_robot/path.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <random>

using namespace prm_planner;

namespace neurobots_prm_planner_problems
{

DrinkingProblemDefinition::DrinkingProblemDefinition() :
				m_tcp(Eigen::Affine3d::Identity()),
				m_mode(InitToMouth),
				m_prm(NULL),
				m_prmInitialized(false)
{
}

DrinkingProblemDefinition::~DrinkingProblemDefinition()
{
}

void DrinkingProblemDefinition::init(const boost::shared_ptr<prm_planner::Robot> robot,
		const boost::shared_ptr<prm_planner::Constraint> constraint,
		const boost::shared_ptr<prm_planner::PRMPlanner> planner,
		const prm_planner::parameters::ProblemConfig& config)
{
	ApproachingProblemDefinition::init(robot, constraint, planner, config);

	m_mode = InitToMouth;
	m_tcp = Eigen::Affine3d::Identity();

	m_robotName = m_robot->getName();

	//get objects
	auto& objects = ObjectManager::getInstance()->getObjects();
	std::string cupName = "";
	std::string headName = "";

	for (auto& it : objects)
	{
		if (boost::contains(it.first, "cup"))
		{
			cupName = it.first;
			break;
		}
	}

	for (auto& it : objects)
	{
		if (boost::contains(it.first, "head"))
		{
			headName = it.first;
			break;
		}
	}

	if (cupName.empty())
	{
		LOG_FATAL("Please add an object with a name which contains 'cup' to the problem definition!");
		exit(-1);
	}

	if (headName.empty())
	{
		LOG_FATAL("Please add an object with a name which contains 'head' to the problem definition!");
		exit(-1);
	}

	LOG_INFO("Using " << cupName << " as cup");
	LOG_INFO("Using " << headName << " as head");

	m_cup = ObjectManager::getInstance()->getObject(cupName);
	m_head = ObjectManager::getInstance()->getObject(headName);
}

//Eigen::Affine3d DrinkingProblemDefinition::samplePose()
//{
//	return Eigen::Affine3d::Identity();
//}

std::vector<boost::shared_ptr<InteractiveMarker> > DrinkingProblemDefinition::getInteractiveMarkers()
{
	if (m_interactiveMarker.empty())
	{
		m_interactiveMarker.push_back(boost::shared_ptr<InteractiveMarker>(new DrinkingInteractiveMarker(m_plannerInterface, m_cup)));
	}
	return m_interactiveMarker;
}

bool DrinkingProblemDefinition::updateToolFrames()
{
	m_tcp = m_robot->getGripper()->getTGripperToObject();

//	if (ParameterServer::connectedToVRep)
//	{
//		Eigen::Affine3d current;
//		m_robot->getCurrentFK(current);
//
//		Eigen::Matrix3d rotCurrent = current.linear();
//		Eigen::Matrix3d rotTCP = m_tcp.linear();
//
//		Eigen::Vector3d newX = Eigen::Vector3d::UnitZ();
//		Eigen::Vector3d tcpZ = rotTCP.col(2);
//		Eigen::Vector3d newY = tcpZ.cross(newX).normalized();
//		Eigen::Vector3d newZ = newX.cross(newY).normalized();
//
//		Eigen::Matrix3d newRot;
//		newRot.col(0) = newX;
//		newRot.col(1) = newY;
//		newRot.col(2) = newZ;
//
//		m_tcp.translation() += tcpZ * m_cup->getHeightAboveCenter() - newY * m_cup->getRadius();
//		m_tcp.linear() = newRot;
//	}
//	else
//	{
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

	m_tcp.translation() += m_tcp.linear().col(1) * m_cup->getHeightAboveCenter() + newY * m_cup->getRadius();
	m_tcp.linear() = newRot;
//	}

	Eigen::Matrix3d rot = m_tcp.linear();
	double det = rot.determinant();

	if (m_tcp.matrix().hasNaN() || rot.isApprox(Eigen::Matrix3d::Zero()) || !(det > 0.95 && det < 1.05))
	{
		LOG_ERROR("Cannot set tool frame because of nan's in the transformation (bottle/mouth not detected?!)");
		return false;
	}

	m_robot->setToolCenterPointTransformation(m_tcp);

	return true;
}

PRM* DrinkingProblemDefinition::createDrinkingPRM()
{
	LOG_DEBUG("Loading task space roadmap...");

	ros::NodeHandle n("problem_definitions/planning/drinking");
	double offsetX = 0.0;
	double offsetY = 0.0;
	double offsetZ = 0.0;
	n.getParam("offset_x", offsetX);
	n.getParam("offset_y", offsetY);
	n.getParam("offset_z", offsetZ);

	LOG_INFO("Using offsets: [" << offsetX << ", " << offsetY << ", " << offsetZ << "]");

	const double cupRadius = m_cup->getRadius();

	boost::shared_ptr<ProblemDefinition> pd = shared_from_this();

	std::unordered_map<int, PRMNode*> nodes;
	std::unordered_map<int, PRMEdge*> edges;

	static std::uniform_real_distribution<double> dist(-M_PI, M_PI);
	static std::random_device gen;

	boost::atomic_int counterNodes(0), counterEdges(0);
	KDL::JntArray joints;

	Eigen::Affine3d toPlanningFrame = ais_ros::RosBaseInterface::getRosTransformation(m_head->c_params.objectFrameName, c_config.planningFrame);
	Eigen::Matrix3d rotMouth = toPlanningFrame.linear();

//	if (ParameterServer::connectedToVRep)
//	{
//		for (double yRot = -0.2; yRot <= 0.2; yRot += 0.05)
//		{
//			Eigen::Matrix3d baseRotation;
//			baseRotation.col(0) = Eigen::Vector3d(0, -1, 0);
//			baseRotation.col(1) = Eigen::Vector3d(1, 0, 0);
//			baseRotation.col(2) = Eigen::Vector3d(0, 0, 1);
//
//			Eigen::Affine3d startPose = Eigen::Affine3d::Identity();
//			startPose.linear() = baseRotation * (Eigen::AngleAxisd(yRot, Eigen::Vector3d::UnitZ())).matrix();
//			startPose.translation() = Eigen::Vector3d(0.05, 0, 0.02);
//
//			Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
//			pose1.linear() = baseRotation * (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yRot, Eigen::Vector3d::UnitZ())).matrix();
//			pose1.translation() = Eigen::Vector3d(0.025, 0, 0.013);
//
//			Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
//			pose2.linear() = baseRotation * (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yRot, Eigen::Vector3d::UnitZ())).matrix(); // * Eigen::AngleAxisd(-dir * M_PI / 180 * 70, Eigen::Vector3d::UnitX()).matrix();
//			pose2.translation() = Eigen::Vector3d(0.005, 0, 0);
//
//			Eigen::Affine3d poseGoal = Eigen::Affine3d::Identity();
//			poseGoal.linear() = baseRotation * (Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yRot, Eigen::Vector3d::UnitZ())).matrix(); // * Eigen::AngleAxisd(-dir * M_PI / 180 * 70, Eigen::Vector3d::UnitX()).matrix();
//			poseGoal.translation() = Eigen::Vector3d(0.0, 0, 0);
//
//			startPose = toPlanningFrame * startPose;
//			pose1 = toPlanningFrame * pose1;
//			pose2 = toPlanningFrame * pose2;
//			poseGoal = toPlanningFrame * poseGoal;
//
//			PRMNode* nodeStart = new PRMNode(startPose, counterNodes, PRMNode::StartNode);
//			PRMNode* node1 = new PRMNode(pose1, counterNodes);
//			PRMNode* node2 = new PRMNode(pose2, counterNodes);
//			PRMNode* nodeGoal = new PRMNode(poseGoal, counterNodes, PRMNode::GoalNode);
//
//			nodes[nodeStart->getId()] = nodeStart;
//			nodes[node1->getId()] = node1;
//			nodes[node2->getId()] = node2;
//			nodes[nodeGoal->getId()] = nodeGoal;
//
//			PRMEdge* edge1 = new PRMEdge(nodeStart, node1, counterEdges);
//			PRMEdge* edge2 = new PRMEdge(node1, node2, counterEdges);
//			PRMEdge* edgeGoal = new PRMEdge(node2, nodeGoal, counterEdges);
//
//			nodeStart->addEdge(edge1);
//			node1->addEdge(edge1);
//			node1->addEdge(edge2);
//			node2->addEdge(edge2);
//			node2->addEdge(edgeGoal);
//			nodeGoal->addEdge(edgeGoal);
//
//			edges[edge1->getId()] = edge1;
//			edges[edge2->getId()] = edge2;
//			edges[edgeGoal->getId()] = edgeGoal;
//		}
//	}
//	else
//	{
	for (double angle = -0.6; angle <= 0.6; angle += 0.1)
	{
		Eigen::Translation3d t(Eigen::Vector3d::UnitX() * m_cup->getRadius());
		Eigen::Quaterniond r(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

		Eigen::Matrix3d baseRotation;
		baseRotation.col(0) = Eigen::Vector3d(0, 1, 0);
		baseRotation.col(1) = Eigen::Vector3d(-1, 0, 0);
		baseRotation.col(2) = Eigen::Vector3d(0, 0, 1);

		Eigen::Affine3d startPose = Eigen::Affine3d::Identity();
		startPose.linear() = baseRotation * r;
		startPose.translation() = (r * t.inverse()).translation() + t.translation() + Eigen::Vector3d(0.15, 0, 0.02);

		Eigen::Affine3d startPose2 = Eigen::Affine3d::Identity();
		startPose2.linear() = baseRotation * r;
		startPose2.translation() = (r * t.inverse()).translation() + t.translation() + Eigen::Vector3d(0.09, 0, 0.02);

		Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
		pose1.linear() = baseRotation * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()).matrix() * r;
		pose1.translation() = (r * t.inverse()).translation() + t.translation() + Eigen::Vector3d(0.08, 0, 0.013);

		Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
		pose2.linear() = baseRotation * Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitX()).matrix() * r; // * Eigen::AngleAxisd(-dir * M_PI / 180 * 70, Eigen::Vector3d::UnitX()).matrix();
		pose2.translation() = (r * t.inverse()).translation() + t.translation() + Eigen::Vector3d(0.08, 0, 0);

		Eigen::Affine3d poseGoal = Eigen::Affine3d::Identity();
		poseGoal.linear() = baseRotation * Eigen::AngleAxisd(-1.1, Eigen::Vector3d::UnitX()).matrix() * r; // * Eigen::AngleAxisd(-dir * M_PI / 180 * 70, Eigen::Vector3d::UnitX()).matrix();
		poseGoal.translation() = (r * t.inverse()).translation() + t.translation() + Eigen::Vector3d(0.07, 0, 0);
		LOG_INFO(poseGoal.translation());

		//adaption to person
		toPlanningFrame(0, 3) += offsetX;
		toPlanningFrame(1, 3) += offsetY;
		toPlanningFrame(2, 3) += offsetZ;

		startPose = toPlanningFrame * startPose;
		startPose2 = toPlanningFrame * startPose2;
		pose1 = toPlanningFrame * pose1;
		pose2 = toPlanningFrame * pose2;
		poseGoal = toPlanningFrame * poseGoal;

		PRMNode* nodeStart = new PRMNode(startPose, counterNodes, PRMNode::StartNode);
		PRMNode* nodeStart2 = new PRMNode(startPose2, counterNodes);
		PRMNode* node1 = new PRMNode(pose1, counterNodes);
		PRMNode* node2 = new PRMNode(pose2, counterNodes);
		PRMNode* nodeGoal = new PRMNode(poseGoal, counterNodes, PRMNode::GoalNode);

		nodes[nodeStart->getId()] = nodeStart;
		nodes[nodeStart2->getId()] = nodeStart2;
		nodes[node1->getId()] = node1;
		nodes[node2->getId()] = node2;
		nodes[nodeGoal->getId()] = nodeGoal;

		PRMEdge* edge1 = new PRMEdge(nodeStart, nodeStart2, counterEdges);
		PRMEdge* edge2 = new PRMEdge(nodeStart2, node1, counterEdges);
		PRMEdge* edge3 = new PRMEdge(node1, node2, counterEdges);
		PRMEdge* edgeGoal = new PRMEdge(node2, nodeGoal, counterEdges);

		nodeStart->addEdge(edge1);
		nodeStart2->addEdge(edge1);
		nodeStart2->addEdge(edge2);
		node1->addEdge(edge2);
		node1->addEdge(edge3);
		node2->addEdge(edge3);
		node2->addEdge(edgeGoal);
		nodeGoal->addEdge(edgeGoal);

		edges[edge1->getId()] = edge1;
		edges[edge2->getId()] = edge2;
		edges[edge3->getId()] = edge3;
		edges[edgeGoal->getId()] = edgeGoal;
	}
//	}

	PRM* prm = new PRM(m_robot,
			c_config.prmConfig.visibilityDistance,
			pd,
			nodes,
			edges);

	return prm;
}

//bool DrinkingProblemDefinition::findStartTaskPose(Eigen::Affine3d& startPose,
//		bool& hasToApproach)
//{
//	//dont needed because of custom planning
//	return false;
//}

bool DrinkingProblemDefinition::plan(const Eigen::Affine3d& goal,
		boost::shared_ptr<Path>& path,
		const PlanningParameters& parameters)
{
	//should be command (pour/back), the name of the bottle and the name of the cup
	if (parameters.input.size() == 3)
	{
		LOG_INFO("Received mode via parameters. New mode: " << parameters.input[0]);

		PLANNER_WRITE_LOCK();
		m_mode = parameters.input[0] == "drink" ? InitToMouth : MouthToInit;

		//get objects
		auto& objects = ObjectManager::getInstance()->getObjects();
		std::string cupName = parameters.input[2];

		if (cupName.empty())
		{
			LOG_FATAL("Please add objects with names that contains 'cup' respectively to the problem definition!");
			exit(-1);
		}

		LOG_INFO("Using " << cupName << " as cup");
		m_cup = ObjectManager::getInstance()->getObject(cupName);
	}

	if (m_mode == InitToMouth)
	{
		PRM* prm = createDrinkingPRM();

		ros::Rate r(10);
		while (prm == NULL)
		{
			LOG_ERROR("PRM is NULL, maybe cup was not detected");
			prm = createDrinkingPRM();
			r.sleep();
		}

		{
			PLANNER_WRITE_LOCK();
			m_prm = prm;
			m_prmPlanner->setPRM(m_prm);
		}

		return drink(path);
	}
	else if (m_mode == MouthToInit)
	{
		return toInit(path);
	}

	return false;
}

void DrinkingProblemDefinition::publish()
{
	static tf::TransformBroadcaster br;

	tf::Transform t;

	if (!m_tcp.matrix().hasNaN())
	{
		tf::transformEigenToTF(m_tcp, t);
		br.sendTransform(tf::StampedTransform(t, ros::Time::now(), m_robot->getOriginalTipLinkFrame(), "cup_rim"));
	}

	ApproachingProblemDefinition::publish();
}

DrinkingProblemDefinition::Mode DrinkingProblemDefinition::getMode() const
{
	return m_mode;
}

void DrinkingProblemDefinition::setMode(DrinkingProblemDefinition::Mode mode)
{
	m_mode = mode;
}

bool DrinkingProblemDefinition::drink(boost::shared_ptr<prm_planner::Path>& path)
{
	ObjectManager* om = ObjectManager::getInstance();

	if (!m_plannerInterface->activateToolFrameFromPD())
	{
//		//try to re-read the transformation between gripper and object
//		//and try again to activate the tool frame
//		m_plannerInterface->initObjectStates();
//
//		//create PRM again, because
//		PRM* prm = createDrinkingPRM();
//
//		if (prm == NULL)
//		{
//			LOG_ERROR("PRM is NULL, maybe cup was not detected");
//			return false;
//		}
//
//		{
//			PLANNER_WRITE_LOCK();
//			m_prm = prm;
//			m_prmPlanner->setPRM(m_prm);
//		}
//
//		if (!m_plannerInterface->activateToolFrameFromPD())
//		{
//			return false;
//		}
		LOG_ERROR("Maybe cup was not detected");
		return false;
	}

	auto& startNodes = m_prm->getStartNodes();
	KDL::JntArray joints = m_robot->getKDLChainJointState();
	m_startPose = getCurrentTaskPose();
	boost::shared_ptr<prm_planner::Path> approachPath;
	boost::shared_ptr<prm_planner::PathPlanner> armPlanner = m_approachingPathPlanner;

	m_mutex.lock_shared();
	boost::shared_ptr<CollisionDetector> cdWithObjects(new CollisionDetector(m_robot, m_planningScene));
	boost::shared_ptr<CollisionDetector> cdWithoutHead(new CollisionDetector(m_robot, m_planningScene, { m_head }));
	m_mutex.unlock_shared();

	FeasibilityChecker feasibilityChecker(m_robot);

	//approach
	for (auto& it : startNodes)
	{
		//check if goal is reachable at all
		Eigen::Affine3d pose = it.second->getPose();
		if (!feasibilityChecker.check(pose, cdWithObjects))
		{
			continue;
		}

		if (armPlanner->plan(joints, m_startPose, pose, cdWithObjects, approachPath))
		{
			const prm_planner::Path::Waypoint& wp = approachPath->back();

			LOG_INFO("found approaching path");

			boost::shared_ptr<Path> drinkPath;
			if (m_planner->planSingleStartMultipleGoal(wp.jointPose, wp.pose, it.first, cdWithoutHead, drinkPath))
			{
				m_drinkPath.reset(new Path(*drinkPath));

				prm_planner::Path::Waypoint& back = approachPath->back();
				back.maxAngularVel = 0.08;
				back.maxTranslationalVel = 0.01;

				int i = 0;
				for (auto& drinkWP : *drinkPath)
				{
					//change pouring velocity
					if (i++ > 0)
					{
						drinkWP.maxAngularVel = 0.04;
						drinkWP.maxTranslationalVel = 0.01;
					}
					approachPath->append(drinkWP);
				}

				path = approachPath;

				return true;
			}
		}
	}

	return false;
}

bool DrinkingProblemDefinition::toInit(boost::shared_ptr<prm_planner::Path>& path)
{
	ObjectManager* om = ObjectManager::getInstance();

	if (m_drinkPath.get() == NULL)
	{
		LOG_ERROR("MouthToInit not possible");
		return false;
	}

	//make reverse pouring motion
	m_drinkPath->reverse();

	//run planner to get back to start pose
//	KDL::JntArray joints = m_drinkPath->back().jointPose;
//	Eigen::Affine3d pose = m_drinkPath->back().getTaskPose();
//
//	boost::shared_ptr<prm_planner::PathPlanner> armPlanner = m_approachingPathPlanner;
//	boost::shared_ptr<prm_planner::Path> reverseApproachPath;
//
//	m_mutex.lock_shared();
//	boost::shared_ptr<CollisionDetector> cdWithObjects(new CollisionDetector(m_robot, m_planningScene));
//	m_mutex.unlock_shared();
//
//	if (!armPlanner->plan(joints, pose, m_startPose, cdWithObjects, reverseApproachPath))
//	{
//		LOG_ERROR("Cannot find a way back to the initial pose");
//		m_drinkPath.reset();
//		return false;
//	}

	path = m_drinkPath;

	//set the velocity of the last pouring waypoint to default
	//to allow normal velocities again
//	prm_planner::Path::Waypoint& back = path->back();
//	back.maxAngularVel = -1;
//	back.maxTranslationalVel = -1;
//
//	path->append(*reverseApproachPath, true);

	m_drinkPath.reset();

	return true;
}

void DrinkingProblemDefinition::update(const boost::shared_ptr<prm_planner::PlanningScene>& planningScene)
{
	ApproachingProblemDefinition::update(planningScene);

	//initialize the PRM here (mainly for visualization, since
	//every plan() call will generate a new PRM). We cannot generate
	//it in the init method, because ROS transformations are not
	//available (they are initialized later).
	if (!m_prmInitialized)
	{
		PRM* prm = createDrinkingPRM();

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

void DrinkingProblemDefinition::initPlanner()
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

} /* namespace neurobots_prm_planner_problems */

PLUGINLIB_EXPORT_CLASS(neurobots_prm_planner_problems::DrinkingProblemDefinition, prm_planner::ProblemDefinition)

#endif
