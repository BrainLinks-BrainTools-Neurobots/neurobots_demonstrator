/*
 * UDPController.cpp
 *
 *  Created on: 19.11.2015
 *      Author: killmani
 */

#include "kuka_manager/UDPController.h"
#include "kuka_manager/definitions.h"
#include "kuka_manager/iiwa.h"

#include <tf/tf.h>
#include <omnirob_lbr/lbr.h>

UDPController::UDPController() :
				robot(KUKA::Robotname::None),
				jointCounter(0),
				cartesianCounter(0)
{
}

UDPController::~UDPController()
{
}

void UDPController::start(ros::NodeHandle& nh,
		const char* host,
		int port,
		KUKA::Robotname robot)
{
	this->robot = robot;
	std::string message_prefix;
	if (robot == KUKA::Robotname::Omnirob)
	{
		message_prefix = "omnirob/";
	}
	else if (robot == KUKA::Robotname::IIWA)
	{
		message_prefix = "iiwa/";
	}

	// Init ros stuff
	this->nh = nh;

	//sync, joint state
	commandJointStateSub = nh.subscribe<sensor_msgs::JointState>(
			(message_prefix + "cmd_joint_state"), 1,
			&UDPController::commandJointStateCallback, this);
	commandJointStateService = nh.advertiseService((message_prefix + "cmd_joint_state"),
			&UDPController::commandJointStateCallback, this);

	//async, joint state
	commandJointStateAsyncSub = nh.subscribe<sensor_msgs::JointState>(
			(message_prefix + "cmd_joint_state_async"), 1,
			&UDPController::commandJointStateAsyncCallback, this);
	commandJointStateAsyncService = nh.advertiseService((message_prefix + "cmd_joint_state_async"),
			&UDPController::commandJointStateAsyncCallback, this);

	commandRealTimeTrajectorySub = nh.subscribe<trajectory_msgs::JointTrajectory>((message_prefix + "cmd_real_time_trajectory"), 1,
			&UDPController::sendRealTimeTrajectory, this);

	//sync, cartesian
	commandCartesianSub = nh.subscribe<geometry_msgs::Transform>(
			(message_prefix + "cmd_cartesian_state"), 1,
			&UDPController::commandCartesianCallback, this);
	commandCartesianService = nh.advertiseService((message_prefix + "cmd_cartesian_state"),
			&UDPController::commandCartesianCallback, this);

	//async, cartesian
	commandCartesianAsyncSub = nh.subscribe<geometry_msgs::Transform>(
			(message_prefix + "cmd_cartesian_state_async"), 1,
			&UDPController::commandCartesianCallback, this);

	//pub, joint state finished
	commandJointStateFinishedPub = nh.advertise<std_msgs::Bool>(
			(message_prefix + "cmd_joint_state_finished"), 1);

	//pub, cartesian finished
	commandCartesianFinishedPub = nh.advertise<std_msgs::Bool>(
			(message_prefix + "cmd_cartesian_state_finished"), 1);

	//change velocity
	changeVelAccCommand = nh.subscribe<kuka_manager::Speed>(
			(message_prefix + "ch_vel_acc"), 1,
			&UDPController::changeVelAccCallback, this);
	changeVelAccService = nh.advertiseService((message_prefix + "change_vel_acc"),
			&UDPController::changeVelAccCallbackService, this);

	//stop arm (only works, if iiwa driver is not blocked?)
	commandStopArm = nh.subscribe<std_msgs::Bool>(
			(message_prefix + "stop_arm"), 1,
			&UDPController::stopArm, this);

	// Init udp stuff
	udpSender.init(host, port, port + 1);

	// Init finished
	ROS_INFO("Successfully initialized Kuka robot controller");
}

void UDPController::runThread()
{
	ros::Rate r(500.0);
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}

void UDPController::changeVelAccCallback(
		const kuka_manager::Speed::ConstPtr &speed)
{
	VelAccPacket velAccPacket;
	velAccPacket.velTheta = speed->velTheta;
	velAccPacket.accTheta = speed->accTheta;

	udpSender.send(&velAccPacket, sizeof(velAccPacket), 0.0);
}

bool UDPController::changeVelAccCallbackService(kuka_manager::SpeedAcceleration::Request &req,
		kuka_manager::SpeedAcceleration::Response &res)
{
	VelAccPacket velAccPacket;
	velAccPacket.velTheta = req.vel;
	velAccPacket.accTheta = req.acc;

	udpSender.send(&velAccPacket, sizeof(velAccPacket), 0.0);
	usleep(100);

	res.success = true;
	return true;
}

void UDPController::commandJointStateCallback(
		const sensor_msgs::JointState::ConstPtr & jointState)
{
	std::unordered_map<std::string, double> joints;
	getJointMap(jointState->name, jointState->position, joints);

	bool isFinished = moveRobotArmWithJointState(jointState->header.stamp.toNSec(),
			jointCounter, joints);

	std_msgs::Bool isFinishedMsg;
	isFinishedMsg.data = isFinished;

	commandJointStateFinishedPub.publish(isFinishedMsg);
}

bool UDPController::commandJointStateCallback(kuka_manager::JointState::Request &req,
		kuka_manager::JointState::Response &res)
{
	std::unordered_map<std::string, double> joints;
	getJointMap(req.jointState.name, req.jointState.position, joints);

	res.isFinished = moveRobotArmWithJointState(req.jointState.header.stamp.toNSec(),
			jointCounter, joints);

	return res.isFinished;
}

void UDPController::commandJointStateAsyncCallback(
		const sensor_msgs::JointState::ConstPtr & jointState)
{
	std::unordered_map<std::string, double> joints;
	getJointMap(jointState->name, jointState->position, joints);
	moveRobotArmWithJointStateAsync(jointState->header.stamp.toNSec(), jointCounter, joints);
}

bool UDPController::commandJointStateAsyncCallback(kuka_manager::JointState::Request &req,
		kuka_manager::JointState::Response &res)
{
	std::unordered_map<std::string, double> joints;
	getJointMap(req.jointState.name, req.jointState.position, joints);
	moveRobotArmWithJointStateAsync(req.jointState.header.stamp.toNSec(), jointCounter, joints);

	res.isFinished = false;
	return true;
}

bool UDPController::moveRobotArmWithJointState(double timestamp,
		int counter,
		std::unordered_map<std::string, double> jointState)
{
	JointPacket joints;
	joints.timestamp_msecs = timestamp;
	joints.counter = counter;

	if (robot == KUKA::Robotname::Omnirob)
	{
		joints.j1 = jointState[LBRJoints::joint1];
		joints.j2 = jointState[LBRJoints::joint2];
		joints.j3 = jointState[LBRJoints::joint3];
		joints.j4 = jointState[LBRJoints::joint4];
		joints.j5 = jointState[LBRJoints::joint5];
		joints.j6 = jointState[LBRJoints::joint6];
		joints.j7 = jointState[LBRJoints::joint7];
	}
	else if (robot == KUKA::Robotname::IIWA)
	{
		joints.j1 = jointState[IIWAJoints::joint1];
		joints.j2 = jointState[IIWAJoints::joint2];
		joints.j3 = jointState[IIWAJoints::joint3];
		joints.j4 = jointState[IIWAJoints::joint4];
		joints.j5 = jointState[IIWAJoints::joint5];
		joints.j6 = jointState[IIWAJoints::joint6];
		joints.j7 = jointState[IIWAJoints::joint7];
	}

	udpSender.send(&joints, sizeof(joints), 0.0);
	jointCounter++;

	// Wait for return
	ActionFinishedPacket packet;
	int length = -1;
	ros::Rate r(100.0);
	while (length == -1)
	{
		length = udpSender.receive(&packet, sizeof(packet), 0.0);
		if (length != -1 && packet.isFinished)
		{
			ROS_INFO("received");
			return true;
		}
		r.sleep();
	}

	return false;
}

bool UDPController::moveRobotArmWithJointStateAsync(double timestamp,
		int counter,
		std::unordered_map<std::string, double> jointState)
{
	JointPacket joints;
	joints.timestamp_msecs = timestamp;
	joints.counter = counter;

	if (robot == KUKA::Robotname::Omnirob)
	{
		joints.j1 = jointState[LBRJoints::joint1];
		joints.j2 = jointState[LBRJoints::joint2];
		joints.j3 = jointState[LBRJoints::joint3];
		joints.j4 = jointState[LBRJoints::joint4];
		joints.j5 = jointState[LBRJoints::joint5];
		joints.j6 = jointState[LBRJoints::joint6];
		joints.j7 = jointState[LBRJoints::joint7];
	}
	else if (robot == KUKA::Robotname::IIWA)
	{
		joints.j1 = jointState[IIWAJoints::joint1];
		joints.j2 = jointState[IIWAJoints::joint2];
		joints.j3 = jointState[IIWAJoints::joint3];
		joints.j4 = jointState[IIWAJoints::joint4];
		joints.j5 = jointState[IIWAJoints::joint5];
		joints.j6 = jointState[IIWAJoints::joint6];
		joints.j7 = jointState[IIWAJoints::joint7];
	}

	udpSender.send(&joints, sizeof(joints), 0.0);
	jointCounter++;

	return true;
}

void UDPController::commandCartesianCallback(
		const geometry_msgs::Transform::ConstPtr &cartesian)
{
	tf::Quaternion quat(cartesian->rotation.x, cartesian->rotation.y,
			cartesian->rotation.z, cartesian->rotation.w);

	bool isFinished = moveRobotArmWithCartesian(ros::Time::now().toNSec(),
			cartesianCounter, cartesian->translation.x,
			cartesian->translation.y, cartesian->translation.z, quat.getX(),
			quat.getY(), quat.getZ());

	std_msgs::Bool isFinishedMsg;
	isFinishedMsg.data = isFinished;

	commandCartesianFinishedPub.publish(isFinishedMsg);
}

bool UDPController::commandCartesianCallback(
		kuka_manager::JointState::Request &req,
		kuka_manager::JointState::Response &res)
{
	tf::Quaternion quat(req.transform.rotation.x, req.transform.rotation.y,
			req.transform.rotation.z, req.transform.rotation.w);

	res.isFinished = moveRobotArmWithCartesian(ros::Time::now().toNSec(),
			cartesianCounter, req.transform.translation.x,
			req.transform.translation.y, req.transform.translation.z,
			quat.getX(), quat.getY(), quat.getZ());

	return res.isFinished;
}

void UDPController::commandCartesianAsyncCallback(
		const geometry_msgs::Transform::ConstPtr &cartesian)
{
	tf::Quaternion quat(cartesian->rotation.x, cartesian->rotation.y,
			cartesian->rotation.z, cartesian->rotation.w);

	moveRobotArmWithCartesianAsync(ros::Time::now().toNSec(), cartesianCounter,
			cartesian->translation.x, cartesian->translation.y,
			cartesian->translation.z, quat.getX(), quat.getY(), quat.getZ());
}

bool UDPController::moveRobotArmWithCartesian(double timestamp,
		int counter,
		double x,
		double y,
		double z,
		double alpha,
		double beta,
		double gamma)
{
	CartesianPacket cartesian;
	cartesian.timestamp_msecs = timestamp;
	cartesian.counter = counter;

	cartesian.x = x;
	cartesian.y = y;
	cartesian.z = z;
	cartesian.alpha = alpha;
	cartesian.beta = beta;
	cartesian.gamma = gamma;

	udpSender.send(&cartesian, sizeof(cartesian), 0.0);
	cartesianCounter++;

	// Wait for return
	ActionFinishedPacket packet;
	int length = -1;
	ros::Rate r(100.0);
	while (length == -1)
	{
		length = udpSender.receive(&packet, sizeof(packet), 0.0);
		if (length != -1 && packet.isFinished)
		{
			return true;
		}
		r.sleep();
	}

	return false;
}

bool UDPController::moveRobotArmWithCartesianAsync(double timestamp,
		int counter,
		double x,
		double y,
		double z,
		double alpha,
		double beta,
		double gamma)
{
	CartesianPacket cartesian;
	cartesian.timestamp_msecs = timestamp;
	cartesian.counter = counter;

	cartesian.x = x;
	cartesian.y = y;
	cartesian.z = z;
	cartesian.alpha = alpha;
	cartesian.beta = beta;
	cartesian.gamma = gamma;

	udpSender.send(&cartesian, sizeof(cartesian), 0.0);
	cartesianCounter++;

	return true;
}

void UDPController::stopArm(const std_msgs::Bool::ConstPtr& tmp)
{
	StopPacket stopPacket;
	ROS_INFO("Stopping the Arm");
	udpSender.send(&stopPacket, sizeof(stopPacket), 0.0);
}

void UDPController::stopArm()
{
	StopPacket stopPacket;
	ROS_INFO("Stopping the Arm");
	udpSender.send(&stopPacket, sizeof(stopPacket), 0.0);
}

bool UDPController::sendTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory,
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServer)
{
	TrajectoryPacket trajectoryPacket;
	// Fill packet
	size_t pathLength = 0;
	bool isSend = false;
	bool success = true;

	ROS_INFO("Start trajectory with %zu points", trajectory->trajectory.points.size());

	trajectoryPacket.pathLength = trajectory->trajectory.points.size();
	for (size_t i = 0; i < trajectory->trajectory.points.size(); i++)
	{
		isSend = false;
		for (size_t j = 0;
				j < trajectory->trajectory.points[i].positions.size(); j++)
		{
			trajectoryPacket.joints[i % 350][j] =
					trajectory->trajectory.points[i].positions[j];
			trajectoryPacket.joints[(i % 350) + 350][j] =
					(double) trajectory->trajectory.points[i].velocities[j];
			// TODO
			trajectoryPacket.joints[(i % 350) + 700][j] =
					(double) trajectory->trajectory.points[i].time_from_start.toSec();
		}

		pathLength++;
		if (pathLength % 350 == 0 && pathLength != 0)
		{
			udpSender.send(&trajectoryPacket, sizeof(trajectoryPacket), 0.0);
			int length = trajectoryPacket.pathLength;

			trajectoryPacket = TrajectoryPacket();
			trajectoryPacket.pathLength = length - 350;
			isSend = true;
		}
	}
	if (!isSend)
	{
		ROS_INFO("Package sent.");
		udpSender.send(&trajectoryPacket, sizeof(trajectoryPacket), 0.0);
	}

	// Wait for return
	ActionFinishedPacket packet;
	int length = -1;
	ros::Rate r(100.0);
	while (length == -1 && ros::ok())
	{
		length = udpSender.receive(&packet, sizeof(packet), 1);

		if (length != -1 && packet.isFinished)
		{
			ROS_INFO("Trajectory finished!");
			success = true;
			break;
		}

		r.sleep();
	}

	return success;
}

bool UDPController::sendTrajectoryAsync(const control_msgs::FollowJointTrajectoryGoalConstPtr& trajectory,
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* actionServer)
{
	TrajectoryPacket trajectoryPacket;
	// Fill packet
	size_t pathLength = 0;
	bool isSend = false;

	ROS_INFO("Start trajectory with %zu points", trajectory->trajectory.points.size());

	trajectoryPacket.pathLength = trajectory->trajectory.points.size();
	for (size_t i = 0; i < trajectory->trajectory.points.size(); i++)
	{
		isSend = false;
		for (size_t j = 0;
				j < trajectory->trajectory.points[i].positions.size(); j++)
		{
			trajectoryPacket.joints[i % 350][j] =
					trajectory->trajectory.points[i].positions[j];
			trajectoryPacket.joints[(i % 350) + 350][j] =
					(double) trajectory->trajectory.points[i].velocities[j];
			// TODO
			trajectoryPacket.joints[(i % 350) + 700][j] =
					(double) trajectory->trajectory.points[i].time_from_start.toSec();
		}

		pathLength++;
		if (pathLength % 350 == 0 && pathLength != 0)
		{
			udpSender.send(&trajectoryPacket, sizeof(trajectoryPacket), 0.0);
			int length = trajectoryPacket.pathLength;

			trajectoryPacket = TrajectoryPacket();
			trajectoryPacket.pathLength = length - 350;
			isSend = true;
		}
	}
	if (!isSend)
	{
		ROS_INFO("Package sent.");
		udpSender.send(&trajectoryPacket, sizeof(trajectoryPacket), 0.0);
	}

	return true;
}

void UDPController::sendRealTimeTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory)
{
	TrajectoryPacket trajectoryPacket;

	// Fill packet
	size_t pathLength = 0;
	bool isSend = false;

	ROS_INFO("Start trajectory with %zu points", trajectory->points.size());

	trajectoryPacket.pathLength = trajectory->points.size();
	for (size_t i = 0; i < trajectory->points.size(); i++)
	{
		isSend = false;
		for (size_t j = 0;
				j < trajectory->points[i].positions.size(); j++)
		{
			trajectoryPacket.joints[i % 350][j] =
					trajectory->points[i].positions[j];
			trajectoryPacket.joints[(i % 350) + 350][j] =
					(double) trajectory->points[i].velocities[j];
			// TODO
			trajectoryPacket.joints[(i % 350) + 700][j] =
					(double) trajectory->points[i].time_from_start.toSec();
		}

		pathLength++;
		if (pathLength % 350 == 0 && pathLength != 0)
		{
			udpSender.send(&trajectoryPacket, sizeof(trajectoryPacket), 0.0);
			int length = trajectoryPacket.pathLength;

			trajectoryPacket = TrajectoryPacket();
			trajectoryPacket.pathLength = length - 350;
			isSend = true;
		}
	}
	if (!isSend)
	{
		ROS_INFO("Package sent.");
		udpSender.send(&trajectoryPacket, sizeof(trajectoryPacket), 0.0);
	}
}

void UDPController::getJointMap(const std::vector<std::string>& names,
		const std::vector<double>& joints,
		std::unordered_map<std::string, double>& ids)
{
	assert(names.size() == joints.size());

	for (size_t i = 0; i < names.size(); ++i)
	{
		ids[names[i]] = joints[i];
	}
}
