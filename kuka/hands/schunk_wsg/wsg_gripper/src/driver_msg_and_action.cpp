#include "ros/ros.h"
#include <ctime>
#include <iostream>
#include <string>
#include <wsg_gripper/WsgGripper.h>
#include <wsg_gripper/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>

class WsgGripperNode
{
public:
	WsgGripperNode(ros::NodeHandle node,
			std::string gripperip,
			uint32_t grippertcpport,
			bool close) :
					node(node),
					gripper(NULL),
					gripper_ip(gripperip),
					gripper_tcp_port(grippertcpport),
					closeGripperAfterStart(close),
					as_(node, std::string("WSG_Gripper_Action"),
							boost::bind(&WsgGripperNode::executeActionCB, this, _1), false)
	{
		//ROS_ASSERT_MSG(gripper->connected_to_gripper,"Check the network connection.");
		init_driver();
		ROS_INFO("Starting action server...");
		as_.start();
		command_listener = node.subscribe<wsg_gripper::GripperCommandGoal>("gripper_command", 1, &WsgGripperNode::executeMsgCB, this);

		state_publisher = node.advertise<sensor_msgs::JointState>("joint_states", 10);
		joints.name.resize(2);
		joints.position.resize(2);
		joints.name[0] = "wsg_gripper_l_tip";
		joints.name[1] = "wsg_gripper_r_tip";
	}

	void publishState()
	{
		if (gripper == NULL)
			return;
		double width = gripper->opening_width / 1000.0;
		joints.position[0] = joints.position[1] = width / 2.0;
		joints.header.stamp = ros::Time::now();
		state_publisher.publish(joints);
	}

	void reinit_driver()
	{
		delete gripper;
		gripper = NULL;
		init_driver();
	}

	void init_driver()
	{
		ROS_INFO("Initializing gripper driver");
		try
		{
			gripper = new WsgGripper(gripper_ip.c_str(), gripper_tcp_port, 10);
			gripper->ackFastStop();
			gripper->Homing();
			if (closeGripperAfterStart) {
				gripper->graspPart(25, 50);
			}
			gripper->getOpeningWidth();
			ROS_INFO("Driver active.");
		}
		catch (std::exception& e)
		{
			ROS_FATAL_STREAM("Error in initialization: " << e.what());
			ros::shutdown();
			ROS_ASSERT(0);
		}
	}

	void executeCommand(std::string cmd,
			std::vector<double> params)
	{
		ROS_INFO("Executing command %s...", cmd.c_str());
		WsgPacket r(0);
		if (!cmd.compare("h"))
		{
			// first, ackowledge any stop conditions
			gripper->ackFastStop();
			gripper->Homing();
			gripper->getOpeningWidth();
		}
		else if (!cmd.compare("a"))
		{
			gripper->ackFastStop();
		}
		else if (!cmd.compare("s"))
		{
			gripper->getSystemState();
		}
		else if (!cmd.compare("m"))
		{
			// first, ackowledge any stop conditions
			gripper->ackFastStop();
			if (params.size() == 2)
			{
				gripper->moveFingers(params.at(0), params.at(1));
				gripper->getOpeningWidth();
			}
			else if (params.size() == 3)
			{
				gripper->moveFingers(params.at(0), params.at(1), params.at(2));
				gripper->getOpeningWidth();
			}
			else if (params.size() == 4)
			{
				gripper->moveFingers(params.at(0), params.at(1), params.at(2), params.at(3));
				gripper->getOpeningWidth();
			}
			else
			{
				throw std::runtime_error("Invalid number of arguments for command prepositionFingers!");
			}
		}
	}

	void executeMsgCB(const wsg_gripper::GripperCommandGoalConstPtr &goal)
	{
		ROS_INFO("Received Gripper Command via Message");
		try
		{
			wsg_gripper::GripperCommandResult result_ = executeCallbackCommon(goal);
		}
		catch (gripperError& ge)
		{
//			gripper->getOpeningWidth();
			ROS_ERROR_STREAM("Error during command execution: " << ge.what());
		}
		catch (std::exception& e)
		{
			ROS_FATAL_STREAM("Error during command execution: " << e.what() << "\nReinitializing.");
			reinit_driver();
		}
		catch (...)
		{
			ROS_FATAL("Unknown exception, shutting down.");
			ros::shutdown();
		}
	}

	//Execute command received from action. Set result appropriately
	void executeActionCB(const wsg_gripper::GripperCommandGoalConstPtr &goal)
	{
		ROS_INFO("Received Gripper Command via Action");
		wsg_gripper::GripperCommandResult result_;
		try
		{
			result_ = executeCallbackCommon(goal);
		}
		catch (gripperError& ge)
		{
//			gripper->getOpeningWidth();
			ROS_ERROR_STREAM("Error during command execution: " << ge.what());
			result_.executed_command = goal->command_code;
			result_.successful = false;
			result_.return_code = gripper->last_error;
			as_.setAborted(result_);
			return;
		}
		catch (std::exception& e)
		{
			gripper->getOpeningWidth();
			ROS_ERROR_STREAM("Error during command execution: " << e.what());
			result_.executed_command = goal->command_code;
			result_.successful = false;
			result_.return_code = gripper->last_error;
			as_.setAborted(result_);
			return;
		}
		result_.executed_command = goal->command_code;
		result_.successful = gripper->cmd_successful;
		result_.return_code = gripper->last_error;
		if (gripper->cmd_successful)
		{
			as_.setSucceeded(result_);
		}
		else
		{
			as_.setAborted(result_);
		}
	}

	wsg_gripper::GripperCommandResult executeCallbackCommon(const wsg_gripper::GripperCommandGoalConstPtr &goal)
	{
		wsg_gripper::GripperCommandResult result_;
		ROS_INFO("Executing gripper command code %i", goal->command_code);
		std::vector<double> params;
		uint32_t raw;
		uint32_t *pRaw;
		// start executing the action
		//GripperCommandGoal
		switch (goal->command_code)
		{
			case wsg_gripper::GripperCommandGoal::HOMING:
				gripper->Homing();
				//executeCommand(std::string("h"), params);
				break;
			case wsg_gripper::GripperCommandGoal::MOVE_FINGERS:
				gripper->moveFingers(goal->width, goal->speed);
				gripper->getOpeningWidth();
				break;
			case wsg_gripper::GripperCommandGoal::ACK_FAST_STOP:
				gripper->ackFastStop();
				break;
			case wsg_gripper::GripperCommandGoal::GRASP_A_PART:
				gripper->graspPart(goal->width, goal->speed);
				gripper->getOpeningWidth();
				break;
			case wsg_gripper::GripperCommandGoal::MOVE_WITHOUT_FORCE_CONTROL:
				gripper->releasePart(goal->width, goal->speed);
				gripper->getOpeningWidth();
				break;
			case wsg_gripper::GripperCommandGoal::SET_ACCELERATION:
				gripper->setAcceleration(goal->acc);
				break;
			case wsg_gripper::GripperCommandGoal::GET_ACCELERATION:
				gripper->getAcceleration();
				result_.acc_limit = gripper->acc_limit;
				break;
			case wsg_gripper::GripperCommandGoal::SET_GRASPING_FORCE_LIMIT:
				gripper->setGraspingForceLimit(goal->force);
				break;
			case wsg_gripper::GripperCommandGoal::GET_GRASPING_FORCE_LIMIT:
				gripper->getGraspingForceLimit();
				result_.force_limit = gripper->force_limit;
				break;
			case wsg_gripper::GripperCommandGoal::GET_SYSTEM_STATE:
				gripper->getSystemState();
				result_.system_state = gripper->system_state;
				break;
			case wsg_gripper::GripperCommandGoal::GET_OPENING_WIDTH:
				gripper->getOpeningWidth();
				result_.opening_width = gripper->opening_width;
				break;
			case wsg_gripper::GripperCommandGoal::GET_FINGER_INFO:
				gripper->getFingerInfo(goal->finger_index);
				result_.finger_type = gripper->finger_type;
				result_.finger_data_size = gripper->finger_data_size;
				break;
			case wsg_gripper::GripperCommandGoal::GET_FINGER_FLAGS:
				gripper->getFingerFlags(goal->finger_index);
				result_.finger_flags = gripper->finger_flags;
				break;
			case wsg_gripper::GripperCommandGoal::FINGER_POWER_CONTROL:
				gripper->fingerPowerControl(goal->finger_index, goal->on_off);
				break;
			case wsg_gripper::GripperCommandGoal::GET_FINGER_DATA:
				gripper->getFingerData(goal->finger_index);
				result_.finger_data = gripper->finger_data;
				// get force
				raw = (gripper->finger_data.at(2) + (gripper->finger_data.at(3) << 8)
						+ (gripper->finger_data.at(4) << 16) + (gripper->finger_data.at(5) << 24));
				pRaw = &raw;
				result_.finger_force = *(reinterpret_cast<float *>(pRaw));
				break;
			default:
				ROS_WARN("Unknown gripper command.");
				break;
		}
		return result_;
	}

	ros::NodeHandle node;
	ros::Publisher state_publisher;
	sensor_msgs::JointState joints;
	ros::Subscriber command_listener;
	std::string gripper_ip;
	uint32_t gripper_tcp_port;
	WsgGripper* gripper;

	std::string action_name_;
	actionlib::SimpleActionServer<wsg_gripper::GripperCommandAction> as_;
	wsg_gripper::GripperCommandFeedback feedback_;

	bool closeGripperAfterStart;
};

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "wsg_gripper_driver");
	ROS_INFO("Starting wsg gripper driver...");
	ros::NodeHandle node;

	bool close = false;
	if (argc > 1 && std::string(argv[1]) == "true")
	{
		close = true;
	}


	// get gripper network parameters
	int gripper_tcp_port = 0;
	std::string gripper_ip("");
	node.param<std::string>("gripper_ip", gripper_ip, std::string("192.168.42.20"));
	node.param<int>("gripper_tcp_port", gripper_tcp_port, 6666);
	ROS_INFO("Connecting to gripper...");
	WsgGripperNode gripper(node, gripper_ip.c_str(), gripper_tcp_port, close);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::Rate r(100);
	while (ros::ok())
	{
		gripper.publishState();
		r.sleep();
	}

	return 0;
}

