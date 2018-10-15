#include "ros/ros.h"
#include <ctime>
#include <iostream>
#include <string>
#include <wsg_gripper/WsgGripper.h>
#include <wsg_gripper/GripperCommandAction.h>

class WsgGripperNodeMsg
{
private:
	ros::NodeHandle node;
	ros::Subscriber command_listener;
	WsgGripper* gripper;

public:
	WsgGripperNodeMsg(ros::NodeHandle node,
			const char* gripper_ip,
			uint32_t gripper_tcp_port) :
					node(node)
	{
		try
		{
			gripper = new WsgGripper(gripper_ip, gripper_tcp_port, 10);
			gripper->ackFastStop();
			gripper->Homing();
		}
		catch (std::exception& e)
		{
			ROS_FATAL_STREAM("Error: " << e.what());
			ROS_ASSERT(0);
		}
		//ROS_ASSERT_MSG(gripper->connected_to_gripper,"Check the network connection.");

		command_listener = node.subscribe<wsg_gripper::GripperCommandGoal>("gripper_command", 1, &WsgGripperNodeMsg::executeCB, this);
	}

	~WsgGripperNodeMsg()
	{
		delete gripper;
	}

	void executeCB(const wsg_gripper::GripperCommandGoalConstPtr &goal)
	{
		wsg_gripper::GripperCommandResult result_;
		// publish info to the console for the user
		ROS_INFO("Executing gripper command code %i", goal->command_code);
		std::vector<double> params;
		uint32_t raw;
		uint32_t *pRaw;
		// start executing the action
		//GripperCommandGoal
		try
		{
			switch (goal->command_code)
			{
				case wsg_gripper::GripperCommandGoal::HOMING:
					gripper->Homing();
					//executeCommand(std::string("h"), params);
					break;
				case wsg_gripper::GripperCommandGoal::MOVE_FINGERS:
					gripper->moveFingers(goal->width, goal->speed);
					break;
				case wsg_gripper::GripperCommandGoal::ACK_FAST_STOP:
					gripper->ackFastStop();
					break;
				case wsg_gripper::GripperCommandGoal::GRASP_A_PART:
					gripper->graspPart(goal->width, goal->speed);
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
		}
		catch (std::exception& e)
		{
			ROS_ERROR_STREAM("Error during command execution: " << e.what());
			return;
		}
		catch (gripperError& ex)
		{
			ROS_ERROR_STREAM("Error during command execution: " << ex.what());
			return;
		}
	}
};

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "wsg_gripper_driver_msg");
	ROS_INFO("Starting wsg gripper driver (msg)...");
	ros::NodeHandle node;

	// get gripper network parameters
	int gripper_tcp_port = 0;
	std::string gripper_ip("");
	node.param<std::string>("gripper_ip", gripper_ip, std::string("192.168.42.20"));
	node.param<int>("gripper_tcp_port", gripper_tcp_port, 6666);
	ROS_INFO("Connecting to gripper...");
	WsgGripperNodeMsg gripper(node, gripper_ip.c_str(), gripper_tcp_port);

	// main loop
	ros::spin();
	//this easts cpu
	//  while (ros::ok()) {
	//    ros::spinOnce();
	//  }
	return 0;
}

