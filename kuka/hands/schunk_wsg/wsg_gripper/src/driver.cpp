#include "ros/ros.h"
#include <ctime>
#include <iostream>
#include <string>
#include <wsg_gripper/WsgGripper.h>
#include <wsg_gripper/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>

// replace action interface by keyboard interface for testing
//#define KEYBOARD_INPUT

// command parser class for processing keyboard input
class CommandParser {
public:

  // command buffer
  #define USART_MAX_COMMAND_PARAMS 5
  #define USART_COMMAND_DELIMITER 10
  #define USART_PARAM_DELIMITER 32

  // buffer sizes
  #define USART_COMMAND_BUFFER_SIZE 100


  CommandParser() {
    ByteCount = 0;
    ParamCount = 0;
    CommandLength = 0;
    bCommandReceived = 0;
  }
  char Command[USART_COMMAND_BUFFER_SIZE];
  char* Param[USART_MAX_COMMAND_PARAMS];
  uint8_t ByteCount;
  uint8_t CommandLength;
  uint8_t ParamCount;
  uint8_t bCommandReceived;

  void onByteReceived(uint8_t Rec) {
    if (bCommandReceived) {
      // receive only one command at a time
      return;
    }
    ByteCount++;
    if ((Rec != USART_COMMAND_DELIMITER) && (Rec != USART_PARAM_DELIMITER)) {
      // add to command
      CommandLength++;
      Command[ByteCount - 1] = Rec;
    } else if (Rec == USART_COMMAND_DELIMITER) {
      Command[ByteCount - 1] = 0;
      bCommandReceived = 1;
    } else if (Rec == USART_PARAM_DELIMITER) {
      ParamCount++; // increase num of parameters
      Command[ByteCount - 1] = 0;
      Param[ParamCount - 1] = (char *) (Command + ByteCount); // set pointer to new parameter
    }
  }

  void CommandBufferInit() {
    ByteCount = 0;
    ParamCount = 0;
    CommandLength = 0;
    bCommandReceived = 0;
  }

  char* GetCommand(void) {
    return Command;
  }

  char* GetParam(uint8_t num) {
    if (num < NumParams()) {
      return Param[num];
    } else {
      return 0;
    }
  }

  uint8_t NumParams(void) {
    return ParamCount;
  }
};

class WsgGripperNode {
public:
  WsgGripperNode(ros::NodeHandle node, const char* gripper_ip, uint32_t gripper_tcp_port) :
          node(node),
          as_(node, std::string("WSG_Gripper_Action"),
              boost::bind(&WsgGripperNode::executeCB, this, _1), false) {
    try {
      gripper = new WsgGripper(gripper_ip, gripper_tcp_port, 10);
      gripper->ackFastStop();
      gripper->Homing();
    } catch (std::exception& e) {
      ROS_FATAL_STREAM("Error: " << e.what());
      ROS_ASSERT(0);
    }
    //ROS_ASSERT_MSG(gripper->connected_to_gripper,"Check the network connection.");
    ROS_INFO("Starting action server...");
    as_.start();
  }

  ros::NodeHandle node;
  ros::Publisher state_publisher;
  ros::Subscriber command_listener;
  const char* gripper_ip;
  uint32_t gripper_tcp_port;
  WsgGripper* gripper;

  std::string action_name_;
  actionlib::SimpleActionServer<wsg_gripper::GripperCommandAction> as_;
  wsg_gripper::GripperCommandFeedback feedback_;

  void executeCommand(std::string cmd, std::vector<double> params) {
    ROS_INFO("Executing command %s...", cmd.c_str());
    WsgPacket r(0);
    try {
      if (!cmd.compare("h")) {
        // first, ackowledge any stop conditions
        gripper->ackFastStop();
        gripper->Homing();
      } else if (!cmd.compare("a")) {
        gripper->ackFastStop();
      } else if (!cmd.compare("s")) {
        gripper->getSystemState();
      } else if (!cmd.compare("m")) {
        // first, ackowledge any stop conditions
        gripper->ackFastStop();
        if (params.size() == 2) {
          gripper->moveFingers(params.at(0), params.at(1));
        } else if (params.size() == 3) {
          gripper->moveFingers(params.at(0), params.at(1), params.at(2));
        } else if (params.size() == 4) {
          gripper->moveFingers(params.at(0), params.at(1), params.at(2), params.at(3));
        } else {
          throw std::runtime_error("Invalid number of arguments for command prepositionFingers!");
        }
      }
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error during command execution: " << e.what());
    }
  }

  void executeCB(const wsg_gripper::GripperCommandGoalConstPtr &goal) {
    wsg_gripper::GripperCommandResult result_;
    // publish info to the console for the user
    ROS_INFO("Executing gripper command code %i", goal->command_code);
    std::vector<double> params;
    uint32_t raw;
    uint32_t *pRaw;
    // start executing the action
    //GripperCommandGoal
    try {
      switch (goal->command_code) {
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
    } catch (std::exception& e) {
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
    if(gripper->cmd_successful){
      as_.setSucceeded(result_);
    }
    else{
      as_.setAborted(result_);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wsg_gripper_driver");
  ROS_INFO("Starting wsg gripper driver...");
  ros::NodeHandle node;

  // get gripper network parameters
  int gripper_tcp_port = 0;
  std::string gripper_ip("");
  node.param<std::string>("gripper_ip", gripper_ip, std::string("192.168.42.20"));
  node.param<int>("gripper_tcp_port", gripper_tcp_port, 6666);
  ROS_INFO("Connecting to gripper...");
  WsgGripperNode gripper(node, gripper_ip.c_str(), gripper_tcp_port);

  CommandParser parser;

  // main loop
  while (ros::ok()) {
    ros::spinOnce();
#ifdef KEYBOARD_INPUT
    uint8_t byte = std::cin.get();
    std::string cmd;
    if(std::cin.gcount() == 1) {
      parser.onByteReceived(byte);
    }
    if(parser.bCommandReceived) {
      ROS_DEBUG_NAMED("cmd_parser","Command received: %s", parser.GetCommand());
      ROS_DEBUG_NAMED("cmd_parser","# of Params: %d", parser.NumParams());
      cmd = std::string(parser.GetCommand());
      std::vector<double> params;
      for (int i = 0; i < parser.NumParams(); ++i) {
        ROS_DEBUG_STREAM_NAMED("cmd_parser","#" << i+1 << ": " << parser.GetParam(i));
        params.push_back(std::atof(parser.GetParam(i)));
      }
      parser.CommandBufferInit();
      gripper.executeCommand(cmd,params);
    }
#endif
  }
  return 0;
}

