#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <wsg_gripper/GripperCommandAction.h>

// command buffer
#define USART_MAX_COMMAND_PARAMS 5
#define USART_COMMAND_DELIMITER 10
#define USART_PARAM_DELIMITER 32

// buffer sizes
#define USART_COMMAND_BUFFER_SIZE 100

// command parser class for processing keyboard input
class CommandParser {
public:
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "wsg_gripper_client");
  ROS_INFO("Starting wsg gripper client...");
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<wsg_gripper::GripperCommandAction> ac("WSG_Gripper_Action", true);
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("done.");
  wsg_gripper::GripperCommandGoal goal;

  CommandParser parser;

  // main loop
  while (ros::ok()) {
    ros::spinOnce();
    uint8_t byte = std::cin.get();
    std::string cmd;
    if (std::cin.gcount() == 1) {
      parser.onByteReceived(byte);
    }
    if (parser.bCommandReceived) {
      ROS_DEBUG_NAMED("cmd_parser", "Command received: %s", parser.GetCommand());
      ROS_DEBUG_NAMED("cmd_parser", "# of Params: %d", parser.NumParams());
      cmd = std::string(parser.GetCommand());
      std::vector<double> params;
      for (int i = 0; i < parser.NumParams(); ++i) {
        ROS_DEBUG_STREAM_NAMED("cmd_parser", "#" << i+1 << ": " << parser.GetParam(i));
        params.push_back(std::atof(parser.GetParam(i)));
      }
      parser.CommandBufferInit();
      ROS_INFO("Executing command %s...", cmd.c_str());
      if (!cmd.compare("h")) {
        // first, ackowledge any stop conditions
        goal.command_code = wsg_gripper::GripperCommandGoal::ACK_FAST_STOP;
        ac.sendGoal(goal);
        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
        if (finished_before_timeout) {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s", state.toString().c_str());
        } else
          ROS_INFO("Action did not finish before the time out.");
        goal.command_code = wsg_gripper::GripperCommandGoal::HOMING;
      } else if (!cmd.compare("a")) {
        //gripper->ackFastStop();
      } else if (!cmd.compare("system_state")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_SYSTEM_STATE;
      } else if (!cmd.compare("acc")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_ACCELERATION;
      } else if (!cmd.compare("set_acc")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::SET_ACCELERATION;
        goal.acc = params.at(0);
      } else if (!cmd.compare("force")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_GRASPING_FORCE_LIMIT;
      } else if (!cmd.compare("set_force")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::SET_GRASPING_FORCE_LIMIT;
        goal.force = params.at(0);
      } else if (!cmd.compare("finger_info")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_FINGER_INFO;
        goal.finger_index = params.at(0);
      } else if (!cmd.compare("finger_power")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::FINGER_POWER_CONTROL;
        goal.finger_index = params.at(0);
        goal.on_off = params.at(1);
      } else if (!cmd.compare("finger_flags")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_FINGER_FLAGS;
        goal.finger_index = params.at(0);
      } else if (!cmd.compare("finger_data")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_FINGER_DATA;
        goal.finger_index = params.at(0);
      } else if (!cmd.compare("width")) {
        goal.command_code = wsg_gripper::GripperCommandGoal::GET_OPENING_WIDTH;
      } else if (!cmd.compare("grasp")) {
        // first, ackowledge any stop conditions
        goal.command_code = wsg_gripper::GripperCommandGoal::ACK_FAST_STOP;
        ac.sendGoal(goal);
        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
        if (finished_before_timeout) {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s", state.toString().c_str());
        } else
          ROS_INFO("Action did not finish before the time out.");
        goal.command_code = wsg_gripper::GripperCommandGoal::GRASP_A_PART;
        goal.width = params.at(0);
        goal.speed = params.at(1);
      } else if (!cmd.compare("m")) {
        // first, ackowledge any stop conditions
        goal.command_code = wsg_gripper::GripperCommandGoal::ACK_FAST_STOP;
        ac.sendGoal(goal);
        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
        if (finished_before_timeout) {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s", state.toString().c_str());
        } else
          ROS_INFO("Action did not finish before the time out.");
        goal.command_code = wsg_gripper::GripperCommandGoal::MOVE_FINGERS;
        goal.width = params.at(0);
        goal.speed = params.at(1);
      } else {
        throw std::runtime_error("unknown command!");
      }

      ac.sendGoal(goal);
//wait for the action to return
      bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
      if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
      } else
        ROS_INFO("Action did not finish before the time out.");
    }
  }
  return 0;
}
