#include <ros/ros.h>

#include "kuka_manager/kuka_robots.h"
#include "MoveItController.h"

using namespace std;
using namespace moveit;

Eigen::Matrix3f orientationOmnirob;
Eigen::Matrix3f orientationIIWA;
MoveItController* armControllerOmnirob;
MoveItController* armControllerIIWA;
KUKA::Robotname robot;
HandType handOmnirob = NONE;
HandType handIIWA = NONE;

bool goToInitPosition(MoveItController* controller, Eigen::Matrix3f orientation) {
	Eigen::Vector3f init_pos(0.6, 0.1, 0.4);
	if(!controller->goToPosition(init_pos, orientation)) {
		ROS_ERROR("Could not go to init position.");
		return false;
	}
	return true;
}

void omnirob_command(const std_msgs::String& input) {
	string command = input.data;

	if(!command.compare("home")) {
		ROS_INFO("Command home received for Omnirob");
		armControllerOmnirob->goHome();
	}
	else if(!command.compare("init")) {
		ROS_INFO("Command init received for Omnirob");
		goToInitPosition(armControllerOmnirob, orientationOmnirob);
	}
	else {
		ROS_INFO("Command goto received for Omnirob");
		string buff;
		stringstream stream(command);
		vector<string> tokens;
		while(stream >> buff) tokens.push_back(buff);
		Eigen::Vector3f new_pos;
		if(!tokens[0].compare("goto")) {
			string::size_type sz;
			new_pos << atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str());
		}
		armControllerOmnirob->goToPosition(new_pos, orientationOmnirob);
	}
}

void iiwa_command(const std_msgs::String& input) {
	string command = input.data;

	if(!command.compare("home")) {
		ROS_INFO("Command home received for IIWA");
		armControllerIIWA->goHome();
	}
	else if(!command.compare("init")) {
		ROS_INFO("Command init received for IIWA");
		goToInitPosition(armControllerIIWA, orientationIIWA);
	}
	else if(!command.compare("open")) {
		ROS_INFO("Command open received");
		armControllerIIWA->openGripper();
	}
	else if(!command.compare("close")) {
		ROS_INFO("Command close received");
		armControllerIIWA->closeGripper();
	}
	else {
		ROS_INFO("Command goto received for IIWA");
		string buff;
		stringstream stream(command);
		vector<string> tokens;
		while(stream >> buff) tokens.push_back(buff);
		Eigen::Vector3f new_pos;
		if(!tokens[0].compare("goto")) {
			string::size_type sz;
			new_pos << atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str());
		}
		armControllerIIWA->goToPosition(new_pos, orientationIIWA);
	}
}

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "moveit_controller_main");
	ros::NodeHandle nh;

	std::string robotname;
	nh.getParam("robot", robotname);
	if (robotname == "Omnirob") {
		robot = KUKA::Omnirob;
		robotname = "omnirob_lbr";
	} else if (robotname == "IIWA") {
		robot = KUKA::IIWA;
		robotname = "iiwa";
	} else if (robotname == "Both") {
		robot = KUKA::Both;
	} else {
		ROS_INFO("Robotname is not available..");
		ROS_INFO("You entered: %s", robotname.c_str());
		ROS_INFO("Available robots: [Omnirob] [IIWA] [Both]");
		return 1;
	}

	if(robot == KUKA::Omnirob || robot == KUKA::Both) {
		std::string gripperOmnirob;
		nh.getParam("gripperOmnirob", gripperOmnirob);
		if (gripperOmnirob == "Schunk_Hand") {
			handOmnirob = SCHUNK_HAND;
			/*orientationOmnirob << 0,1,0,
								  0,0,1,
								  1,0,0;*/
			orientationOmnirob << 1,0,0,
								  0,-1,0,
								  0,0,-1;
		} else if (gripperOmnirob == "WSG_Gripper") {
			handOmnirob = WSG_GRIPPER;
			orientationOmnirob << 1,0,0,
								  0,0,1,
								  0,-1,0;
		} else if (gripperOmnirob == "None") {
			handOmnirob = NONE;
			orientationOmnirob << 1,0,0,
								  0,1,0,
								  0,0,1;
		} else {
			ROS_INFO("[Warning]: You did not enter a valid gripper for Omnirob");
			ROS_INFO("You entered: %s", gripperOmnirob.c_str());
			ROS_INFO("Available grippers: [Schunk_Hand] [WSG_Gripper] [None]");
			ROS_INFO("Assuming there is no gripper.");
			handOmnirob = NONE;
			orientationOmnirob << 1,0,0,
								  0,1,0,
								  0,0,1;
		}
	}

	if(robot == KUKA::IIWA || robot == KUKA::Both) {
		std::string gripperIIWA;
		nh.getParam("gripperIIWA", gripperIIWA);
		if (gripperIIWA == "Schunk_Hand") {
			handIIWA = SCHUNK_HAND;
			orientationIIWA << 0,1,0,
							   0,0,1,
							   1,0,0;
		} else if (gripperIIWA == "WSG_Gripper") {
			handIIWA = WSG_GRIPPER;
			orientationIIWA << 1,0,0,
							   0,0,1,
							   0,-1,0;
		} else if (gripperIIWA == "None") {
			handIIWA = NONE;
			orientationIIWA << 1,0,0,
							   0,1,0,
							   0,0,1;
		} else {
			ROS_INFO("[Warning]: You did not enter a valid gripper for Omnirob");
			ROS_INFO("You entered: %s", gripperIIWA.c_str());
			ROS_INFO("Available grippers: [Schunk_Hand] [WSG_Gripper] [None]");
			ROS_INFO("Assuming there is no gripper.");
			handIIWA = NONE;
			orientationIIWA << 1,0,0,
							   0,1,0,
							   0,0,1;
		}
	}

	if (robot == KUKA::Omnirob) {
		MoveItController omnirobControl("omnirob_lbr", handOmnirob);
		omnirobControl.init(nh);
		armControllerOmnirob = &omnirobControl;

		ROS_INFO("ArmController for Omnirob successfully loaded. "
				"Now waiting command on rostopic /omnirob_command");
		ros::Subscriber help_command = nh.subscribe("/omnirob_command", 1, omnirob_command);

		ros::AsyncSpinner spinner(1);
		spinner.start();
		ros::spin();
	} else if (robot == KUKA::IIWA) {
		MoveItController iiwaControl("iiwa", handIIWA);
		iiwaControl.init(nh);
		armControllerIIWA = &iiwaControl;

		ROS_INFO("ArmController for IIWA successfully loaded. "
				"Now waiting command on rostopic /iiwa_command");
		ros::Subscriber help_command = nh.subscribe("/iiwa_command", 1, iiwa_command);

		ros::AsyncSpinner spinner(1);
		spinner.start();
		ros::spin();
	} else if (robot == KUKA::Both) {
		MoveItController omnirobControl("omnirob_lbr", handOmnirob);
		omnirobControl.init(nh);
		armControllerOmnirob = &omnirobControl;
		ROS_INFO("ArmController for Omnirob successfully loaded. "
				"Now waiting for command on rostopic /omnirob_command");
		ros::Subscriber help_command_omnirob = nh.subscribe("/omnirob_command", 1, omnirob_command);

		MoveItController iiwaControl("iiwa", handIIWA);
		iiwaControl.init(nh);
		armControllerIIWA = &iiwaControl;
		ROS_INFO("ArmController for IIWA successfully loaded. "
				"Now waiting for command on rostopic /iiwa_command");
		ros::Subscriber help_command_iiwa = nh.subscribe("/iiwa_command", 1, iiwa_command);

		ros::AsyncSpinner spinner(1);
		spinner.start();
		ros::spin();
	}

	return 0;
}
