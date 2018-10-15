/*
 * MoveItController.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: Ingo Killmann
 */

#include "MoveItController.h"

using namespace std;
using namespace moveit;
using moveit_msgs::RobotTrajectory;

const string MoveItController::FINISHED_ERROR = "movement_error";       //message, when movement did not work
const string MoveItController::FINISHED_SUCCESS = "movement_finished";  //message, when movement was successfully

/**
 * Constructor
 * MoveIt! initialization
 * Something that can be changed (Which planner should be used, how much time for planning...)
 * Sets up the move group for the robot with name $robot_name
 *
 * robot_name: The name of the robotic arm, which will be controlled
 */
MoveItController::MoveItController(string robot_name, HandType hand)
	: group(moveit::planning_interface::MoveGroup::Options(robot_name, "robot_description")) {
	if(robot_name == "iiwa") {
		name_prefix = "iiwa";
	} else {
		name_prefix = "lbr";
	}

	group.setPoseReferenceFrame(name_prefix + "_0_link");
	group.setNumPlanningAttempts(1000);
	group.setPlanningTime(60);
	group.setStartStateToCurrentState();
	group.setGoalTolerance(0.01);
	group.setPlannerId("RRTstarkConfigDefault");

	// robot_model_loader::RobotModelLoader robot_model_loader("iiwa/robot_description");
	// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

	// const boost::shared_ptr< const srdf::Model > &srdf_robot = kinematic_model->getSRDF();
	// const std::vector< srdf::Model::VirtualJoint > &virtual_joint = srdf_robot->getVirtualJoints();

	// std::cout << virtual_joint[0].parent_frame_ << std::endl;
	// std::cout << virtual_joint[0].child_link_ << std::endl;

	// std::cout << kinematic_model->getModelFrame() << std::endl;

	std::cout << "Planning frame: "<< group.getPlanningFrame() << std::endl;

	this->hand = hand;
	this->robot_name = robot_name;
}

/**
 * Destructor
 */
MoveItController::~MoveItController() {
}

/**
 * Initialize ROS for Arm Controller
 * Needs this to publish, when movement is finished
 * And when there is a hand at the arm
 * This is just for Schunk hand right now
 *
 * nh: The NodeHandle for the ROS communication
 */
void MoveItController::init(ros::NodeHandle &nh) {

	this->nh = nh;
	if(hand == SCHUNK_HAND) {
		hand_client = nh.serviceClient<sdh2_hand::SDHAction>("sdh_action");
	}
	else if(hand == WSG_GRIPPER) {
		wsg_pub = nh.advertise<wsg_gripper::GripperCommandGoal>("gripper_command",1);
	}
	pub_finish = nh.advertise<std_msgs::String> (name_prefix + "_arm_movement_finished",1);
	// spin();

}

/**
 * Start the ROS spinning for the controller
 * This is required since the ArmController is communicating with MoveIt! Framework
 */
void MoveItController::spin() {
	ros::AsyncSpinner spinner(4);
	spinner.start();
}

/**
 * Executes a moveit-plan through the move_group
 * And publishes it on rostopic "arm_movement_finished" when it worked
 *
 * plan: The plan to be executed
 *
 * return: 0 When everything is fine
 * 		   1 When something went wrong
 */
int MoveItController::executePlan(planning_interface::MoveGroup::Plan& plan) {

	bool moved = false;
	moved = group.execute(plan);
	if(!moved)  {
		ROS_INFO("Could not execute desired plan, sorry.");
		return 1;
	}
	else {
		pub_finished(FINISHED_SUCCESS);
		return 0;
	}

}

/**
 * Executes a plan from the savedPlans of the ArmController
 * Checks if plan exists, if not it just returns
 *
 * plan: The name of the plan to be executed
 * recover: The place the arm should go back to, if executing fails
 * orientation: The orientation belonging to the recover-position
 */
void MoveItController::executePlan(string plan, Eigen::Vector3f& recover, Eigen::Matrix3f& orientation) {

	if(!checkPlan(plan)) {
		ROS_INFO("Plan asked for execution was not found");
		return;
	}
	planning_interface::MoveGroup::Plan myplan = savedPlans[plan];
	if(executePlan(myplan) == 1) {
		goToPosition(recover, orientation);
	}
}

/**
 * Calculates a plan out of waypoints
 * Publishes an error on rostopic "arm_movement_finished", when calculating went wrong
 *
 * waypoints: The waypoints, the plan should follow, when it only is able to follow
 * 				under 80% of the waypoints, it detects it as an error
 * plan: The plan that will get calculate, just give an empty plan, but it also overwrites
 *
 * return: 0 When everything is fine
 * 		   1 When it was not able to compute any path
 * 		   2 When it found a solution with less than 80% of the given waypoints
 */
int MoveItController::calculatePlan(std::vector<Eigen::Affine3d>& waypoints,
		planning_interface::MoveGroup::Plan& plan) {

	std::vector<geometry_msgs::Pose> wayPointsPoses(waypoints.size());
	for(int i = 0; i < waypoints.size(); i++) {
		tf::poseEigenToMsg(waypoints[i], wayPointsPoses[i]);
	}

	moveit_msgs::RobotTrajectory traj;
	double fraction = group.computeCartesianPath(wayPointsPoses, 0.1, 1000, traj);

	if(fraction < 0) {
		ROS_INFO("Could not compute cartesian path. Sorry!");
		return 1;
		pub_finished(FINISHED_ERROR);
	} else if(fraction < 0.8) {
		ROS_INFO("Only a path through %3.2f%% of the trajectory was found", fraction * 100);
		return 2;
		pub_finished(FINISHED_ERROR);
	}

	robot_trajectory::RobotTrajectory trajectory(group.getCurrentState()->getRobotModel(),
			group.getName());
	trajectory.setRobotTrajectoryMsg(*group.getCurrentState(),traj);
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	bool success = iptp.computeTimeStamps(trajectory);

	makePlanOfTrajectory(trajectory, plan);
	lastPlan = plan;
	return 0;
}

/**
 * Saves the last plan, that was calculated in global Hashmap savedPlans
 *
 * name: The name for the plan (this is the Hashtag to use for retrieving the plan again)
 */
void MoveItController::saveLastPlan(string name) {

	savedPlans[name] = lastPlan;

}

/**
 * Deletes a plan in the Hashmap savedPlans
 *
 * plan: The name of the plan to be deleted
 */
void MoveItController::deletePlan(string plan) {

	if(!checkPlan(plan)) {
		ROS_INFO("Plan to delete not found.");
		return;
	}
	savedPlans.erase(plan);
}

/**
 * Clears the Hashmap savedPlans
 */
void MoveItController::eraseAllPlans() {

	savedPlans.clear();

}

/**
 * Creates a plan out of a trajectory
 *
 * trajectory: The trajectory, from which the plan is build
 * plan: The Variable, where the plan will be saved in
 */
void MoveItController::makePlanOfTrajectory(robot_trajectory::RobotTrajectory& trajectory,
		planning_interface::MoveGroup::Plan& plan) {

	moveit_msgs::RobotTrajectory robot_traj;
	trajectory.getRobotTrajectoryMsg(robot_traj);
	plan.trajectory_ = robot_traj;

}

/**
 * Concatenate two plans and save it in the Hashmap savedPlans for later use
 * It checks if the two plans that should get concatenated are available in the Hashmap savedPlans
 * If not it just returns
 *
 * plan1: The name of the first plan, that will be at the front
 * plan2: The name of the second plan, that will be at the end
 * combined_plan: The name of the concatenated plan
 * 				  It will be saved in the Hashmap under this name for later use
 */
void MoveItController::concatPlans(string plan1, string plan2, string combined_plan) {

	if(!checkPlan(plan1)) {
		ROS_INFO("Plan 1 to concatenate not found.");
		return;
	}
	if(!checkPlan(plan2)) {
		ROS_INFO("Plan 2 to concatenate not found.");
		return;
	}
	robot_state::RobotState state(group.getCurrentState()->getRobotModel());
	robot_trajectory::RobotTrajectory combined_traj(group.getCurrentState()->getRobotModel(),
			group.getName());
	combined_traj.setRobotTrajectoryMsg(state, savedPlans[plan1].trajectory_);
	robot_trajectory::RobotTrajectory second_traj(group.getCurrentState()->getRobotModel(),
			group.getName());
	second_traj.setRobotTrajectoryMsg(state, savedPlans[plan2].trajectory_);
	combined_traj.append(second_traj, 0);
	planning_interface::MoveGroup::Plan new_plan;
	makePlanOfTrajectory(combined_traj, new_plan);
	savedPlans[combined_plan] = new_plan;

}

/**
 * Reverses a plan and saves it as a new one in the Hashmap savedPlans
 * Checks if the plan to be reserved is available, if not it just returns
 *
 * plan: The name of the plan in the Hashmap to be reversed
 * save_name: The name, the reversed plan will be saved in, in the Hashmap for later use
 */
void MoveItController::reversePlan(string plan, string save_name) {

	if(!checkPlan(plan)) {
		ROS_INFO("Plan to reverse not found.");
		return;
	}
	robot_trajectory::RobotTrajectory trajectory(group.getCurrentState()->getRobotModel(),
			group.getName());
	robot_state::RobotState state(group.getCurrentState()->getRobotModel());
	trajectory.setRobotTrajectoryMsg(state, savedPlans[plan].trajectory_);
	trajectory.reverse();
	planning_interface::MoveGroup::Plan new_plan;
	makePlanOfTrajectory(trajectory, new_plan);
	savedPlans[save_name] = new_plan;

}

/**
 * Checks if a plan is already available in the Hashmap savedPlans
 *
 * plan: The name of the plan to be checked
 *
 * return: True, if plan is available in Hashmap savedPlans
 * 		   False, if the plan is not available in the Hashmap
 */
bool MoveItController::checkPlan(string plan) {

	map<string,planning_interface::MoveGroup::Plan>::iterator it = savedPlans.find(plan);
	if(it == savedPlans.end()) {
		return false;
	} else {
		return true;
	}

}

/**
 * Creates a rotation and translation matrix out of the input
 * It is a 4x4 matrix with the upper left 3x3 block as the rotation
 * And the very right 1x3 block as the translation
 * The last line is always 0 0 0 1
 *
 * position: The translation, in the new matrix at the very right upper 1x3 block
 * orientation: The orientation, in the new matrix the left upper 3x3 block
 * matrix: The new matrix, can be given empty or with something in it (gets overwritten)
 */
void MoveItController::createRotTransMatrix(Eigen::Vector3f& position, Eigen::Matrix3f& orientation,
		Eigen::Matrix4d& matrix) {

	matrix << orientation(0,0), orientation(0,1), orientation(0,2), position(0),
			orientation(1,0), orientation(1,1), orientation(1,2), position(1),
			orientation(2,0), orientation(2,1), orientation(2,2), position(2),
			0,0,0,1;
}

/**
 * Sends arm to a new position. MoveIt! Planner plans the whole motion,
 * There are no constraints on the arm, except for the input
 *
 * position: The position, that the End-Effector of the arm should have after moving
 * 			 In x, y and z coordinates respectively to the base_frame of the robot
 * 			 In this case "manipulator_base_frame"
 * orientation: The orientation, that the End-Effector of the arm should have after moving
 * 				As a 3x3 Matrix
 */
bool MoveItController::goToPosition(Eigen::Vector3f& position, Eigen::Matrix3f& orientation) {

	Eigen::Matrix4d target;
	createRotTransMatrix(position,orientation,target);
	Eigen::Affine3d targetAffine;
	targetAffine.matrix() = target;
	group.setPoseTarget(targetAffine);
	bool moved = false;
	moved = group.move();
	std::cout << "moved" << std::endl << std::flush;
	if(moved) {
		pub_finished(FINISHED_SUCCESS);
	}
	else {
		pub_finished(FINISHED_ERROR);
	}

	return moved;

}

bool MoveItController::goToPosition(Eigen::Affine3d& position) {

	group.setPoseTarget(position);
	return group.move();

}

bool MoveItController::goToPositions(EigenSTL::vector_Affine3d& positions) {

	bool moved = false;
	group.setPoseTargets(positions);
	int size = 10000000;
	planning_interface::MoveGroup::Plan myplan;
	planning_interface::MoveGroup::Plan temp_plan;
	int error = group.plan(myplan);
	if(error == 1) {
		int counter = 0;
		for(size_t i=0; i<4;i++) {
			int err = group.plan(temp_plan);
			ROS_INFO("num waypoints: %d", (int)temp_plan.trajectory_.joint_trajectory.points.size());
			if(err == 1) {
				++counter;
			} else {
				continue;
			}
			if(size > temp_plan.trajectory_.joint_trajectory.points.size()) {
				myplan = temp_plan;
				size = temp_plan.trajectory_.joint_trajectory.points.size();
			}
		}
		if(counter >=3) {
			moved = group.execute(myplan);
			pub_finished(FINISHED_SUCCESS);
		}
	}
	return moved;
}

/**
 * Drives the arm to the desired location, but is flexible in the orientation
 * The orientation to begin with is still hard-coded (need to be changed)
 * This orientation will be the first try, then the arm tries to oscillate left and right of the position
 * It is a method for auto-drinking and was not used for other stuff yet
 *
 * position: Desired location of the End-Effector (x, y, z coordinates in base_frame)
 * lengthHand: Length of the End-Effector that is over the End-Effector position
 * rad: The radius between desired location and End-Effector
 * angle: The angle from where to start the oscillation
 *
 * return: The angle that it found a plan for
 */
double MoveItController::goToPositionWithFlexAngle(Eigen::Vector3f& position, Eigen::Matrix3f& orientation,
		double lengthHand, double rad, double angle) {

	Eigen::Matrix4d pos_matrix;
	Eigen::Matrix3f z_turn;
	Eigen::Matrix3f rotation;
	Eigen::Vector3f new_position;
	Eigen::Affine3d pos;
	double alpha = angle;
	bool moved = false;
	int toggler = 1;
	int counter = 1;
	double radius = lengthHand + rad;
	while(!moved) {
		new_position(0) = position(0) + sin(alpha) * radius;
		new_position(1) = position(1) + lengthHand - cos(alpha) * radius;
		new_position(2) = position(2);
		z_turn << cos(alpha), -sin(alpha), 0,
				  sin(alpha), cos(alpha), 0,
				  0, 0, 1;
		rotation = z_turn * orientation;
		createRotTransMatrix(new_position,rotation,pos_matrix);
		pos.matrix() = pos_matrix;
		group.setPoseTarget(pos);
		int size = 10000000;
		planning_interface::MoveGroup::Plan myplan;
		planning_interface::MoveGroup::Plan temp_plan;
		int error = group.plan(myplan);
		if(error == 1) {
			int counter = 0;
			for(size_t i=0; i<4;i++) {
				int err = group.plan(temp_plan);
				ROS_INFO("num waypoints: %d", (int)temp_plan.trajectory_.joint_trajectory.points.size());
				if(err == 1) {
					++counter;
				} else {
					continue;
				}
				if(size > temp_plan.trajectory_.joint_trajectory.points.size()) {
					myplan = temp_plan;
					size = temp_plan.trajectory_.joint_trajectory.points.size();
				}
			}
			if(counter >=3) {
				moved = group.execute(myplan);
				pub_finished(FINISHED_SUCCESS);
			}
		}
		alpha += 0.08 * toggler;
		toggler += counter;
		toggler *= -1;
		counter *= -1;
		if (alpha >= 1.8) {
			ROS_INFO("No Plan was found for this target. Sorry!");
			pub_finished(FINISHED_ERROR);
			break;
		}
	}

	return alpha;
}

/**
 * Computes waypoints in 3D-space between the two position on a straight line
 *
 * start: Start position for the waypoints (in xyz-coordinates in base_frame)
 * target: End position for the waypoints (in xyz-coordinates in base_frame)
 * orientation: The orientation of the End-Effector at all the waypoints
 * num_waypoints: The number of waypoints that should be created on the given line
 * waypoints: Vector of waypoints, where the solution is stored
 */
void MoveItController::computeWaypointsStraight(Eigen::Vector3f& start,
		Eigen::Vector3f& target, Eigen::Matrix3f& orientation, int num_waypoints,
		std::vector<Eigen::Affine3d>& waypoints) {

	Eigen::Matrix4d waypoint;

	for(double i=1; i <= num_waypoints; i++) {
		Eigen::Vector3f diff;
		diff = target - start;
		diff = start + diff * (i/num_waypoints);
		createRotTransMatrix(diff,orientation,waypoint);
		Eigen::Affine3d waypointAffine;
		waypointAffine.matrix() = waypoint;
		waypoints.push_back(waypointAffine);
	}
}

/**
 * Computes waypoints on a circular line, beginning from the start point and keeping a position the same
 * It is still with a fixed starting orientation (coming from auto_drinking) need to be changed
 *
 * start: The start point of the End-Effector
 * xOffset: The offset in x-direction to the point, which should stay in place (the turning point/center)
 * zOffset: The offset in z-direction to the point, which should stay in place (the turning point/center)
 * angleTransPoints: The angle between those two points
 * num_waypoints: The number of waypoints, that should be calculated
 * 				  This also determines on how much the End-Effector gets turned
 * 				  In degrees: (num_waypoints-1)*10
 * waypoints: Vector of waypoints, where the solution is stored
 *
 * TODO: Make orientation changeable
 */
// void MoveItController::computeWaypointCircular(Eigen::Vector3f& start, double yOffset, double zOffset,
// 		double angleTransPoints, int num_waypoints, std::vector<Eigen::Affine3d>& waypoints) {

// 	Eigen::Matrix4d waypoint;

// 	start(1) -= yOffset;
// 	start(2) += zOffset;

// 	const double turn_radius = sqrt(yOffset*yOffset+zOffset*zOffset);
// 	const double alpha = 1.5*M_PI + angleTransPoints;

// 	for(int i=0; i <= num_waypoints; i++) {
// 		double rad_phi = 10 * i * (M_PI / 180);
// 		double beta =  alpha + rad_phi - 2*M_PI;

// 		if(hand == WSG_GRIPPER) {
// 			rad_phi -= (M_PI / 2);
// 		}

// 		double new_y = cos(beta) * turn_radius + start(1);
// 		double new_z = sin(beta) * turn_radius + start(2);

// 		waypoint << 0, 0, 1, start(0),
// 					-cos(rad_phi), sin(rad_phi), 0, new_y,
// 					-sin(rad_phi), -cos(rad_phi), 0, new_z,
// 					0, 0, 0, 1;

// 		Eigen::Affine3d waypointAffine;
// 		waypointAffine.matrix() = waypoint;
// 		waypoints.push_back(waypointAffine);
// 	}
// }

void MoveItController::computeWaypointCircular(Eigen::Vector3f centerOfCup, const double &cupRadius,
	                                                const double &cupHalfHeight, const double &angle,
	                                                const int numOfWaypoints,
	                                                std::vector<Eigen::Affine3d> &waypoints)
{
	double turnRadius = sqrt(cupRadius*cupRadius + cupHalfHeight*cupHalfHeight);

	Eigen::Matrix4d waypoint;
	for(int i = 0; i <= numOfWaypoints; ++i)
	{
		// angular step for rotation
		double gamma = 1.45 * i * (M_PI / 180);

		// angle for calculationg new position of cup center
		double beta = angle + gamma;

		//calculating new center position of cup
		double y = turnRadius * sin(beta);
		double z = turnRadius * cos(beta);

		double deltaY = y - cupRadius;
		double deltaZ = cupHalfHeight - z;

		double newY = centerOfCup(1) + deltaY;
		double newZ = centerOfCup(2) + deltaZ;

		// rotation
		Eigen::Matrix3d r;
		r = Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitX())
		        * Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitY())
		        * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());

		// translation
		Eigen::Vector4d t(centerOfCup(0), newY, newZ, 1);

		// transformation matrix
		Eigen::Matrix4d trans;

		trans.setIdentity();
		trans.block<3,3>(0,0) = r;
		trans.rightCols<1>() = t;

		std:: cout << "Waypoint " << i << ":\n" << trans << std::endl << std::flush;
		Eigen::Affine3d waypointAffine;
		waypointAffine.matrix() = trans;
		waypoints.push_back(waypointAffine);
	}
}

/**
 * This is an extra method for the needs of BMI group for puring
 * Do not use this
 * TODO: Make available
 */
void MoveItController::computeWaypointCircularReverse(Eigen::Vector3f& start, int num_waypoints,
		std::vector<Eigen::Affine3d>& waypoints) {

	Eigen::Matrix4d waypoint;

	for(int i = num_waypoints; i >= 0; i--) {
		double rad_phi = 10 * i * (M_PI / 180);

		if(hand == WSG_GRIPPER) {
			rad_phi -= (M_PI / 2);
		}

		if((10 * i) < 50) {
			start(0) += 0.02;
		}

		waypoint << 0, 0, 1, start(0),
					-cos(rad_phi), sin(rad_phi), 0, start(1),
					-sin(rad_phi), -cos(rad_phi), 0, start(2),
					0, 0, 0, 1;

		Eigen::Affine3d waypointAffine;
		waypointAffine.matrix() = waypoint;
		waypoints.push_back(waypointAffine);
	}
}


/**
 * Sends the arm in the predefined (in robots srdf-file) hom position
 * For the KUKA arms its all joints on zero (arm pointing upwards)
 */
void MoveItController::goHome() {

	group.setNamedTarget("home");
	bool moved = group.move();
	if(!moved)  {
		pub_finished(FINISHED_ERROR);
	}
	else {
		pub_finished(FINISHED_SUCCESS);
	}
}

/**
 * Publishes on the topic "arm_movement_finished", when the arm movement is finished
 * Or if there was an error
 *
 * finish: publishes "movement_finished", when everthing worked fine
 * 		   publishes "movement_error", when there was an error and the movement was not occupied
 */
void MoveItController::pub_finished(string finish) {
	std_msgs::String finished;
	finished.data = finish;
	pub_finish.publish(finished);
}

/**
 * When the Schunk Hand is online and available this method induces the gripper to get open
 *
 * speed: The speed of the Hand (ranging from 0 to 1.0, 0.3 as reasonable default value)
 */
void MoveItController::openGripper(double speed) {
	ROS_INFO("I will now open my gripper.");

	if(hand == SCHUNK_HAND) {
		sdh2_hand::SDHAction msg_gripper;
		msg_gripper.request.type = PARALLEL;
		msg_gripper.request.gripType = NOSTOP;
		msg_gripper.request.ratio = 0;
		msg_gripper.request.velocity = speed;
		hand_client.call(msg_gripper);
	}
	else if(hand == WSG_GRIPPER) {
		sleep(1);
		wsg_gripper::GripperCommandGoal goal;
		goal.width = 109;
		goal.speed = 250;
		goal.command_code = 33;
		wsg_pub.publish(goal);
	}

}

/**
 * When the Schunk Hand is online and available this method induces the gripper to close
 *
 * speed: The speed of the Hand (ranging from 0 to 1.0, 0.2 as reasonable default value)
 * closing_mm: How many millimeters do you want to close the wsg_gripper.
 * 			   Only works for wsg_gripper, not for Schunk Hand
 */
void MoveItController::closeGripper(double speed, double closing_mm) {
	ROS_INFO("I will now close my gripper.");

	if(hand == SCHUNK_HAND) {
		sdh2_hand::SDHAction msg_gripper;
		msg_gripper.request.type = PARALLEL;
		msg_gripper.request.gripType = STOPFINGER;
		msg_gripper.request.ratio = 1;
		msg_gripper.request.velocity = speed;
		hand_client.call(msg_gripper);
	}
	else if(hand == WSG_GRIPPER) {
		sleep(1);
		wsg_gripper::GripperCommandGoal goal;
		goal.width = closing_mm;
		goal.speed = 250;
		goal.command_code = 33;
		wsg_pub.publish(goal);
	}

}


/**
 * Moves the Arm straight on a line between a start and an end position of the endeffector
 * If there is no plan or anything else did not work it return the arm to a revory position
 *
 * start: Start position for the movement (in xyz-coordinates in base_frame)
 * target: End position for the movement (in xyz-coordinates in base_frame)
 * orientation: The orientation of the End-Effector at all the waypoints
 * num_waypoints: The number of waypoints that should be created on the given line
 * recover: The position, where the robot drives to, when goStraight was not possible
 *
 * return: True, if movement was completed successfully
 * 		   False, if an error occurred
 */
bool MoveItController::goStraight(Eigen::Vector3f& start, Eigen::Vector3f& target,
		Eigen::Matrix3f& orientation, int num_waypoints, Eigen::Vector3f& recover) {

	std::vector<Eigen::Affine3d> waypoints;
	computeWaypointsStraight(start, target, orientation, num_waypoints, waypoints);
	// std::vector<Eigen::Affine3d>::iterator it;
	// for (it = waypoints.begin(); it != waypoints.end(); ++it)
	// {
	//     std::cout << it->matrix() << std::endl;
	// }

	planning_interface::MoveGroup::Plan plan;
	if(calculatePlan(waypoints, plan) > 0) {
		goToPosition(recover, orientation);
		return false;
	}
	if(executePlan(plan) == 1) {
		goToPosition(recover, orientation);
		return false;
	}
	return true;

}

/**
 * Turns the End-Effektor with a fixed starting orientation
 * Is able to transform the turning_center to a new position (great for auto_drinking)
 *
 * position: The start point of the End-Effector
 * xOffset: The offset in x-direction to turning center (give 0, when willing to turn End-Effector)
 * zOffset: The offset in z-direction to turning center (give 0, when willing to turn End-Effector)
 * angleTransPoints: Angle between $position and turning center (give 0, when willing to turn End-Effector)
 * num_waypoints: The number of waypoints, that should be calculated
 * 				  This also determines on how much the End-Effector gets turned
 * 				  In degrees: (num_waypoints-1)*10
 * recover: The position, where the robot drives to, when goStraight was not possible
 * orientation: The orientation of the recovery-position
 *
 * return: True, if movement was completed successfully
 * 		   False, if an error occurred
 */
bool MoveItController::goCircular(Eigen::Vector3f& position, double xOffset, double zOffset,
		double angleBetweenPoints, int num_waypoints, Eigen::Vector3f& recover,
		Eigen::Matrix3f& orientation, bool reverse) {

	std::vector<Eigen::Affine3d> waypoints;
	if(reverse) {
		computeWaypointCircularReverse(position, num_waypoints, waypoints);
	} else {
		computeWaypointCircular(position, xOffset, zOffset, angleBetweenPoints, num_waypoints, waypoints);
	}
	planning_interface::MoveGroup::Plan plan;
	if(calculatePlan(waypoints, plan) > 0) {
		goToPosition(recover, orientation);
		return false;
	}
	if(executePlan(plan) == 1) {
		goToPosition(recover, orientation);
		return false;
	}
	return true;

}

/**
 * Get the current position of the End-Effector
 * Not really working. Takes wrong base_frame
 *
 * return: The current position as a vector with xyz-coordinates
 */
Eigen::Vector3f MoveItController::getCurrentPosition() {

	geometry_msgs::PoseStamped currentPose = group.getCurrentPose("iiwa_7_link");
	Eigen::Vector3f current_position(currentPose.pose.position.x,
			currentPose.pose.position.y,currentPose.pose.position.z);
	return current_position;

}

/**
 * Check Reachability for a given position for the current robot
 *
 * position: the position to be checked, given in coordinates in robot frame
 *
 * return: true, if position is reachable
 */
bool MoveItController::checkReachability(Eigen::Affine3d& position) {

	core::RobotStatePtr kinematic_state(new
			robot_state::RobotState(group.getCurrentState()->getRobotModel()));
	const core::JointModelGroup* joint_model_group =
			group.getCurrentState()->getRobotModel()->getJointModelGroup(robot_name);

	Eigen::Affine3d transform = kinematic_state->getFrameTransform(name_prefix + "_0_link");
	return kinematic_state->setFromIK(joint_model_group, position, name_prefix + "_7_link", 10, 0.1);

}

/**
 * computes the needed jointPositions for a given cartesian position
 *
 * robotPose: the cartesian position in robot frame
 * jointPose: the output of the function, is empty if position is not reachable
 *
 * return: true, if position is reachable and calculation is done properly
 */
bool MoveItController::cartesianToJointPosition(Eigen::Affine3d& robotPose,
		sensor_msgs::JointState& jointPose) {

	core::RobotStatePtr kinematic_state(new
			robot_state::RobotState(group.getCurrentState()->getRobotModel()));
	const core::JointModelGroup* joint_model_group =
			group.getCurrentState()->getRobotModel()->getJointModelGroup(robot_name);
	Eigen::Affine3d transform = kinematic_state->getFrameTransform(name_prefix + "_0_link");
	if(kinematic_state->setFromIK(joint_model_group, robotPose, name_prefix + "_7_link", 10, 0.1)) {

		std::vector<double> joint_values;
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		const std::vector<std::string> &joint_names = joint_model_group->getActiveJointModelNames();
		sensor_msgs::JointState state;

		for(size_t i = 0; i < joint_names.size(); ++i) {
			state.name.push_back(joint_names[i]);
			state.position.push_back(joint_values[i]);
		}
		jointPose = state;
		return true;
	}
	else {
		return false;
	}
}

/**
 * Computes for a vector of given cartisian positions the corresponding joint positions for the robot
 * the output might be smaller than the input, because some of the positions might not be reachable
 *
 * robotPoses: The given cartesian positions in the robot frame
 * jointPoses: The output jointPositions corresponding to the cartesian positions
 * 				Note: This vector might be smaller than the other one, because some positions
 * 				      might not be reachable
 */
void MoveItController::cartesianToJointPositions(std::vector<Eigen::Affine3d>& robotPoses,
		std::vector<sensor_msgs::JointState>& jointPoses) {

	core::RobotStatePtr kinematic_state(new
			robot_state::RobotState(group.getCurrentState()->getRobotModel()));
	const core::JointModelGroup* joint_model_group =
			group.getCurrentState()->getRobotModel()->getJointModelGroup(robot_name);
	Eigen::Affine3d transform = kinematic_state->getFrameTransform(name_prefix + "_0_link");

	for (size_t i = 0; i < robotPoses.size(); ++i) {

		if(kinematic_state->setFromIK(joint_model_group, robotPoses[i], name_prefix + "_7_link", 10, 0.1)) {

			sleep(0.04);
			std::vector<double> joint_values;
			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			const std::vector<std::string> &joint_names = joint_model_group->getActiveJointModelNames();
			sensor_msgs::JointState state;

			for(size_t i = 0; i < joint_names.size(); ++i)
			{
				state.name.push_back(joint_names[i]);
				state.position.push_back(joint_values[i]);
			}
			jointPoses.push_back(state);
		}
		else {
			ROS_INFO("For point %f %f %f no inverse kinematic solution was found.",
					robotPoses[i].translation()[0], robotPoses[i].translation()[1],
					robotPoses[i].translation()[2]);
		}
	}
}
