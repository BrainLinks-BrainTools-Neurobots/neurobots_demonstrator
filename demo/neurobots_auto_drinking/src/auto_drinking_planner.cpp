/*
 * auto_drinking.cpp
 *
 *  Created on: September 14, 2016
 *      Author: Jeremias Holub
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "kuka_manager/kuka_robots.h"
#include "MoveItController.h"
#include <robot_interface_definition/robot_interface.h>

#include <string>
#include <iostream>

class AutoDrinkingPlanner : public robot_interface_definition::RobotInterface
{
	tf::TransformListener listener;

	MoveItController *armController;
	ros::NodeHandle nh;


public:
	AutoDrinkingPlanner()
	{
		std::cout << "Initialize arm controller..." << std::endl;
		armController = new MoveItController("iiwa", SCHUNK_HAND);
		armController->init(nh);
		std::cout << "MoveIt! controller initialized!" << std::endl;
		sleep(2);
		int a;
		std::cin >> a;
	}

	virtual ~AutoDrinkingPlanner()
	{
		delete armController;
	}

	/**
	 * Implement this method in your planner.
	 * The frame of 'goal' is 'world'
	 */
	virtual bool plan(const Eigen::Affine3d& goal)
	{
		return true;
	}

	virtual bool execute()
	{
		////////////////////////////////////////////////////////////////////////////////////////////////
		//// Initializiaton of position and orientation position of start target, gripper and mouth ////
		////////////////////////////////////////////////////////////////////////////////////////////////

		// initial position for testing
		Eigen::Vector3f startingPosition(0.6,0.7,0.3);

		// initial orientation for testing
		Eigen::Matrix3f startingOrientation;
		startingOrientation.row(0) = Eigen::Vector3f(0,0,1);
		startingOrientation.row(1) = Eigen::Vector3f(1,0,0);
		startingOrientation.row(2) = Eigen::Vector3f(0,1,0);
	    
		// get current gripper Position
		tf::StampedTransform gripperTF;
		Eigen::Vector3f gripperPosition = getCurrentGripperPosition(gripperTF);

		// get current gripper orientation
		Eigen::Quaterniond quat;
		Eigen::Matrix3f gripperOrientation = getCurrentGripperOrientation(gripperTF, quat);
		
		// get current mouth position
		tf::StampedTransform mouthTF;
		getTransform("/iiwa/iiwa_base_link", "/mouth", mouthTF);
		Eigen::Vector3f mouthPosistion = tfToEigenVector3f(mouthTF);
		
		// output current gripper position
		std::cout << "Current Position of Gripper: \n (" << gripperPosition.x() << "," 
		          << gripperPosition.y() << "," << gripperPosition.z() << ")" << std::endl;
		
		// output current mouth position
		std::cout << "Current Position of Mouth: \n (" << mouthPosistion.x() << "," << mouthPosistion.y() 
		          << "," << mouthPosistion.z() << ")" << std::endl;

        ////////////////////////////////////////////////////////////////////////////////////////////////
		//////////// Move gripper to the starting point from where it will move to the mouth ///////////
		////////////////////////////////////////////////////////////////////////////////////////////////


		// armController->goToPosition(startingPosition, startingOrientation);
		
		// sleep(3);

		////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////// Move gripper to mouth //////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////

		// update gripper position and orientation
		gripperPosition = getCurrentGripperPosition(gripperTF);
		gripperOrientation = getCurrentGripperOrientation(gripperTF, quat);

		// // offsets of cup
		float cupXOffset = -0.03;
		float cupRadius = 0.045;
		float cupHalfHeight = 0.0475;

		mouthPosistion(0) += cupXOffset;
		mouthPosistion(1) += cupRadius; // + 0.015;
		mouthPosistion(2) -= cupHalfHeight;

		armController->goStraight(gripperPosition, mouthPosistion, gripperOrientation, 30, gripperPosition);
		// sleep(3);

		////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////// Rotate cup /////////////////////////////////////////
		// ////////////////////////////////////////////////////////////////////////////////////////////////
		
		// // update gripper position and orientation
		// gripperPosition = getCurrentGripperPosition(gripperTF);
		// gripperOrientation = getCurrentGripperOrientation(gripperTF, quat);

		// // angle between center of cup and mouth in rad
		// double angleMouthCup = 0.733038;

		// armController->goCircular(gripperPosition, cupRadius, cupHalfHeight, angleMouthCup, 60, 
		// 	                      gripperPosition, gripperOrientation, false);
		// armController->saveLastPlan("turn");

		// sleep(1);

		// ////////////////////////////////////////////////////////////////////////////////////////////////
		// ///////////////////////////////////////// Rotate cup back //////////////////////////////////////
		// ////////////////////////////////////////////////////////////////////////////////////////////////

		// // // update gripper position and orientation
		// gripperPosition = getCurrentGripperPosition(gripperTF);
		// gripperOrientation = getCurrentGripperOrientation(gripperTF, quat);

		// armController->reversePlan("turn","reverse_turn");
		// armController->executePlan("reverse_turn", gripperPosition, gripperOrientation);

		// sleep(1);

		// ////////////////////////////////////////////////////////////////////////////////////////////////
		// ////////////////////////////////// Move back to initial position ///////////////////////////////
		// ////////////////////////////////////////////////////////////////////////////////////////////////

		// armController->goStraight(mouthPosistion, startingPosition, startingOrientation, 30, mouthPosistion);

    	////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////// DEBUGGING CIRCULAR WAYPOINTS WITH TF BROADCASTER /////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////

		// // offsets of cup
		// float cupXOffset = 0.0;
		// float cupRadius = 0.045;
		// float cupHalfHeight = 0.0775;

		// // angle between center of cup and mouth in rad
		// double angleMouthCup = 0.523599;

		// // waypoints
		// std::vector<Eigen::Affine3d> waypoints;
		// int numOfWaypoints = 10;

		// armController->computeWaypointCircular(gripperPosition, cupRadius, cupHalfHeight, 
		// 	                                   angleMouthCup, numOfWaypoints, waypoints);

		// std::vector<tf::Transform> transforms;
		// std::vector<Eigen::Affine3d>::iterator it;
		// for(it = waypoints.begin(); it != waypoints.end(); ++it)
		// {
		// 	std::cout << "Waypoint: " << it->matrix() << std::endl << std::flush;
		// 	tf::Transform transform;
		// 	tf::transformEigenToTF(*it, transform);
		// 	transforms.push_back(transform);
		// }

		// tf::TransformBroadcaster br;
	
		// while(true)
		// {
		// 	tf::Transform transform;
		// 	std::vector<tf::Transform>::iterator it;
		// 	int count = 0;
		// 	for(it = transforms.begin(); it != transforms.end(); ++it)
		// 	{
		// 		++count;
		// 		br.sendTransform(tf::StampedTransform(*it, ros::Time::now(), "iiwa/iiwa_base_link", "rot_" + std::to_string(count)));
		// 	}
		// 	sleep(1);
		// }

		return true;
	}

	Eigen::Vector3f getCurrentGripperPosition(tf::StampedTransform &gripperTF)
	{
		getTransform("/iiwa/iiwa_base_link", "/iiwa/sdh2_grasp_link", gripperTF);
		return tfToEigenVector3f(gripperTF);
	}

	Eigen::Matrix3f getCurrentGripperOrientation(const tf::StampedTransform &gripperTF, Eigen::Quaterniond &quat)
	{
		tf::quaternionTFToEigen(gripperTF.getRotation(), quat);
		return quat.toRotationMatrix().cast<float>();
	}

	Eigen::Vector3f tfToEigenVector3f(const tf::StampedTransform &tf)
	{
		tf::Vector3 vec = tf.getOrigin();
		return Eigen::Vector3f(vec.getX(), vec.getY(), vec.getZ());
	}

	void getTransform(const std::string &source, const std::string &target, tf::StampedTransform &transform)
	{
		
	    ros::Rate rate(50.0);

	    bool transformArrived = false;
	    while (!transformArrived)
	    {
		    try
		    {
		    	listener.waitForTransform(source, target, ros::Time(0), ros::Duration(1.0));
		        listener.lookupTransform(source, target, ros::Time(0), transform);
		        transformArrived = true;
		    }
		    catch (tf::TransformException ex){
			    ROS_ERROR("%s",ex.what());
		    }
		    rate.sleep();
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "auto_drinking_planner");

	AutoDrinkingPlanner myPlanner;

	myPlanner.execute();

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::waitForShutdown();
	return 0;
}