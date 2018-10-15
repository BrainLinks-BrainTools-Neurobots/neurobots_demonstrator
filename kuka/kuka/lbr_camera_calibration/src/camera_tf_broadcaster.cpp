/*
 * camera_tf_broadcaster.cpp
 *
 * Copyright (C) 2015 Daniel Kuhner (kuhnerd@informatik.uni-freiburg.de)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <fstream>

bool loadCalibrationMatrix(const std::string& path,
		tf::Transform& transform)
{
	if (!boost::filesystem::exists(path))
	{
		return false;
	}

	std::ifstream myfile;
	myfile.open(path.c_str());

	Eigen::Affine3d matrix;
	myfile >> matrix(0, 0) >> matrix(0, 1) >> matrix(0, 2) >> matrix(0, 3)
			>> matrix(1, 0) >> matrix(1, 1) >> matrix(1, 2) >> matrix(1, 3)
			>> matrix(2, 0) >> matrix(2, 1) >> matrix(2, 2) >> matrix(2, 3)
			>> matrix(3, 0) >> matrix(3, 1) >> matrix(3, 2) >> matrix(3, 3);

	tf::transformEigenToTF(matrix, transform);
//	transform = transform.inverse();

	return true;
}

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "camera_tf_broadcaster");
	ros::NodeHandle nh("~");

	std::string fromLink, toLink, calibrationFile;
	if (!nh.getParam("from_link", fromLink))
	{
		fromLink = "lbr_0_link";
	}
	if (!nh.getParam("to_link", toLink))
	{
		toLink = "camera_rgb_optical_frame";
	}
	if (!nh.getParam("calibration_file", calibrationFile))
	{
		calibrationFile = ros::package::getPath("lbr_camera_calibration") + "/calibration.txt";
	}

	tf::Transform tf;
	if (!loadCalibrationMatrix(calibrationFile, tf))
	{
		ROS_ERROR("Cannot read calibration matrix!");
		return -1;
	}

	ROS_INFO("============================================================");
	ROS_INFO("Configuration: ");
	ROS_INFO("  Calibration matrix:     %s", calibrationFile.c_str());
	ROS_INFO("  'from'-link:            %s", fromLink.c_str());
	ROS_INFO("  'to'-link:              %s", toLink.c_str());
	ROS_INFO("============================================================");

	tf::TransformBroadcaster br;

	ros::Rate r(100);
	while (ros::ok())
	{
		br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), fromLink, toLink));
		r.sleep();
	}
}

