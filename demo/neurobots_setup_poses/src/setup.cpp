/*
 * setup.cpp
 *
 *  Created on: Oct 2, 2016
 *      Author: kuhnerd
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <termios.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

tf::TransformListener* listener;
ros::NodeHandle* n;
Eigen::VectorXd jointState;
std::string linkWhichIsIncluded;
ros::Subscriber sub;
bool subActive;

int getch()
{
	static struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

void checkEnter()
{
	ros::Rate r(10);
	while (ros::ok() && getch() != 10)
		r.sleep();
}

Eigen::Affine3d getTf(const std::string frame1,
		const std::string frame2)
{
	tf::StampedTransform transform;
	try
	{
		listener->lookupTransform(frame1, frame2,
				ros::Time(0), transform);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	Eigen::Affine3d pose;
	tf::transformTFToEigen(transform, pose);
	return pose;
}

void cbJoints(sensor_msgs::JointStateConstPtr joints)
{
	if (std::find(joints->name.begin(), joints->name.end(), linkWhichIsIncluded) != joints->name.end())
	{
		jointState.resize(7);
		int j = 0;
		for (int i = joints->position.size() - 7; i < joints->position.size(); ++i)
		{
			jointState(j++) = joints->position[i];
		}

		sub.shutdown();
		subActive = false;
	}
}

Eigen::VectorXd getJointState(const std::string& topic,
		const std::string& linkWhichIsIncludedInMsg)
{
	linkWhichIsIncluded = linkWhichIsIncludedInMsg;
	subActive = true;
	sub = n->subscribe(topic, 1, cbJoints);
	ros::Rate r(10);
	while (subActive)
	{
		r.sleep();
	}
	return jointState;
}

void writeMatrix(const std::string& name,
		std::ofstream& file,
		const Eigen::Affine3d& matrix)
{
	file << name << ":\n  [";
	for (int i = 0; i < 16; ++i)
	{
		file << matrix.data()[i];
		if (i < 15)
			file << ", ";
	}
	file << "]\n";
}

void writeVector(const std::string& name,
		std::ofstream& file,
		const Eigen::VectorXd& vec)
{
	file << name << ":\n  [";
	for (int i = 0; i < vec.size(); ++i)
	{
		file << vec.data()[i];
		if (i < vec.size() - 1)
			file << ", ";
	}
	file << "]\n";
}

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "neurobots_setup_poses");
	n = new ros::NodeHandle;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	listener = new tf::TransformListener;
	subActive = false;

	Eigen::Affine3d startOmnirobPose, platformShelfLeft, platformShelfRight, eefTransport, bottleTable, platformTable;
	Eigen::VectorXd startOmnirobArmPose, initIiwa;

	ROS_INFO("Platform init position in map");
	checkEnter();
	startOmnirobPose = getTf("/map", "/omnirob_lbr/base_link");

	ROS_INFO("Platform init arm pose");
	checkEnter();
	startOmnirobArmPose = getJointState("/omnirob_lbr/joint_states", "lbr_1_joint");

	ROS_INFO("Platform position shelf left");
	checkEnter();
	platformShelfLeft = getTf("/map", "/omnirob_lbr/base_link");

	ROS_INFO("Platform position shelf right");
	checkEnter();
	platformShelfRight = getTf("/map", "/omnirob_lbr/base_link");

	ROS_INFO("Platform table shelf");
	checkEnter();
	platformTable = getTf("/map", "/omnirob_lbr/base_link");

	ROS_INFO("Platform arm eef pose robot during transport");
	checkEnter();
	eefTransport = getTf("/omnirob_lbr/lbr_0_link", "/omnirob_lbr/sdh2_grasp_link");

//	ROS_INFO("bottle on table");
//	checkEnter();
//	bottleTable = getTf("/map", "/table_camera/simtrack/water_bottle");

	std::string file = "/tmp/poses.yaml";

	if (boost::filesystem::exists(file))
		boost::filesystem::rename(file, file + "_backup");

	ROS_INFO("Write file");
	std::ofstream out(file);
	writeMatrix("init_base", out, startOmnirobPose);
	writeVector("init_arm", out, initIiwa);
	writeMatrix("base_shelf_left", out, platformShelfLeft);
	writeMatrix("base_shelf_right", out, platformShelfRight);
	writeMatrix("base_table", out, platformTable);
	writeMatrix("pose_eef_transport", out, eefTransport);
//	writeMatrix("pose_bottle_table", out, bottleTable);
	out.close();

	ros::shutdown();

	return 0;
}

