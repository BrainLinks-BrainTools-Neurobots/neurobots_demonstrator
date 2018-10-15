/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 31, 2017
 *      Author: kuhnerd
 * 	  Filename: calibration.cpp
 */

#include <kuka_hand2eye_calibration/calibration.h>

#include <aruco/aruco_helpers.h>
#include <boost/filesystem/operations.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <kuka_manager/JointState.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <visp_hand2eye_calibration/compute_effector_camera_quick.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>

namespace enc = sensor_msgs::image_encodings;

namespace kuka_hand2eye_calibration
{

Calibration::Calibration(const std::string& robot,
		const std::string& boardConfigFile,
		const double markerSize,
		const std::string& flangeFrame,
		const std::string& worldFrame,
		const std::string& jointStateService,
		const std::string& trajectoryFile,
		const std::string& calibrationFile,
		const std::string& cameraPrefix) :
				m_cameraInfoReceivedRGB(false),
				m_markerSize(markerSize),
				m_storeData(false),
				m_cameraPrefix(cameraPrefix),
				m_flangeFrame(flangeFrame),
				m_worldFrame(worldFrame),
				m_robot(robot),
				m_jointStateService(jointStateService),
				m_trajectoryFile(trajectoryFile),
				m_calibrationFile(calibrationFile)
{
	ros::NodeHandle nh;

	readTrajectoryFromFile(m_trajectoryFile, m_joints);
	ROS_INFO_STREAM("Using " << m_joints.size() << " joint positions");

	m_imagePub = nh.advertise<sensor_msgs::Image>("marker_image", 1);
	m_transformListener = new tf::TransformListener;

	m_boardConfig.readFromFile(boardConfigFile);

	m_subscriberCameraInfoRGB = nh.subscribe("/" + cameraPrefix + "camera/rgb/camera_info", 1, &Calibration::callbackCameraInfoRGB, this);

	ROS_INFO("Listening to %s and %s", ("/" + cameraPrefix + "camera/depth_registered/image_raw").c_str(),
			("/" + cameraPrefix + "camera/rgb/image_raw").c_str());

	static message_filters::Subscriber<sensor_msgs::Image> depthSub(nh, "/" + cameraPrefix + "camera/depth_registered/image_raw", 1);
	static message_filters::Subscriber<sensor_msgs::Image> colorSub(nh, "/" + cameraPrefix + "camera/rgb/image_raw", 1);
	m_sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), depthSub, colorSub);
	m_sync->registerCallback(boost::bind(&Calibration::callbackImages, this, _1, _2));

	m_calibrationClient = nh.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick>("compute_effector_camera_quick");
}

Calibration::~Calibration()
{
	if (m_transformListener != NULL)
		delete m_transformListener;

	delete m_sync;
}

void Calibration::run()
{
	ROS_INFO("run");
	sleep(2);

	ros::NodeHandle n;
	ros::ServiceClient omnirobCmdClient = n.serviceClient<kuka_manager::JointState>(m_jointStateService);

//	std::vector<sensor_msgs::JointState> joints =
//			{
//					j(-2.2, -1.0, 0.7, 1.1, 2.3, 0.0, 2.6),
//					j(-2.3, -1.0, 0.7, 1.1, 2.3, 0.0, 2.6),
//					j(-2.1, -1.0, 0.7, 1.1, 2.3, 0.0, 2.6),
//					j(-2.2, -1.0, 0.6, 1.1, 2.3, 0.0, 1.6),
//					j(-2.2, -1.0, 0.6, 1.2, 2.4, 0.0, 2.6),
//					j(-2.2, -1.0, 0.8, 1.1, 2.3, 0.0, 2.0),
//					j(-2.2, -1.0, 0.6, 1.2, 2.3, 0.0, 2.6),
//					j(-2.2, -1.0, 0.7, 1.1, 2.4, 0.0, 2.6),
//					j(-2.3, -1.0, 0.7, 1.1, 2.2, 0.0, 2.1),
//					j(-2.1, -1.0, 0.7, 1.1, 2.1, 0.0, 2.6),
//					j(-2.2, -1.0, 0.7, 1.1, 2.3, 0.1, 2.6),
//					j(-2.2, -1.0, 0.7, 1.1, 2.4, 0.2, 2.2),
//					j(-2.2, -1.0, 0.8, 1.1, 2.3, -0.1, 2.6),
//					j(-2.2, -1.0, 0.7, 1.0, 2.2, 0.0, 2.3)
//			};

//	for (double j1 = -0.85; j1 <= -0.7; j1 += 0.05)
//		for (double j2 = -0.28; j2 <= -0.18; j2 += 0.05)
//			for (double j3 = -0.35; j3 <= -0.25; j3 += 0.05)
//				for (double j4 = 0.75; j4 <= 0.9; j4 += 0.05)
//					for (double j5 = 1.45; j5 <= 1.60; j5 += 0.05)
//						joints.push_back(
//								j(j1, j2, j3, j4, j5, 1.7012809494870302, 1.0862639796508573));

	ros::Rate r(30);
	int i = 0;
	for (auto& it : m_joints)
	{
		kuka_manager::JointState srv;
		srv.request.jointState = it;
		omnirobCmdClient.call(srv);
		sleep(2);

		m_storeData = true;

		ROS_INFO("%d: Waiting for data!", i++);
		while (m_storeData && ros::ok())
		{
			r.sleep();
		}

		if (!ros::ok())
			break;
	}

	//fill header of TransformArray?
	visp_hand2eye_calibration::compute_effector_camera_quick srvCalibration;

	for (size_t i = 0; i < m_markerPoses.size(); ++i)
	{
		geometry_msgs::Transform camToObjectMsg, baseToFlangeMsg;
		tf::transformEigenToMsg(m_markerPoses[i], camToObjectMsg);
		tf::transformEigenToMsg(m_robotPoses[i], baseToFlangeMsg);
		srvCalibration.request.camera_object.transforms.push_back(camToObjectMsg);
		srvCalibration.request.world_effector.transforms.push_back(baseToFlangeMsg);
	}

	m_calibrationClient.call(srvCalibration);
	tf::Transform handToEye;
	tf::transformMsgToTF(srvCalibration.response.effector_camera, handToEye);

	ROS_INFO_STREAM("Translation: " << handToEye.getOrigin().x() << ", " << handToEye.getOrigin().y() << ", " << handToEye.getOrigin().z());
	ROS_INFO_STREAM(
			"Rotation: " << handToEye.getRotation().x() << ", " << handToEye.getRotation().y() << ", " << handToEye.getRotation().z() << ", " << handToEye.getRotation().w());

	static tf::TransformBroadcaster tfB;

	tf::StampedTransform tfCamera;
	try
	{
		m_transformListener->lookupTransform(m_cameraPrefix + "camera_link", m_cameraPrefix + "camera_rgb_optical_frame", ros::Time(), tfCamera);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("TF exception: %s", ex.what());
	}

	tf::Transform final = handToEye * tfCamera.inverse();

	saveCalibration(final);

	kuka_manager::JointState srv;
	srv.request.jointState = j(-1.301288402042873, -0.2056609404741501, -1.7049988498280326, 1.6427482617393463, 1.7780608100631494, -0.9729205711136143, 1.22810592682393);;
	omnirobCmdClient.call(srv);

	while (ros::ok())
	{
		tfB.sendTransform(
				tf::StampedTransform(final, ros::Time::now(), m_flangeFrame,
						m_cameraPrefix + "camera_link"));
		r.sleep();
	}
}

void Calibration::callbackImages(const sensor_msgs::Image::ConstPtr& imageDepth,
		const sensor_msgs::Image::ConstPtr& imageRgb)
{
	if (!m_storeData)
		return;

	if (!m_cameraInfoReceivedRGB)
		return;

	static tf::TransformBroadcaster tfB;

	//convert the color image
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(imageRgb, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat colorImage = cv_ptr->image;

	//convert the depth image
	try
	{
		cv_ptr = cv_bridge::toCvCopy(imageDepth, enc::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat depthImage = cv_ptr->image;

	//detect the markers
	std::vector<aruco::Marker> markers;
	m_markerDetector.detect(colorImage, markers, m_cameraParameters, m_markerSize);

	std::vector<Eigen::Vector3d> markerCenterPoints;

	for (auto& marker : markers)
	{
		//draw image
		marker.draw(colorImage, cv::Scalar(0, 0, 255), 2);
		aruco::CvDrawingUtils::draw3dAxis(colorImage, marker, m_cameraParameters);
		aruco::CvDrawingUtils::draw3dCube(colorImage, marker, m_cameraParameters);

//		//get depth value from depth image
//		float depth = depthImage.at<float>(marker.getCenter().y, marker.getCenter().x);
//		if (std::isnan(depth))
//		{
//			ROS_WARN("Got nan value for marker depth");
//			return;
//		}
//
//		// Qick hack. If camera publishes depth in mm, divide by 1000
//		if (depth > 100)
//		{
//			depth /= 1000.0;
//		}
//
		Eigen::Vector3d tempPos;
		tempPos << marker.Tvec.at<float>(0, 0), marker.Tvec.at<float>(1, 0), marker.Tvec.at<float>(2, 0);
		markerCenterPoints.push_back(tempPos);
	}

	//compute position based on the positions in the depth image
	Eigen::Vector3d centerPos(0, 0, 0);
	for (auto& center : markerCenterPoints)
	{
		centerPos += center;
	}
	centerPos /= markerCenterPoints.size();

	if (markers.size() != m_boardConfig.size())
	{
		m_image = colorImage;
		return;
	}

	aruco::Board board;
	float probDetect = m_boardDetector.detect(markers, m_boardConfig, board, m_cameraParameters, m_markerSize);

	aruco::CvDrawingUtils::draw3dAxis(colorImage, board, m_cameraParameters);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImage).toImageMsg();
	m_imagePub.publish(msg);

	Eigen::Affine3d tMarker = aruco_helpers::toEigen(board);

	tf::Transform t;
	tf::transformEigenToTF(tMarker, t);
	tfB.sendTransform(tf::StampedTransform(t, ros::Time::now(), m_cameraPrefix + "camera_rgb_optical_frame", "marker"));

	//get transformation from robot
	tf::StampedTransform tf;
	try
	{
		m_transformListener->lookupTransform(m_worldFrame, m_flangeFrame, imageDepth->header.stamp, tf);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("TF exception: %s", ex.what());
		return;
	}
	Eigen::Affine3d tRobot;
	tf::transformTFToEigen(tf, tRobot);

	m_robotPoses.push_back(tRobot);
	tMarker.translation() = centerPos;
	m_markerPoses.push_back(tMarker);

	visp_hand2eye_calibration::compute_effector_camera_quick srvCalibration;

	for (size_t i = 0; i < m_markerPoses.size(); ++i)
	{
		geometry_msgs::Transform camToObjectMsg, baseToFlangeMsg;
		tf::transformEigenToMsg(m_markerPoses[i], camToObjectMsg);
		tf::transformEigenToMsg(m_robotPoses[i], baseToFlangeMsg);
		srvCalibration.request.camera_object.transforms.push_back(camToObjectMsg);
		srvCalibration.request.world_effector.transforms.push_back(baseToFlangeMsg);
	}

	m_calibrationClient.call(srvCalibration);
	tf::Transform eyeToHand;
	tf::transformMsgToTF(srvCalibration.response.effector_camera, eyeToHand);

	ROS_INFO_STREAM("Translation: " << eyeToHand.getOrigin().x() << ", " << eyeToHand.getOrigin().y() << ", " << eyeToHand.getOrigin().z());
	ROS_INFO_STREAM(
			"Rotation: " << eyeToHand.getRotation().x() << ", " << eyeToHand.getRotation().y() << ", " << eyeToHand.getRotation().z() << ", " << eyeToHand.getRotation().w());

	m_storeData = false;
}

void Calibration::callbackCameraInfoRGB(const sensor_msgs::CameraInfo::ConstPtr& rgbCameraInfo)
{
	if (m_cameraInfoReceivedRGB)
	{
		return;
	}

	m_cameraInfoReceivedRGB = true;
	m_cameraParameters.CamSize.height = rgbCameraInfo->height;
	m_cameraParameters.CamSize.width = rgbCameraInfo->width;

	m_cameraParameters.CameraMatrix = cv::Mat(3, 3, CV_32F);
	m_cameraParameters.CameraMatrix.at<float>(0, 0) = rgbCameraInfo->K[0]; //fx
	m_cameraParameters.CameraMatrix.at<float>(1, 1) = rgbCameraInfo->K[4]; //fy
	m_cameraParameters.CameraMatrix.at<float>(0, 2) = rgbCameraInfo->K[2]; //cx
	m_cameraParameters.CameraMatrix.at<float>(1, 2) = rgbCameraInfo->K[5]; //cy

	m_subscriberCameraInfoRGB.shutdown();
}

sensor_msgs::JointState Calibration::j(double j1,
		double j2,
		double j3,
		double j4,
		double j5,
		double j6,
		double j7)
{
	sensor_msgs::JointState jointState;
	if (m_robot == "omnirob")
		jointState.name =
		{	"lbr_1_joint", "lbr_2_joint", "lbr_3_joint", "lbr_4_joint", "lbr_5_joint", "lbr_6_joint",
			"lbr_7_joint"};
		else
		jointState.name =
		{	"iiwa_1_joint", "iiwa_2_joint", "iiwa_3_joint", "iiwa_4_joint", "iiwa_5_joint", "iiwa_6_joint",
			"iiwa_7_joint"};
	jointState.position =
	{	j1, j2, j3, j4, j5, j6, j7};

	return jointState;
}

bool Calibration::readTrajectoryFromFile(const std::string& filename,
		std::vector<sensor_msgs::JointState>& joints)
{
	std::ifstream ifile(filename, std::ios::in);

	if (!ifile.is_open())
	{
		ROS_ERROR("There was a problem opening the input file!");
		return false;
	}

	std::vector<double> in(7);
	while (true)
	{
		int i = 0;
		for (; ifile >> in[i]; ++i)
		{
			if (i == 6)
			{
				joints.push_back(j(in[0], in[1], in[2], in[3], in[4], in[5], in[6]));
				break;
			}
		}
		if (i != 6)
			break;
	}

	return true;
}

void Calibration::saveCalibration(const tf::Transform& t)
{
	std::string path = m_calibrationFile;
	std::string pathOld = m_calibrationFile + "_backup";

	if (boost::filesystem::exists(path))
	{
		boost::filesystem::rename(path, pathOld);
	}

	std::ofstream myfile;
	myfile.open(path.c_str());
	myfile.precision(15);

	tf::Matrix3x3 rot = t.getBasis();
	tf::Vector3 trans = t.getOrigin();

	myfile << rot[0].x() << " " << rot[0].y() << " " << rot[0].z() << " " << trans.x() << "\n"
			<< rot[1].x() << " " << rot[1].y() << " " << rot[1].z() << " " << trans.y() << "\n"
			<< rot[2].x() << " " << rot[2].y() << " " << rot[2].z() << " " << trans.z() << "\n"
			<< 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << "\n";

	myfile.close();

	ROS_INFO("Saved calibration in '%s'", path.c_str());
}

} /* namespace kuka_hand2eye_calibration */
