/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 31, 2017
 *      Author: kuhnerd
 * 	  Filename: calibration.h
 */

#ifndef HAC90A675_D401_425D_A886_48E25687DDCE
#define HAC90A675_D401_425D_A886_48E25687DDCE
#include <aruco.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

namespace kuka_hand2eye_calibration
{

class Calibration
{
public:
	Calibration(const std::string& robot,
			const std::string& boardConfigFile,
			const double markerSize,
			const std::string& flangeFrame,
			const std::string& worldFrame,
			const std::string& jointStateService,
			const std::string& trajectoryFile,
			const std::string& calibrationFile,
			const std::string& cameraPrefix);
	virtual ~Calibration();

	void run();

	void callbackImages(const sensor_msgs::Image::ConstPtr& imageDepth,
			const sensor_msgs::Image::ConstPtr& imageRgb);
	void callbackCameraInfoRGB(const sensor_msgs::CameraInfo::ConstPtr& rgbCameraInfo);

private:
	sensor_msgs::JointState j(double j1,
			double j2,
			double j3,
			double j4,
			double j5,
			double j6,
			double j7);

	bool readTrajectoryFromFile(const std::string& filename,
			std::vector<sensor_msgs::JointState>& joints);

	void saveCalibration(const tf::Transform& t);

private:
	bool m_cameraInfoReceivedRGB;
	boost::atomic_bool m_storeData;

	tf::TransformListener* m_transformListener;
	ros::Publisher m_imagePub;
	ros::Subscriber m_subscriberCameraInfoRGB;
	ros::AsyncSpinner* m_spinner;
	message_filters::Synchronizer<SyncPolicy>* m_sync;
	ros::ServiceClient m_calibrationClient;

	aruco::MarkerDetector m_markerDetector;
	aruco::BoardConfiguration m_boardConfig;
	aruco::BoardDetector m_boardDetector;
	aruco::CameraParameters m_cameraParameters;

	std::vector<Eigen::Affine3d> m_markerPoses;
	std::vector<Eigen::Affine3d> m_robotPoses;

	double m_markerSize;
	const std::string m_cameraPrefix;
	const std::string m_flangeFrame;
	const std::string m_worldFrame;
	const std::string m_robot;
	const std::string m_jointStateService;
	const std::string m_trajectoryFile;
	const std::string m_calibrationFile;
	std::vector<sensor_msgs::JointState> m_joints;

	cv::Mat m_image;
};

} /* namespace kuka_hand2eye_calibration */

#endif /* HAC90A675_D401_425D_A886_48E25687DDCE */
