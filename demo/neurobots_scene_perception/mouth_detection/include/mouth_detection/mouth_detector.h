/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 11, 2017
 *      Author: kuhnerd
 * 	  Filename: mouth_detection.h
 */

#ifndef HBACFDA4E_17BC_4593_BE24_794EE5D4DECF
#define HBACFDA4E_17BC_4593_BE24_794EE5D4DECF

#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>

class MouthDetector
{
private:
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

public:
	MouthDetector();
	~MouthDetector();
	void parameterCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void callback(const sensor_msgs::ImageConstPtr& msg_rgb,
			const sensor_msgs::ImageConstPtr& msg_depth);
	void publish();

private:
	bool detectFace(bool& eyesDetected);
	bool addToList(const Eigen::Vector3d &pos,
			const Eigen::Vector3d& mean,
			std::list<Eigen::Vector3d>& lastCoordinates);
	Eigen::Vector3d calculateMeanPos(const std::list<Eigen::Vector3d>& lastCoordinates);
	Eigen::Vector3d calculate3dCoordinates(const Eigen::Vector2d& p);

private:
	ros::NodeHandle m_nh;

	//parameters
	std::string m_cameraInfoTopic, m_cameraRGBTopic, m_cameraDepthTopic;
	std::string m_cameraRGBFrame;

	//cascade
	cv::CascadeClassifier m_faceCascade, m_eyeCascade, m_mouthCascade;

	// 3d coordinate of mouth in real world (averaged)
	Eigen::Vector3d m_meanMouthPos, m_meanEyeLeftPos, m_meanEyeRightPos, m_meanForeHeadPos;
	cv_bridge::CvImagePtr m_cvBridgeRGB;
	cv_bridge::CvImagePtr m_cvBridgeDepth;

	// 2d coordinate of mouth in image
	Eigen::Vector2d m_mouth2DPos, m_eyeLeftPos, m_eyeRightPos, m_foreHearPos;

	// list for saving the last n mouth/eye ccordinates
	std::list<Eigen::Vector3d> m_lastMouthCoordinates;
	std::list<Eigen::Vector3d> m_lastEyeLeftCoordinates;
	std::list<Eigen::Vector3d> m_lastEyeRightCoordinates;
	std::list<Eigen::Vector3d> m_lastForeHeadCoordinates;
	tf::Transform m_transform, m_transformEyeLeft, m_transformEyeRight, m_transformForeHead;
	int m_framesWithoutInsertion;
	cv::Mat m_frameGray;

	// intrinsic camera parameter
	double m_cx;
	double m_cy;
	double m_fx;
	double m_fy;

	//camera info
	ros::Subscriber m_subscriberCameraInfo;
	bool m_gotCameraInfo;

	//rgb and depth image
	message_filters::Subscriber<sensor_msgs::Image> m_subscriberRGB;
	message_filters::Subscriber<sensor_msgs::Image> m_subscriberDepth;
	message_filters::Synchronizer<MySyncPolicy>* m_sync;

	//publishers
	ros::Publisher m_publisherCVImage;
	tf::TransformBroadcaster m_tfBroadcaster;
};

#endif /* HBACFDA4E_17BC_4593_BE24_794EE5D4DECF */
