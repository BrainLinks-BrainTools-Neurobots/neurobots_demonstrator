/*
 * aruco_detector.h
 *
 *  Created on: Aug 31, 2018
 *      Author: nematoli
 */

#ifndef INCLUDE_ARUCO_ARUCO_DETECTOR_H_
#define INCLUDE_ARUCO_ARUCO_DETECTOR_H_
#include <ros/ros.h>
#include <aruco/aruco_helpers.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "boarddetector.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


class ArucoDetector
{
public:

	void set_tf_frame(const Eigen::Affine3d& marker);
	void setSubscirerCameraInfoRGB(ros::Subscriber sub) {m_subscriberCameraInfoRGB = sub;}
	void setSubscirerImage(ros::Subscriber sub) {m_subscriberImage = sub;}
	void setCameraInfoReceivedRGB(bool state) {m_cameraInfoReceivedRGB = state;}
	void setCameraParameters(aruco::CameraParameters parameters) {m_cameraParameters = parameters;}
	void setMarkerSize(double size) {m_markerSize = size;}


	ros::Subscriber getSubscirerCameraInfoRGB() {return m_subscriberCameraInfoRGB;}
	ros::Subscriber getSubscirerImage() {return m_subscriberImage;}
	bool getCameraInfoReceivedRGB() {return m_cameraInfoReceivedRGB;}
	aruco::CameraParameters getCameraParameters() {return m_cameraParameters;}
	aruco::MarkerDetector getMarkerDetector(){return m_markerDetector;}
	double getMarkerSize() {return m_markerSize;}


private:

	ros::Subscriber m_subscriberCameraInfoRGB;
	ros::Subscriber m_subscriberImage;

	bool m_cameraInfoReceivedRGB = false;

	double m_markerSize;

	aruco::CameraParameters m_cameraParameters;
	aruco::MarkerDetector m_markerDetector;
	aruco::BoardConfiguration m_boardConfig;
	aruco::BoardDetector m_boardDetector;

    tf::StampedTransform m_transform;
	tf::TransformBroadcaster m_br;

};


#endif /* INCLUDE_ARUCO_ARUCO_DETECTOR_H_ */
