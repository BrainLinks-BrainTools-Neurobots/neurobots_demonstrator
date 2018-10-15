/*
 *mouth_detect.cpp
 *
 *  Created on: September 11, 2016
 *      Author: Jeremias Holub
 */

#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <mouth_detection/mouth_detector.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Geometry>

MouthDetector::MouthDetector() :
				m_gotCameraInfo(false),
				m_meanMouthPos(Eigen::Vector3d::Zero()),
				m_framesWithoutInsertion(0)
{
	m_transform.setIdentity();
	m_transformEyeLeft.setIdentity();
	m_transformEyeRight.setIdentity();
	m_transformForeHead.setIdentity();

	// read calibrated intrinsic camera parameter
	ros::NodeHandle nParam("~");

	//get topics from parameter server
	nParam.getParam("camera_image_topic", m_cameraInfoTopic);
	nParam.getParam("camera_rgb_topic", m_cameraRGBTopic);
	nParam.getParam("camera_depth_topic", m_cameraDepthTopic);
	nParam.getParam("camera_rgb_frame", m_cameraRGBFrame);

	//camera
	m_subscriberCameraInfo = m_nh.subscribe<sensor_msgs::CameraInfo>(m_cameraInfoTopic, 100,
			boost::bind(&MouthDetector::parameterCallback, this, _1));

	// Load the face cascade
	std::string path = ros::package::getPath("mouth_detection");
	std::string face_cascade_name = path + "/haarcascade_frontalface_alt.xml";
	std::string eye_cascade_name = path + "/haarcascade_eyes.xml";
	std::string mouth_cascade_name = path + "/haarcascade_mcs_mouth.xml";

	if (!m_faceCascade.load(face_cascade_name)
			|| !m_eyeCascade.load(eye_cascade_name)
			|| !m_mouthCascade.load(mouth_cascade_name))
	{
		ROS_ERROR("Error loading face, eye, or mouth cascades");
		return;
	}
	else
	{
		ROS_INFO("Loading successful!");
	}

	m_subscriberRGB.subscribe(m_nh, m_cameraRGBTopic, 1);
	m_subscriberDepth.subscribe(m_nh, m_cameraDepthTopic, 1);

	m_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), m_subscriberRGB, m_subscriberDepth);
	m_sync->registerCallback(boost::bind(&MouthDetector::callback, this, _1, _2));

	m_publisherCVImage = nParam.advertise<sensor_msgs::Image>("face_detection", 10);
}

MouthDetector::~MouthDetector()
{
	delete m_sync;
}

void MouthDetector::parameterCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
	m_fx = msg->K[0];
	m_fy = msg->K[4];
	m_cx = msg->K[2];
	m_cy = msg->K[5];

	m_gotCameraInfo = true;

	m_subscriberCameraInfo.shutdown();
}

void MouthDetector::callback(const sensor_msgs::ImageConstPtr& msg_rgb,
		const sensor_msgs::ImageConstPtr& msg_depth)
{
	//wait until we got the camera info
	if (!m_gotCameraInfo)
		return;

	try
	{
		m_cvBridgeRGB = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	try
	{
		m_cvBridgeDepth = cv_bridge::toCvCopy(msg_depth, "32FC1");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	// mouth detection
	bool eyesDetected = false;
	if (detectFace(eyesDetected))
	{
		//reset last-element lists if there were no data for some time
		if (m_framesWithoutInsertion > 20)
		{
			m_lastEyeLeftCoordinates.clear();
			m_lastEyeRightCoordinates.clear();
			m_lastMouthCoordinates.clear();
			m_lastForeHeadCoordinates.clear();
		}

		// calculate mouth coordinate with depth data
		Eigen::Vector3d pos = calculate3dCoordinates(m_mouth2DPos);
		if ((!std::isnan(pos.x()) && !std::isnan(pos.y()) && !std::isnan(pos.z()))
				|| pos.norm() > 0.7)
		{
			if (!addToList(pos, m_meanMouthPos, m_lastMouthCoordinates))
				++m_framesWithoutInsertion;
			else
				m_framesWithoutInsertion = 0;
		}

		if (eyesDetected)
		{
			pos = calculate3dCoordinates(m_eyeLeftPos);
			if (!std::isnan(pos.x()) && !std::isnan(pos.y()) && !std::isnan(pos.z()))
				addToList(pos, m_meanEyeLeftPos, m_lastEyeLeftCoordinates);

			pos = calculate3dCoordinates(m_eyeRightPos);
			if (!std::isnan(pos.x()) && !std::isnan(pos.y()) && !std::isnan(pos.z()))
				addToList(pos, m_meanEyeRightPos, m_lastEyeRightCoordinates);

			pos = calculate3dCoordinates(m_foreHearPos);
			if (!std::isnan(pos.x()) && !std::isnan(pos.y()) && !std::isnan(pos.z()))
				addToList(pos, m_meanForeHeadPos, m_lastForeHeadCoordinates);
		}

		//compute mouth position based on the last detections
		m_meanMouthPos = calculateMeanPos(m_lastMouthCoordinates);
		m_meanEyeLeftPos = calculateMeanPos(m_lastEyeLeftCoordinates);
		m_meanEyeRightPos = calculateMeanPos(m_lastEyeRightCoordinates);
		m_meanForeHeadPos = calculateMeanPos(m_lastForeHeadCoordinates);

		//set transformations for tf
		tf::Quaternion q(0.471, 0.457, -0.490, 0.574);

		tf::Matrix3x3 qMat;
		tf::Vector3 col0 = tf::Vector3(-m_meanMouthPos.x(), -m_meanMouthPos.y(), -m_meanMouthPos.z()).normalized();
		tf::Vector3 col2(0, -1, 0);
		tf::Vector3 col1 = col2.cross(col0).normalized();
		col0 = col1.cross(col2).normalized();
		qMat.setValue(col0.x(), col1.x(), col2.x(),
				col0.y(), col1.y(), col2.y(),
				col0.z(), col1.z(), col2.z());

		//ROS_INFO_STREAM(col0.x() << " " << col0.y() << " " << col0.z());		
		//ROS_INFO_STREAM(col1.x() << " " << col1.y() << " " << col1.z());		
		//ROS_INFO_STREAM(col2.x() << " " << col2.y() << " " << col2.z() << "\n");		

		qMat.getRotation(q);

		q.normalize();
//		q.setRPY(0.0, 0.0, 0.0);

		if (m_meanMouthPos.hasNaN())
			return;

		m_transform.setOrigin(tf::Vector3(m_meanMouthPos(0), m_meanMouthPos(1), m_meanMouthPos(2)));

		if (!m_meanEyeLeftPos.hasNaN())
		{
			m_transformEyeLeft.setOrigin(tf::Vector3(m_meanEyeLeftPos(0), m_meanEyeLeftPos(1), m_meanEyeLeftPos(2)));
			m_transformEyeLeft.setRotation(q);
			eyesDetected = false;
		}

		if (!m_meanEyeRightPos.hasNaN())
		{
			m_transformEyeRight.setOrigin(tf::Vector3(m_meanEyeRightPos(0), m_meanEyeRightPos(1), m_meanEyeRightPos(2)));
			m_transformEyeRight.setRotation(q);
			eyesDetected = false;
		}

		if (!m_meanForeHeadPos.hasNaN())
		{
			m_transformForeHead.setOrigin(tf::Vector3(m_meanForeHeadPos(0), m_meanForeHeadPos(1), m_meanForeHeadPos(2)));
			m_transformForeHead.setRotation(q);
			eyesDetected = false;
		}

		//if the eye was detected or we have old data compute the orientation
		if (eyesDetected || !m_lastEyeLeftCoordinates.empty())
		{
			Eigen::Vector3d y = (m_meanEyeRightPos - m_meanEyeLeftPos).normalized();
			Eigen::Vector3d mouthToForehead = (m_meanForeHeadPos - m_meanMouthPos).normalized();
			Eigen::Vector3d z(0,-1,0);
			Eigen::Vector3d x = y.cross(z).normalized();//y.cross(mouthToForehead).normalized();
			Eigen::Matrix3d rot;
			rot.col(0) = x;
			rot.col(1) = y;
			rot.col(2) = z;
			Eigen::Quaterniond quat(rot);
			m_transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
		}
		//else we assume a default orientation
		else
		{
			m_transform.setRotation(q);
		}
	}

	// broadcast the mouth transform
	publish();
}

void MouthDetector::publish()
{
	m_tfBroadcaster.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), m_cameraRGBFrame, "mouth2"));
	m_tfBroadcaster.sendTransform(tf::StampedTransform(m_transformEyeLeft, ros::Time::now(), m_cameraRGBFrame, "eye_left"));
	m_tfBroadcaster.sendTransform(tf::StampedTransform(m_transformEyeRight, ros::Time::now(), m_cameraRGBFrame, "eye_right"));
	m_tfBroadcaster.sendTransform(tf::StampedTransform(m_transformForeHead, ros::Time::now(), m_cameraRGBFrame, "fore_head"));

	if (m_publisherCVImage.getNumSubscribers() > 0)
	{
		m_publisherCVImage.publish(m_cvBridgeRGB->toImageMsg());
	}
}

bool MouthDetector::detectFace(bool& eyesDetected)
{
	std::vector<cv::Rect> faces;
	std::vector<cv::Rect> eyes;
	std::vector<cv::Rect> mouths;

	eyesDetected = false;

	cv::cvtColor(m_cvBridgeRGB->image, m_frameGray, CV_BGR2GRAY);
	cv::equalizeHist(m_frameGray, m_frameGray);

	//detect face
	m_faceCascade.detectMultiScale(m_frameGray, faces, 1.15, 3,
			0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 60));

	for (int i = 0; i < faces.size(); i++)
	{
		//eyes will only be detected in a small area in the face
		//to reduce false positives
		double hSliceEyes = 0.4 * faces[i].height;
		cv::Rect rectEyes = faces[i];
		rectEyes.height = hSliceEyes;
		rectEyes.y += 0.2 * faces[i].height;
		cv::Mat roiEyes = m_frameGray(rectEyes);
		m_eyeCascade.detectMultiScale(roiEyes, eyes);

		if (eyes.size() >= 2)
		{
			//compute left and right eye position
			if (eyes[0].x < eyes[1].x)
			{
				m_eyeLeftPos.x() = eyes[0].x + eyes[0].width * 0.5 + faces[i].x;
				m_eyeLeftPos.y() = eyes[0].y + eyes[0].height * 0.5 + rectEyes.y;
				m_eyeRightPos.x() = eyes[1].x + eyes[1].width * 0.5 + faces[i].x;
				m_eyeRightPos.y() = eyes[1].y + eyes[1].height * 0.5 + rectEyes.y;
			}
			else
			{
				m_eyeLeftPos.x() = eyes[1].x + eyes[1].width * 0.5 + faces[i].x;
				m_eyeLeftPos.y() = eyes[1].y + eyes[1].height * 0.5 + rectEyes.y;
				m_eyeRightPos.x() = eyes[0].x + eyes[0].width * 0.5 + faces[i].x;
				m_eyeRightPos.y() = eyes[0].y + eyes[0].height * 0.5 + rectEyes.y;
			}

			//compute fore head: take center between eyes and go up a bit, lets say 4cm
			Eigen::Vector2d center = ((m_eyeRightPos + m_eyeLeftPos) / 2.0);
			Eigen::Vector2d leftToRight = (m_eyeRightPos - m_eyeLeftPos).normalized();
			center += (Eigen::Vector2d(leftToRight.y(), -leftToRight.x()) * 30.0);
			m_foreHearPos = center;

			//draw fore head
			cv::Point fh(m_foreHearPos.x(), m_foreHearPos.y());
			cv::circle(m_cvBridgeRGB->image, fh, 7.0, cv::Scalar(0, 128, 0), 2, 8, 0);
			cv::Point eyeLeft(m_eyeLeftPos.x(), m_eyeLeftPos.y());
			cv::circle(m_cvBridgeRGB->image, eyeLeft, 7.0, cv::Scalar(0, 255, 0), 2, 8, 0);
			cv::Point eyeRight(m_eyeRightPos.x(), m_eyeRightPos.y());
			cv::circle(m_cvBridgeRGB->image, eyeRight, 7.0, cv::Scalar(0, 255, 0), 2, 8, 0);

			eyesDetected = true;
		}

		//detect mouth in face
		cv::Point mouth;
		double hSliceMouth = 0.3 * faces[i].height;
		cv::Rect rectMouth = faces[i];
		rectMouth.height = hSliceMouth;
		rectMouth.y += 0.65 * faces[i].height;
		cv::Mat roiMouth = m_frameGray(rectMouth);
		m_mouthCascade.detectMultiScale(roiMouth, mouths);

		if (mouths.size() >= 1)
		{
			m_mouth2DPos.x() = mouth.x = mouths[0].x + mouths[0].width * 0.5 + faces[i].x;
			m_mouth2DPos.y() = mouth.y = mouths[0].y + mouths[0].height * 0.5 + rectMouth.y;
		}
		else
		{
			//compute mouth position based on assumption that it is at 0.78 * height
			m_mouth2DPos.x() = mouth.x = faces[i].x + faces[i].width * 0.5;
			m_mouth2DPos.y() = mouth.y = faces[i].y + faces[i].height * 0.78;
		}

		//draw mouth
		cv::circle(m_cvBridgeRGB->image, mouth, 9.0, cv::Scalar(0, 0, 255), 4, 8, 0);

		//draw face and search area for eyes and mouth
		cv::rectangle(m_cvBridgeRGB->image, rectEyes, cv::Scalar(0, 0, 255), 1, 8, 0);
		cv::rectangle(m_cvBridgeRGB->image, rectMouth, cv::Scalar(0, 0, 255), 1, 8, 0);
		cv::rectangle(m_cvBridgeRGB->image, faces[i], cv::Scalar(255, 0, 0), 2, 8, 0);

		return true;
	}

	return false;
}

bool MouthDetector::addToList(const Eigen::Vector3d &pos,
		const Eigen::Vector3d& mean,
		std::list<Eigen::Vector3d>& lastCoordinates)
{
	// number of mouth positions over which will be averaged later
	static const int numOfMouthPositions = 10;

	//something is wrong (moved more than 10 cm)
	if (!lastCoordinates.empty() && (pos - mean).norm() > 0.1)
		return false;

	lastCoordinates.push_back(pos);
	if (lastCoordinates.size() > numOfMouthPositions)
		lastCoordinates.pop_front();

	return true;
}

Eigen::Vector3d MouthDetector::calculateMeanPos(const std::list<Eigen::Vector3d>& lastCoordinates)
{
	Eigen::Vector3d mean(0, 0, 0);
	for (auto& it : lastCoordinates)
	{
		mean += it;
	}

	mean /= lastCoordinates.size();
	return mean;

}

Eigen::Vector3d MouthDetector::calculate3dCoordinates(const Eigen::Vector2d& p)
{
	Eigen::Vector3d mean(0, 0, 0);
	Eigen::Vector2i pi = p.cast<int>();
	int counter = 0;
	for (int ix = std::max(pi.x() - 1, 0); ix <= std::max(pi.x() + 1, m_cvBridgeDepth->image.cols); ++ix)
	{
		for (int iy = std::max(pi.y() - 1, 0); iy <= std::max(pi.y() + 1, m_cvBridgeDepth->image.rows); ++iy)
		{
			double z = m_cvBridgeDepth->image.at<float>(pi.y(), pi.x());
			double x = (pi.x() - m_cx) * z / m_fx;
			double y = (pi.y() - m_cy) * z / m_fy;
			mean += Eigen::Vector3d(x, y, z);
			++counter;
		}
	}

	return mean / counter;
}

