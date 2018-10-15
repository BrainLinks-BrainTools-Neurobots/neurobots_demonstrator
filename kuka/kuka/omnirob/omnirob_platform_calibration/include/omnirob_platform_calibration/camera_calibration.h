/*
 * camera_calibration.h
 *
 * Copyright (C) 2015 Daniel Kuhner (kuhnerd@informatik.uni-freiburg.de)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef KUKA_omnirob_platform_calibration_CAMERA_CALIBRATION_H_
#define KUKA_omnirob_platform_calibration_CAMERA_CALIBRATION_H_

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>
#include <aruco.h>
#include <qmutex.h>
#include <qthread.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/spinner.h>
#include <Eigen/Geometry>

#include "board.h"
#include "boarddetector.h"

namespace omnirob_platform_calibration
{

class CameraCalibration
{
public:
	CameraCalibration(int argc,
			char** argv,
			double markerSize);
	virtual ~CameraCalibration();

	void run();

	int getIterations() const;
	void setIterations(int iterations);
	const std::string& getPosesFileNameRead() const;
	void setPosesFileNameRead(const std::string& posesFileNameRead);
	const std::string& getPosesFileNameWrite() const;
	void setPosesFileNameWrite(const std::string& posesFileNameWrite);
	bool isReadPosesFromFile() const;
	void setReadPosesFromFile(bool readPosesFromFile);
	double getThreshold() const;
	void setThreshold(double threshold);
	void setDirection(int direction);
	int getDirection() const;
	bool isWritePosesToFile() const;
	void setWritePosesToFile(bool writePosesToFile);
	bool isTestCase() const;
	void setTestCase(bool testCase);
	const Eigen::VectorXd& getParameters() const;
	void setParameters(const Eigen::VectorXd& parameters);
	bool isPublishMarkerArray() const;
	void setPublishMarkerArray(bool publishMarkerArray);
	bool isSaveCalibration() const;
	void setSaveCalibration(const std::string& cal_save_path);
	const std::string& getBoardConfigFile() const;
	void setBoardConfigFile(const std::string& boardConfigFile);
	const std::string& getCalibrationResultPath() const;
	void setCalibrationResultPath(const std::string& calibrationResultPath);
	const std::string& getCameraPrefix() const;
	void setCameraPrefix(const std::string& cameraPrefix);
	int getCalibrationPatternId() const;
	void setCalibrationPatternId(int calibrationPatternId);

private:
	void showImage();

	void callbackImages(const sensor_msgs::Image::ConstPtr& imageDepth,
			const sensor_msgs::Image::ConstPtr& imageRgb);
	void callbackCameraInfoRGB(const sensor_msgs::CameraInfo::ConstPtr& rgbCameraInfo);
	void callbackCameraInfoDepth(const sensor_msgs::CameraInfo::ConstPtr& depthCameraInfo);

	bool optimize();
	Eigen::Affine3d transformToCameraBaseLink(Eigen::Affine3d& matrix);

	//help functions
	Eigen::Matrix3d computeRotationMatrix(const double theta1,
			const double theta2,
			const double theta3);
	Eigen::Matrix3d computeRotationMatrixDerivative(const double theta1,
			const double theta2,
			const double theta3,
			const int partialDerivativeIndex);
	Eigen::Matrix3d skewSymmetricMatrix(const double theta1,
			const double theta2,
			const double theta3);
	Eigen::Matrix3d skewSymmetricMatrixDerivative(const int partialDerivativeIndex);
	Eigen::Matrix3d getMultiplicationDerivative(const double theta1,
			const double theta2,
			const double theta3,
			const int partialDerivativeIndex);
	Eigen::VectorXd computePredictedMarkerDistance(const Eigen::VectorXd& X);
	Eigen::Vector4d computePredictedMarkerPose(const Eigen::VectorXd& X,
			const Eigen::Matrix4d& tEEf);
	Eigen::VectorXd buildMeassurementVector();
	Eigen::VectorXd computeResidual(const Eigen::VectorXd& meassurement,
			const Eigen::VectorXd& predicted);
	Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& X);

	Eigen::Matrix4d buildTransformation(const Eigen::VectorXd& X);
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& a,
			double epsilon = std::numeric_limits<double>::epsilon());
	void normalizeAngles(Eigen::VectorXd& X);

	Eigen::Affine3d getTransformToCameraBaseLink();

	inline void convert2DTo3DDepth(const double& x,
			const double& y,
			const double& depth,
			double& xOut,
			double& yOut,
			double& zOut)
	{
		static const double cx = m_camInfoDepth->K[2];
		static const double cy = m_camInfoDepth->K[5];
		static const double fx = 1.0/m_camInfoDepth->K[0];
		static const double fy = 1.0/m_camInfoDepth->K[4];
		xOut = ((x - cx) * depth * fx);
		yOut = ((y - cy) * depth * fy);
		zOut = depth;
	}

	void buildTestCase();

	void writePoses(const std::string& file);
	void readPoses(const std::string& file);
	void saveCalibration(const Eigen::Affine3d& t);

	void printErrors(const Eigen::VectorXd& X);
	void printConfig();
	void printParameters(const Eigen::VectorXd& X);
	void printTransformation(const Eigen::Affine3d& t);

	void visualize(const Eigen::Affine3d& matrix,
			const Eigen::Vector3d& markerPos);

private:
	tf::TransformListener* m_transformListener;
	ros::Publisher m_visPub;
	ros::Publisher m_imagePub;
	ros::Subscriber m_subscriberCameraInfoRGB;
	ros::Subscriber m_subscriberCameraInfoDepth;
	ros::ServiceClient m_serviceClient;
	ros::AsyncSpinner* m_spinner;
	QMutex m_mutex;

	aruco::MarkerDetector m_markerDetector;
	aruco::BoardConfiguration m_boardConfig;
	aruco::BoardDetector m_boardDetector;

	aruco::CameraParameters m_cameraParameters;

	bool m_saveData;
	int m_armPoseId;
	bool m_cameraInfoReceivedRGB;
	bool m_cameraInfoReceivedDepth;
	const double m_markerSize;
	std::string m_boardConfigFile;
	bool m_readPosesFromFile;
	bool m_testCase;
	bool m_writePosesToFile;
	std::string m_posesFileNameRead;
	std::string m_posesFileNameWrite;
	int m_iterations;
	double m_threshold;
	Eigen::VectorXd m_parameters;
	bool m_publishMarkerArray;
	std::string m_saveCalibration;
	std::string m_calibrationResultPath;
	int m_listenToTopicsSeconds;
	std::string m_cameraPrefix;
	int m_direction;
	int m_calibrationPatternId;

	sensor_msgs::CameraInfo::ConstPtr m_camInfoDepth;

	std::vector<Eigen::Affine3d> m_markerPoses;
	std::vector<Eigen::Affine3d> m_robotPoses;

	cv::Mat m_image;
};

} /* namespace omnirob_platform_calibration */

#endif /* KUKA_omnirob_platform_calibration_CAMERA_CALIBRATION_H_ */
