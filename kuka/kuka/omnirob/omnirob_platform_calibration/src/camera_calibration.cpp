/*
 * camera_calibration.cpp
 *
 * Copyright (C) 2015 Daniel Kuhner (kuhnerd@informatik.uni-freiburg.de)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <aruco/aruco_helpers.h>
#include <boost/filesystem.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf_conversions/tf_eigen.h>
#include <qfile.h>
#include <qtextstream.h>
#include <visualization_msgs/MarkerArray.h>
#include <omnirob_platform_calibration/camera_calibration.h>
#include <kuka_manager/JointState.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <cvdrawingutils.h>
#include <termios.h>
#include <random>

#define ANSI_RESET   			"\033[0m"

#define ANSI_BLACK   			"\033[30m"
#define ANSI_RED     			"\033[31m"
#define ANSI_GREEN   			"\033[32m"
#define ANSI_YELLOW  			"\033[33m"
#define ANSI_BLUE    			"\033[34m"
#define ANSI_MAGENTA 			"\033[35m"
#define ANSI_CYAN    			"\033[36m"
#define ANSI_WHITE   			"\033[37m"

#define ANSI_BACK_BLACK   		"\033[40m"
#define ANSI_BACK_RED     		"\033[41m"
#define ANSI_BACK_GREEN   		"\033[42m"
#define ANSI_BACK_YELLOW  		"\033[43m"
#define ANSI_BACK_BLUE    		"\033[44m"
#define ANSI_BACK_MAGENTA 		"\033[45m"
#define ANSI_BACK_CYAN    		"\033[46m"
#define ANSI_BACK_WHITE   		"\033[47m"
#define ANSI_BACK_LIGHT_BLUE   	"\033[104m"

#define ANSI_BOLD				"\033[1m"
#define ANSI_ITALIC   			"\033[3m"

#define BOOL_TO_STRING(VAR) VAR ? "true" : "false"

namespace enc = sensor_msgs::image_encodings;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

namespace omnirob_platform_calibration
{

CameraCalibration::CameraCalibration(int argc,
		char** argv,
		double markerSize) :
				m_cameraInfoReceivedRGB(false),
				m_cameraInfoReceivedDepth(false),
				m_markerSize(markerSize),
				m_readPosesFromFile(false),
				m_writePosesToFile(false),
				m_posesFileNameRead("poses.txt"),
				m_posesFileNameWrite("poses.txt"),
				m_iterations(100),
				m_direction(1),
				m_threshold(1e-15),
				m_testCase(false),
				m_transformListener(NULL),
				m_spinner(NULL),
				m_parameters(9),
				m_publishMarkerArray(false),
				m_saveCalibration(""),
				m_listenToTopicsSeconds(-1),
				m_saveData(false),
				m_armPoseId(0),
				m_calibrationPatternId(0)
{
	ros::init(argc, argv, "omnirob_platform_calibration");
	ros::NodeHandle n;
	sleep(1);
	m_transformListener = new tf::TransformListener;

	std::uniform_real_distribution<double> dist(-10, 10);
	std::random_device gen;
//	m_parameters << 0, 1, 0, -2.391670, 1.4, 2.190303, 0.1, 0.1, 0.1;
	m_parameters << dist(gen), dist(gen), dist(gen), dist(gen), dist(gen), dist(gen), dist(gen), dist(gen), dist(gen);
}

CameraCalibration::~CameraCalibration()
{
	if (m_spinner != NULL)
		delete m_spinner;

	if (m_transformListener != NULL)
		delete m_transformListener;
}

void CameraCalibration::run()
{
	ros::NodeHandle nh;
	ros::Rate r(30);

//	cv::namedWindow("window", CV_WINDOW_AUTOSIZE);

	m_boardConfig.readFromFile(m_boardConfigFile);

	m_imagePub = nh.advertise<sensor_msgs::Image>("marker_image", 1);

	m_subscriberCameraInfoRGB = nh.subscribe(m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera/rgb/camera_info", 1,
			&CameraCalibration::callbackCameraInfoRGB, this);
	m_subscriberCameraInfoDepth = nh.subscribe(m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera/depth_registered/camera_info", 1,
			&CameraCalibration::callbackCameraInfoDepth, this);
	m_serviceClient = nh.serviceClient<kuka_manager::JointState>("omnirob_lbr/omnirob/cmd_joint_state");

	printConfig();

	//load trajectory from file
	if (m_readPosesFromFile)
	{
		readPoses(m_posesFileNameRead);
		sleep(1); //wait until tf is ready
	}
	//run test case
	else if (m_testCase)
	{
		buildTestCase();
	}
	//error...
	else
	{
//		cv::namedWindow("window", CV_WINDOW_AUTOSIZE);

		message_filters::Subscriber<sensor_msgs::Image> depthSub(nh,
				m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera/depth_registered/image_raw", 1);
		message_filters::Subscriber<sensor_msgs::Image> colorSub(nh, m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera/rgb/image_raw",
				1);
		message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), depthSub, colorSub);
		sync.registerCallback(boost::bind(&CameraCalibration::callbackImages, this, _1, _2));

		m_subscriberCameraInfoRGB = nh.subscribe(m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera/rgb/camera_info", 1,
				&CameraCalibration::callbackCameraInfoRGB, this);

		m_spinner = new ros::AsyncSpinner(0);
		m_spinner->start();

		kuka_manager::JointState j;
		j.request.header.stamp = ros::Time::now();
		j.request.jointState.name =
		{	"lbr_1_joint", "lbr_2_joint", "lbr_3_joint", "lbr_4_joint", "lbr_5_joint", "lbr_6_joint", "lbr_7_joint"};
		j.request.jointState.position =
		{	0, 0, -M_PI / 2.0, 0, 0, 0, 0};
		ROS_INFO_STREAM(m_serviceClient.call(j));

		while (ros::ok())
		{
			showImage();
			int key = getch(); //cv::waitKey(10) & 0xFF;

			if (key != 255 && !m_saveData)
			{
				std::cout << key << std::endl;

				//enter
				if (key == 10)
				{
					//move joint
					++m_armPoseId;

//					switch(m_armPoseId) {
//										case 1:
//											j.request.header.stamp = ros::Time::now();
//											j.request.jointState.position = {-0.3658479559925705, 1.6203033975626129, 1.0741453908123775, 1.6209709336359501, 1.4480260500860622, 0.8960725903373686, -0.008969054963559271};
//											m_serviceClient.call(j);
//											break;
//										case 2:
//											j.request.header.stamp = ros::Time::now();
//											j.request.jointState.position = {-1.1371909414620065, 1.6209605314736966, 1.8087456179146177, 1.3754740750540442, 1.2577303948039453, 1.462445331924969, -0.008969054963559271};
//											m_serviceClient.call(j);
//											break;
//										case 3:
//											j.request.header.stamp = ros::Time::now();
//											j.request.jointState.position = {-2.0872769861933986, 1.6216098114032134, 1.679973132401723, 1.4787416106386893, 2.281396378581459, 1.4568668058973977, 1.6648915155706057};
//											m_serviceClient.call(j);
//											break;
//										case 4:
//											j.request.header.stamp = ros::Time::now();
//											j.request.jointState.position = {-2.36385904860707, 1.2470808146778, 1.4336797230092688, 1.6191530510623993, 1.307414996357498, 1.4231396099328781, 1.9072723331456523};
//											m_serviceClient.call(j);
//											break;
//										case 5:
//											j.request.header.stamp = ros::Time::now();
//											j.request.jointState.position = {-1.3668048327468445, 1.6161311705678847, 1.9104330894875678, 0.6705613746771577, 1.1835760155459967, 1.3560808143287342, -0.7425215260181466};
//											m_serviceClient.call(j);
//											break;
//										}

//					if (m_calibrationPatternId == 0)
//					{
					//marker auf karton
//						switch (m_armPoseId)
//						{
//							case 1:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	0, 0.8, -1.3, 0, 0, 0, 0};
//								m_serviceClient.call(j);
//								break;
//								case 2:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	0, 1.2, -1.1, 0, 0, 0, 0};
//								m_serviceClient.call(j);
//								break;
//								case 3:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	0, 1.7, -1.9, 0, 0, 0, 0};
//								m_serviceClient.call(j);
//								break;
//								case 4:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	0, -0.9, -1.2, 0, 0, 0, 0};
//								m_serviceClient.call(j);
//								break;
//								case 5:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	0, -1.2, -M_PI / 2.0, 0, 0, 0, 0};
//								m_serviceClient.call(j);
//								break;
//								case 6:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	0, -1.1, -M_PI / 2.0, 0, 0, 0, 0};
//								m_serviceClient.call(j);
//								break;
//							}

					//marker for overview camera
					switch (m_armPoseId)
					{
						case 1:
							j.request.header.stamp = ros::Time::now();
							j.request.jointState.position =
							{	0, 0.8, -M_PI / 2.0 + 0.1, 0, 0, 0, 0};
							m_serviceClient.call(j);
							break;
							case 2:
							j.request.header.stamp = ros::Time::now();
							j.request.jointState.position =
							{	0, 1.2, -M_PI / 2.0 + 0.15, 0, 0, 0, 0};
							m_serviceClient.call(j);
							break;
							case 3:
							j.request.header.stamp = ros::Time::now();
							j.request.jointState.position =
							{	0, 1.4, -M_PI / 2.0 -0.05, 0, 0, 0, 0};
							m_serviceClient.call(j);
							break;
							case 4:
							j.request.header.stamp = ros::Time::now();
							j.request.jointState.position =
							{	0, -0.6, -M_PI / 2.0 -0.1, 0, 0, 0, 0};
							m_serviceClient.call(j);
							break;
							case 5:
							j.request.header.stamp = ros::Time::now();
							j.request.jointState.position =
							{	0, -0.9, -M_PI / 2.0, 0, 0, 0, 0};
							m_serviceClient.call(j);
							break;
							case 6:
							j.request.header.stamp = ros::Time::now();
							j.request.jointState.position =
							{	0, -0.7, -M_PI / 2.0, 0, 0, 0, 0};
							m_serviceClient.call(j);
							break;
						}
//						}
//						else
//						{
//							switch(m_armPoseId)
//							{
//								case 1:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	-1.738623647326208, -1.3946492394378782, -2.01627044784101, 1.4577042323186804, 1.956339733921067, -0.5932500788016151, -2.0113587772083736};
//								m_serviceClient.call(j);
//								break;
//								case 2:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	-1.8031684183372667, -1.4466446569014333, -2.091573585349567, 1.2524806538980706, 2.3257776925281406, -0.6480693866110209, -2.006168674202533};
//								m_serviceClient.call(j);
//								break;
//								case 3:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	-2.048505614200557, -1.5558895262443413, -2.341463978673473, 1.5012181525410773, 2.0881599482544786, -0.8435462786264538, -2.451825649288656};
//								m_serviceClient.call(j);
//								break;
//								case 4:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	-1.969533829542071, -1.3222795640446818, -2.3566045876328063, 1.2726718093374416, 1.9786502426531238, -0.9252666426742074, -1.8546743582162595};
//								m_serviceClient.call(j);
//								break;
//								case 5:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	-1.8288102718908792, -1.4785606549033803, -2.35975164314021, 1.1656510828546187, 1.9007607541078941, -0.7312947655132717, -1.191251781544819};
//								m_serviceClient.call(j);
//								break;
//								case 6:
//								j.request.header.stamp = ros::Time::now();
//								j.request.jointState.position =
//								{	-1.6815289363370216, -0.8830014706144153, -2.329466969469654, 1.2442176544241128, 1.4655152962399673, -1.445205615492101, -2.086491605489968};
//								m_serviceClient.call(j);
//								break;
//							}
//						}

						//stop data collection
					if (m_armPoseId > 6)
						break;
				}
				//space
				else if (key == 32)
				{
					ROS_INFO("Added pose");
					m_saveData = true;
				}
			}

			r.sleep();
		}

		m_spinner->stop();

//		cv::destroyWindow("window");
	}

	optimize();

	if (m_writePosesToFile)
	{
		writePoses(m_posesFileNameWrite);
	}
}

int CameraCalibration::getIterations() const
{
	return m_iterations;
}

void CameraCalibration::setIterations(int iterations)
{
	m_iterations = iterations;
}

const std::string& CameraCalibration::getPosesFileNameRead() const
{
	return m_posesFileNameRead;
}

void CameraCalibration::setPosesFileNameRead(const std::string& posesFileNameRead)
{
	m_posesFileNameRead = posesFileNameRead;
}

const std::string& CameraCalibration::getPosesFileNameWrite() const
{
	return m_posesFileNameWrite;
}

void CameraCalibration::setPosesFileNameWrite(const std::string& posesFileNameWrite)
{
	m_posesFileNameWrite = posesFileNameWrite;
}

bool CameraCalibration::isReadPosesFromFile() const
{
	return m_readPosesFromFile;
}

void CameraCalibration::setReadPosesFromFile(bool readPosesFromFile)
{
	m_readPosesFromFile = readPosesFromFile;
}

double CameraCalibration::getThreshold() const
{
	return m_threshold;
}

void CameraCalibration::setThreshold(double threshold)
{
	m_threshold = threshold;
}

int CameraCalibration::getDirection() const
{
	return m_direction;
}

void CameraCalibration::setDirection(int direction)
{
	m_direction = direction;
}

bool CameraCalibration::isWritePosesToFile() const
{
	return m_writePosesToFile;
}

void CameraCalibration::setWritePosesToFile(bool writePosesToFile)
{
	m_writePosesToFile = writePosesToFile;
}

bool CameraCalibration::isTestCase() const
{
	return m_testCase;
}

void CameraCalibration::setTestCase(bool testCase)
{
	m_testCase = testCase;
}

const Eigen::VectorXd& CameraCalibration::getParameters() const
{
	return m_parameters;
}

void CameraCalibration::setParameters(const Eigen::VectorXd& parameters)
{
	m_parameters = parameters;
}

bool CameraCalibration::isPublishMarkerArray() const
{
	return m_publishMarkerArray;
}

void CameraCalibration::setPublishMarkerArray(bool publishMarkerArray)
{
	m_publishMarkerArray = publishMarkerArray;
}

bool CameraCalibration::isSaveCalibration() const
{
	if (m_saveCalibration.empty())
		return false;
	else
		return true;
}

void CameraCalibration::setSaveCalibration(const std::string& cal_save_path)
{
	m_saveCalibration = cal_save_path;
}

const std::string& CameraCalibration::getBoardConfigFile() const
{
	return m_boardConfigFile;
}

void CameraCalibration::setBoardConfigFile(const std::string& boardConfigFile)
{
	m_boardConfigFile = boardConfigFile;
}

const std::string& CameraCalibration::getCalibrationResultPath() const
{
	return m_calibrationResultPath;
}

void CameraCalibration::setCalibrationResultPath(const std::string& calibrationResultPath)
{
	m_calibrationResultPath = calibrationResultPath;
}

const std::string& CameraCalibration::getCameraPrefix() const
{
	return m_cameraPrefix;
}

void CameraCalibration::setCameraPrefix(const std::string& cameraPrefix)
{
	m_cameraPrefix = cameraPrefix;
}

int CameraCalibration::getCalibrationPatternId() const
{
	return m_calibrationPatternId;
}

void CameraCalibration::setCalibrationPatternId(int calibrationPatternId)
{
	m_calibrationPatternId = calibrationPatternId;
}

void CameraCalibration::callbackImages(const sensor_msgs::Image::ConstPtr& imageDepth,
		const sensor_msgs::Image::ConstPtr& imageRgb)
{
	if (!m_cameraInfoReceivedRGB)
	{
		return;
	}

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

		//get depth value from depth image
		float depth = depthImage.at<float>(marker.getCenter().y, marker.getCenter().x);
		if (std::isnan(depth))
		{
			ROS_INFO("NaN in Depth value. Ignoring marker pose");
			return;
		}

		Eigen::Vector3d tempPos;
		// Quick hack. If camera publishes depth in mm, divide by 1000
		if (depth > 100)
		{
			depth /= 1000.0;
		}
		tempPos << marker.Tvec.at<float>(0, 0), marker.Tvec.at<float>(1, 0), depth;
		markerCenterPoints.push_back(tempPos);
		// ROS_INFO_STREAM(tempPos.transpose());
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
		m_mutex.lock();
		m_image = colorImage;
		m_mutex.unlock();

		m_saveData = false;

		return;
	}

	aruco::Board board;
	float probDetect = m_boardDetector.detect(markers, m_boardConfig, board, m_cameraParameters, m_markerSize);

	aruco::CvDrawingUtils::draw3dAxis(colorImage, board, m_cameraParameters);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImage).toImageMsg();
	m_imagePub.publish(msg);

	m_mutex.lock();
	m_image = colorImage;
	m_mutex.unlock();

	if (m_saveData)
	//	double x, y, z;
//	ROS_INFO("%f, %f, %f", markers[0][0].x, markers[0][0].y, depthImage.at<float>(markers[0][0].y, markers[0][0].x));
//	convert2DTo3DDepth(markers[0][0].x, markers[0][0].y, depthImage.at<float>(markers[0][0].y, markers[0][0].x) / 1000.0, x, y, z);
//
//	ROS_INFO("d: %f, %f, %f", x, y, z);
//	ROS_INFO("c: %f, %f, %f", markers[0].Tvec.at<float>(0, 0), markers[0].Tvec.at<float>(1, 0), markers[0].Tvec.at<float>(2, 0));
	{
		Eigen::Affine3d tMarker = aruco_helpers::toEigen(board);

		//get transformation from robot
		tf::StampedTransform tf;
		try
		{
			m_transformListener->lookupTransform("/map", "/omnirob_lbr/lbr_flange_link", imageDepth->header.stamp, tf);
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
		m_saveData = false;

//		ROS_INFO("Robot pose:");
//		ROS_INFO_STREAM(tRobot.matrix());
//		ROS_INFO("Marker pose:");
//		ROS_INFO_STREAM(tMarker.matrix());
	}
}

void CameraCalibration::callbackCameraInfoRGB(const sensor_msgs::CameraInfo::ConstPtr& rgbCameraInfo)
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

//	m_cameraParameters.Distorsion = cv::Mat(4, 1, CV_32FC1);
//	m_cameraParameters.Distorsion.at<float>(0, 0) = 0;//rgbCameraInfo->D[0];
//	m_cameraParameters.Distorsion.at<float>(1, 0) = 0;//rgbCameraInfo->D[1];
//	m_cameraParameters.Distorsion.at<float>(2, 0) = 0;//rgbCameraInfo->D[2];
//	m_cameraParameters.Distorsion.at<float>(3, 0) = 0;//rgbCameraInfo->D[3];
//
//	m_cameraParameters.resize(cv::Size(1280, 1024));

	m_subscriberCameraInfoRGB.shutdown();
}

void CameraCalibration::callbackCameraInfoDepth(const sensor_msgs::CameraInfo::ConstPtr& depthCameraInfo)
{
	if (m_cameraInfoReceivedDepth)
	{
		return;
	}

	m_camInfoDepth = depthCameraInfo;
	m_cameraInfoReceivedDepth = true;
}

void CameraCalibration::showImage()
{
	QMutexLocker lock(&m_mutex);

	if (m_image.empty())
	{
		return;
	}

//	cv::imshow("window", m_image);
}

bool CameraCalibration::optimize()
{
	ROS_INFO(" ");
	ROS_INFO("Starting optimization with %zu marker poses and %zu end-effector poses...",
			m_markerPoses.size(), m_robotPoses.size());

	Eigen::VectorXd delta;

	int i = 0;
	for (; i < m_iterations; ++i)
	{
		Eigen::VectorXd zPredicted = computePredictedMarkerDistance(m_parameters);
		Eigen::VectorXd zMeassured = buildMeassurementVector();

//		ROS_INFO_STREAM(i << " pred: " << zPredicted.transpose());
//		ROS_INFO_STREAM(i << " meas: " << zMeassured.transpose());

		Eigen::VectorXd epsilon = computeResidual(zMeassured, zPredicted);
		//printErrors(epsilon);

		Eigen::MatrixXd j = computeJacobian(m_parameters);
		Eigen::MatrixXd w(j.rows(), j.rows());
		w.setIdentity();

//		for (int wIndex = 0; wIndex < w.rows(); ++wIndex)
//		{
//			w(wIndex, wIndex) = 1.0 / sqrt(1.0 + 0.5 * pow((double) epsilon(wIndex), 2.0));
//		}

//		LOG_INF

		Eigen::MatrixXd A = (j.transpose() * w * j);
		Eigen::VectorXd b = (-j).transpose() * w * epsilon;
		delta = A.fullPivHouseholderQr().solve(b);
//		delta = pseudoInverse(-j) * w * epsilon;
		m_parameters = m_parameters + delta;
		normalizeAngles(m_parameters);

//		std::cout << ANSI_BACK_GREEN << "zPredicted:\n" << ANSI_RESET << zPredicted << std::endl;
//		std::cout << ANSI_BACK_GREEN << "zMeassured:\n" << ANSI_RESET << zMeassured << std::endl;
//		std::cout << ANSI_BACK_GREEN << "epsilon:\n" << ANSI_RESET << epsilon << std::endl;
//		std::cout << ANSI_BACK_GREEN << "j:\n" << ANSI_RESET << j << std::endl;
//		std::cout << ANSI_BACK_GREEN << "w:\n" << ANSI_RESET << w << std::endl;
//		std::cout << ANSI_BACK_GREEN << "hesse:\n" << ANSI_RESET << hesse << std::endl;
//		std::cout << ANSI_BACK_GREEN << "hesseInv:\n" << ANSI_RESET << hesseInv << std::endl;
//		std::cout << ANSI_BACK_GREEN << "result:\n" << ANSI_RESET << (hesse * hesseInv) << std::endl;
//		std::cout << ANSI_BACK_GREEN << "b:\n" << ANSI_RESET << b << std::endl;
//		std::cout << ANSI_BACK_GREEN << "c:\n" << ANSI_RESET << c << std::endl;
//		std::cout << ANSI_BACK_GREEN << "delta:\n" << ANSI_RESET << delta << std::endl;
//		std::cout << ANSI_BACK_GREEN << "X:\n" << ANSI_RESET << X << std::endl;

		if (delta.norm() < m_threshold)
		{
			break;
		}

		if (i % 80 == 0 && i > 0)
		{
			std::cout << std::endl;
		}
		std::cout << ".";
		std::cout.flush();

		if (!ros::ok())
		{
			return true;
		}
	}
	std::cout << std::endl;

	if (i == m_iterations)
	{
		ROS_WARN("No more iterations were possible because max_iterations was reached!");
	}

//	m_parameters(4) = -m_parameters(4);

	ROS_INFO("Stopped after %d iterations!", i);
	ROS_INFO(" ");

	ROS_INFO("============================================================");
	printParameters(m_parameters);
	Eigen::Affine3d t;
	t.matrix() = buildTransformation(m_parameters);

	ROS_INFO("Final matrix (camera_rgb_optical_frame to lbr_0_link/iiwa_0_link):");
	printTransformation(t);

	Eigen::Affine3d tCameraLink = transformToCameraBaseLink(t);

	if (m_direction == 2)
	{
		tCameraLink = tCameraLink.inverse();
	}

	ROS_INFO("Final matrix (camera_link to lbr_0_link/iiwa_0_link):");
	printTransformation(tCameraLink);

	ROS_INFO("============================================================");

	if (!m_saveCalibration.empty())
	{
		saveCalibration(tCameraLink);
	}

	if (m_publishMarkerArray)
	{
		Eigen::Vector3d markerPos = m_parameters.segment<3>(6);
		visualize(tCameraLink, markerPos);
	}

	return true;
}

Eigen::Affine3d CameraCalibration::transformToCameraBaseLink(Eigen::Affine3d& matrix)
{
	Eigen::Affine3d matrixOut;
	matrixOut.setIdentity();

	Eigen::Affine3d tCamera = getTransformToCameraBaseLink();

	matrixOut = (tCamera * matrix).inverse();
	return matrixOut;
}

Eigen::Matrix3d CameraCalibration::computeRotationMatrix(const double theta1,
		const double theta2,
		const double theta3)
{
	const Eigen::Vector3d theta(theta1, theta2, theta3);
	const double thetaNorm = theta.norm();
	const Eigen::Matrix3d skew = skewSymmetricMatrix(theta1, theta2, theta3);
	Eigen::Matrix3d rotation = cos(thetaNorm) * Eigen::Matrix3d::Identity()
			+ sin(thetaNorm) / thetaNorm * skew
			+ (1.0 - cos(thetaNorm)) / (thetaNorm * thetaNorm) * theta * theta.transpose();
	return rotation;
}

Eigen::Matrix3d CameraCalibration::computeRotationMatrixDerivative(const double theta1,
		const double theta2,
		const double theta3,
		const int partialDerivativeIndex)
{
	const double partialDerivativeTheta = (partialDerivativeIndex == 1 ? theta1 : (partialDerivativeIndex == 2 ? theta2 : theta3));
	const Eigen::Vector3d theta(theta1, theta2, theta3);
	const double thetaNorm = theta.norm();
	const double thetaNorm2 = thetaNorm * thetaNorm;
	const double thetaNorm3 = thetaNorm2 * thetaNorm;
	const double thetaNorm4 = thetaNorm3 * thetaNorm;
	const double sinTheta = sin(thetaNorm);
	const double cosTheta = cos(thetaNorm);
	const Eigen::Matrix3d skew = skewSymmetricMatrix(theta1, theta2, theta3);
	const Eigen::Matrix3d skewDerivative = skewSymmetricMatrixDerivative(partialDerivativeIndex);

	Eigen::Matrix3d matrix = -partialDerivativeTheta * (sinTheta / thetaNorm) * Eigen::Matrix3d::Identity()
			+ partialDerivativeTheta * ((cosTheta / (thetaNorm2)) - (sinTheta / (thetaNorm3))) * skew
			+ (sinTheta / thetaNorm) * skewDerivative
			+ partialDerivativeTheta * ((sinTheta / thetaNorm3) + 2.0 * (cosTheta - 1.0) / thetaNorm4) * (theta * theta.transpose())
			+ ((1.0 - cosTheta) / thetaNorm2) * getMultiplicationDerivative(theta1, theta2, theta3, partialDerivativeIndex);

	return matrix;
}

Eigen::Matrix3d CameraCalibration::skewSymmetricMatrix(const double theta1,
		const double theta2,
		const double theta3)
{
	Eigen::Matrix3d skew;
	skew.setZero();
	skew(0, 1) = -theta3;
	skew(0, 2) = theta2;
	skew(1, 0) = theta3;
	skew(1, 2) = -theta1;
	skew(2, 0) = -theta2;
	skew(2, 1) = theta1;

	return skew;
}

Eigen::Matrix3d CameraCalibration::skewSymmetricMatrixDerivative(const int partialDerivativeIndex)
{
	Eigen::Matrix3d skew;
	skew.setZero();

	switch (partialDerivativeIndex)
	{
		case 1:
			skew(1, 2) = -1;
			skew(2, 1) = 1;
			break;
		case 2:
			skew(0, 2) = 1;
			skew(2, 0) = -1;
			break;
		case 3:
			skew(0, 1) = -1;
			skew(1, 0) = 1;
			break;
		default:
			ROS_ERROR("partialDerivativeIndex needs to be 1, 2 or 3");
			exit(-1);
	}

	return skew;
}

Eigen::Matrix4d CameraCalibration::buildTransformation(const Eigen::VectorXd& X)
{
	Eigen::Matrix4d matrix;
	matrix.setIdentity();
	matrix.block<3, 3>(0, 0) = computeRotationMatrix((double) X(0), (double) X(1), (double) X(2));
	matrix.block<3, 1>(0, 3) = Eigen::Vector3d((double) X(3), (double) X(4), (double) X(5));
	return matrix;
}

Eigen::Vector4d CameraCalibration::computePredictedMarkerPose(const Eigen::VectorXd& X,
		const Eigen::Matrix4d& tEEf)
{
//	ROS_INFO_STREAM(buildTransformation(X));
//	ROS_INFO_STREAM((tEEf * Eigen::Vector4d((double) X(6), (double) X(7), (double) X(8), 1)).transpose());
	return buildTransformation(X) * tEEf * Eigen::Vector4d((double) X(6), (double) X(7), (double) X(8), 1);
}

Eigen::VectorXd CameraCalibration::computePredictedMarkerDistance(const Eigen::VectorXd& X)
{
	Eigen::VectorXd dists(m_robotPoses.size() * 3);
	for (size_t i = 0; i < m_robotPoses.size(); ++i)
	{
		Eigen::Vector4d pos = computePredictedMarkerPose(X, m_robotPoses[i].matrix());
		dists.segment<3>(i * 3) = pos.head(3);
	}

	return dists;
}

Eigen::VectorXd CameraCalibration::buildMeassurementVector()
{
	Eigen::VectorXd dists(m_markerPoses.size() * 3);
	for (size_t i = 0; i < m_markerPoses.size(); ++i)
	{
		Eigen::Vector4d pos = (m_markerPoses[i].matrix()).block<4, 1>(0, 3);
		dists.segment<3>(i * 3) = pos.head(3);
	}

	return dists;
}

Eigen::VectorXd CameraCalibration::computeResidual(const Eigen::VectorXd& meassurement,
		const Eigen::VectorXd& predicted)
{
	return predicted - meassurement;
}

Eigen::Matrix3d CameraCalibration::getMultiplicationDerivative(const double theta1,
		const double theta2,
		const double theta3,
		const int partialDerivativeIndex)
{
	Eigen::Matrix3d mat;
	mat.setZero();

	switch (partialDerivativeIndex)
	{
		case 1:
			mat(0, 0) = 2.0 * theta1;
			mat(0, 1) = theta2;
			mat(0, 2) = theta3;
			mat(1, 0) = theta2;
			mat(2, 0) = theta3;
			break;
		case 2:
			mat(1, 1) = 2.0 * theta2;
			mat(0, 1) = theta1;
			mat(1, 0) = theta1;
			mat(1, 2) = theta3;
			mat(2, 1) = theta3;
			break;
		case 3:
			mat(2, 2) = 2.0 * theta3;
			mat(0, 2) = theta1;
			mat(1, 2) = theta2;
			mat(2, 0) = theta1;
			mat(2, 1) = theta2;
			break;
		default:
			ROS_ERROR("partialDerivativeIndex needs to be 1, 2 or 3");
			exit(-1);
	}

	return mat;
}

Eigen::MatrixXd CameraCalibration::computeJacobian(const Eigen::VectorXd& X)
{
	Eigen::Vector4d mee((double) X(6), (double) X(7), (double) X(8), 1);
	Eigen::MatrixXd j(m_robotPoses.size() * 3, X.size());

	for (size_t i = 0; i < m_robotPoses.size(); ++i)
	{
		Eigen::Matrix4d rot;
		rot.setZero();

		Eigen::Matrix4d trans;
		trans.setZero();

		//theta1
		rot.block<3, 3>(0, 0) = computeRotationMatrixDerivative((double) X(0), (double) X(1), (double) X(2), 1);
		Eigen::Vector4d partTheta = rot * m_robotPoses[i].matrix() * mee;
		j.block<3, 1>(i * 3, 0) = partTheta.head(3);

		//theta2
		rot.block<3, 3>(0, 0) = computeRotationMatrixDerivative((double) X(0), (double) X(1), (double) X(2), 2);
		partTheta = rot * m_robotPoses[i].matrix() * mee;
		j.block<3, 1>(i * 3, 1) = partTheta.head(3);

		//theta3
		rot.block<3, 3>(0, 0) = computeRotationMatrixDerivative((double) X(0), (double) X(1), (double) X(2), 3);
		partTheta = rot * m_robotPoses[i].matrix() * mee;
		j.block<3, 1>(i * 3, 2) = partTheta.head(3);

		//tx
		trans.block<3, 1>(0, 3) = Eigen::Vector3d(1, 0, 0);
		Eigen::Vector4d partT = trans * m_robotPoses[i].matrix() * mee;
		j.block<3, 1>(i * 3, 3) = partT.head(3);

		//ty
		trans.block<3, 1>(0, 3) = Eigen::Vector3d(0, 1, 0);
		partT = trans * m_robotPoses[i].matrix() * mee;
		j.block<3, 1>(i * 3, 4) = partT.head(3);

		//ty
		trans.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 1);
		partT = trans * m_robotPoses[i].matrix() * mee;
		j.block<3, 1>(i * 3, 5) = partT.head(3);

		//mee_x
		Eigen::Matrix4d t = buildTransformation(X);
		Eigen::Vector4d partMee = t * m_robotPoses[i].matrix() * Eigen::Vector4d(1, 0, 0, 0);
		j.block<3, 1>(i * 3, 6) = partMee.head(3);

		//mee_y
		partMee = t * m_robotPoses[i].matrix() * Eigen::Vector4d(0, 1, 0, 0);
		j.block<3, 1>(i * 3, 7) = partMee.head(3);

		//mee_z
		partMee = t * m_robotPoses[i].matrix() * Eigen::Vector4d(0, 0, 1, 0);
		j.block<3, 1>(i * 3, 8) = partMee.head(3);
	}

	return j;
}

Eigen::MatrixXd CameraCalibration::pseudoInverse(const Eigen::MatrixXd& a,
		double epsilon)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
	return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()
			* svd.matrixU().adjoint();
}

void CameraCalibration::buildTestCase()
{
	Eigen::VectorXd X(9);
	X << 0.0001, 0, 0, 1, 2, 3, 0.1, 0.2, 0.3;
	Eigen::Matrix4d tRobotCamera = buildTransformation(X);

	std::vector<Eigen::Matrix4d> teefs;
	for (int i = 1; i < 100; ++i)
	{
		Eigen::Matrix4d teef;
		teef.setIdentity();
		double x = rand() % 2 == 0 ? rand() : -rand();
		double y = rand() % 2 == 0 ? rand() : -rand();
		double z = rand() % 2 == 0 ? rand() : -rand();
		teef.block<3, 1>(0, 3) = Eigen::Vector3d(x / RAND_MAX, y / RAND_MAX, z / RAND_MAX);
		teef.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::AngleAxisd(((double) rand() / RAND_MAX) * M_PI, Eigen::Vector3d::UnitX())
				* Eigen::AngleAxisd(((double) rand() / RAND_MAX) * M_PI, Eigen::Vector3d::UnitY()));
		teefs.push_back(teef);
	}

	Eigen::Matrix4d mee;
	mee.setIdentity();
	mee.block<3, 1>(0, 3) = Eigen::Vector3d((double) X(6), (double) X(7), (double) X(8));

	for (auto& it : teefs)
	{
		Eigen::Matrix4d noise = Eigen::Matrix4d::Random() / 1000.0;
		Eigen::Affine3d t1, t2;
		t1.matrix() = it;
		t2.matrix() = tRobotCamera * it * mee + noise;
		m_robotPoses.push_back(t1);
		m_markerPoses.push_back(t2);
	}
}

void CameraCalibration::printErrors(const Eigen::VectorXd& X)
{
	Eigen::Matrix4d t = buildTransformation(X);

	Eigen::Vector4d mee((double) X(6), (double) X(7), (double) X(8), 1);

	for (size_t i = 0; i < m_markerPoses.size(); ++i)
	{
		Eigen::Vector4d marker = m_markerPoses[i].matrix().col(3);
		Eigen::Vector4d markerEstimated = t * m_robotPoses[i].matrix() * mee;
		std::cout << "error " << i << ": " << (marker - markerEstimated).norm() << std::endl;
	}
}

void CameraCalibration::normalizeAngles(Eigen::VectorXd& X)
{
	static const double pi2 = 2.0 * M_PI;
	Eigen::Vector3d theta((double) X(0), (double) X(1), (double) X(2));
	Eigen::Vector3d norm2pi = theta.normalized() * pi2;

	while (theta.norm() > pi2)
	{
		theta -= norm2pi;
	}

	X(0) = theta(0);
	X(1) = theta(1);
	X(2) = theta(2);
}

Eigen::Affine3d CameraCalibration::getTransformToCameraBaseLink()
{
	tf::StampedTransform tf;
	try
	{
		m_transformListener->lookupTransform(m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera_link",
				m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera_rgb_optical_frame", ros::Time(), tf);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("TF exception: %s", ex.what());
		return Eigen::Affine3d::Identity();
	}
	Eigen::Affine3d tCamera;
	tf::transformTFToEigen(tf, tCamera);

	return tCamera;
}

void CameraCalibration::writePoses(const std::string& file)
{
	std::ofstream myfile;
	myfile.open(file.c_str());
	myfile.precision(10);

	myfile << "markers " << m_markerPoses.size() << "\n";

	for (auto& it : m_markerPoses)
	{
		myfile << it(0, 0) << " " << it(0, 1) << " " << it(0, 2) << " " << it(0, 3) << " "
				<< it(1, 0) << " " << it(1, 1) << " " << it(1, 2) << " " << it(1, 3) << " "
				<< it(2, 0) << " " << it(2, 1) << " " << it(2, 2) << " " << it(2, 3) << " "
				<< it(3, 0) << " " << it(3, 1) << " " << it(3, 2) << " " << it(3, 3) << "\n";
	}

	myfile << "robot " << m_robotPoses.size() << "\n";

	for (auto& it : m_robotPoses)
	{
		myfile << it(0, 0) << " " << it(0, 1) << " " << it(0, 2) << " " << it(0, 3) << " "
				<< it(1, 0) << " " << it(1, 1) << " " << it(1, 2) << " " << it(1, 3) << " "
				<< it(2, 0) << " " << it(2, 1) << " " << it(2, 2) << " " << it(2, 3) << " "
				<< it(3, 0) << " " << it(3, 1) << " " << it(3, 2) << " " << it(3, 3) << "\n";
	}

	myfile.close();
}

void CameraCalibration::readPoses(const std::string& file)
{
	std::ifstream myfile;
	myfile.open(file.c_str());

	int types = 0;

	while (!myfile.eof() && types < 2)
	{
		std::string type;
		int numberOfEntries;

		myfile >> type;
		myfile >> numberOfEntries;

		std::vector<Eigen::Affine3d> poses;

		for (int i = 0; i < numberOfEntries; ++i)
		{
			Eigen::Matrix4d matrix;
			myfile >> matrix(0, 0) >> matrix(0, 1) >> matrix(0, 2) >> matrix(0, 3)
					>> matrix(1, 0) >> matrix(1, 1) >> matrix(1, 2) >> matrix(1, 3)
					>> matrix(2, 0) >> matrix(2, 1) >> matrix(2, 2) >> matrix(2, 3)
					>> matrix(3, 0) >> matrix(3, 1) >> matrix(3, 2) >> matrix(3, 3);
			Eigen::Affine3d pose;
			pose.matrix() = matrix;
			poses.push_back(pose);
		}

		if (type == "markers")
		{
			m_markerPoses = poses;
			++types;
		}
		else
		{
			m_robotPoses = poses;
			++types;
		}
	}

	if (types != 2)
	{
		ROS_ERROR("File has errors!");
		exit(-1);
	}

	myfile.close();
}

void CameraCalibration::printConfig()
{
	ROS_INFO("============================================================");
	ROS_INFO("Configuration: ");
	ROS_INFO("  Read from file:         %s", BOOL_TO_STRING(m_readPosesFromFile));
	ROS_INFO("    Filename:             %s", m_posesFileNameRead.c_str());
	ROS_INFO("  Write to file:          %s", BOOL_TO_STRING(m_writePosesToFile));
	ROS_INFO("    Filename:             %s", m_posesFileNameWrite.c_str());
	ROS_INFO("  Run test case:          %s", BOOL_TO_STRING(m_testCase));
	ROS_INFO("  Listen to topics:       %s", BOOL_TO_STRING(m_listenToTopicsSeconds));
	ROS_INFO("  Save calibration:       %s", m_saveCalibration.c_str());
	ROS_INFO("  Visualize:              %s", BOOL_TO_STRING(m_publishMarkerArray));
	ROS_INFO("  Iterations:             %d", m_iterations);
	ROS_INFO("  Threshold (stop):       %e", m_threshold);
	ROS_INFO("  Marker size:            %fm", m_markerSize);
	ROS_INFO("  Initialization:");
	ROS_INFO("    theta_x:              % 6f", m_parameters(0));
	ROS_INFO("    theta_y:              % 6f", m_parameters(1));
	ROS_INFO("    theta_z:              % 6f", m_parameters(2));
	ROS_INFO("    x:                    % 6f", m_parameters(3));
	ROS_INFO("    y:                    % 6f", m_parameters(4));
	ROS_INFO("    z:                    % 6f", m_parameters(5));
	ROS_INFO("    mee_x:                % 6f", m_parameters(6));
	ROS_INFO("    mee_y:                % 6f", m_parameters(7));
	ROS_INFO("    mee_z:                % 6f", m_parameters(8));
	ROS_INFO("============================================================");
}

void CameraCalibration::printParameters(const Eigen::VectorXd& X)
{
	ROS_INFO("Parameters:");
	ROS_INFO("  theta_x: % 6f", X(0));
	ROS_INFO("  theta_y: % 6f", X(1));
	ROS_INFO("  theta_z: % 6f", X(2));
	ROS_INFO("  x:       % 6f", X(3));
	ROS_INFO("  y:       % 6f", X(4));
	ROS_INFO("  z:       % 6f", X(5));
	ROS_INFO("  mee_x:   % 6f", X(6));
	ROS_INFO("  mee_y:   % 6f", X(7));
	ROS_INFO("  mee_z:   % 6f", X(8));
}

void CameraCalibration::printTransformation(const Eigen::Affine3d& t)
{
	ROS_INFO("  % 6f  % 6f  % 6f  % 6f", t(0, 0), t(0, 1), t(0, 2), t(0, 3));
	ROS_INFO("  % 6f  % 6f  % 6f  % 6f", t(1, 0), t(1, 1), t(1, 2), t(1, 3));
	ROS_INFO("  % 6f  % 6f  % 6f  % 6f", t(2, 0), t(2, 1), t(2, 2), t(2, 3));
	ROS_INFO("  % 6f  % 6f  % 6f  % 6f", t(3, 0), t(3, 1), t(3, 2), t(3, 3));
}

void CameraCalibration::saveCalibration(const Eigen::Affine3d& t)
{
	std::string path = m_saveCalibration;
	std::string pathOld = m_saveCalibration + "_backup";

	if (boost::filesystem::exists(path))
	{
		boost::filesystem::rename(path, pathOld);
	}

	std::ofstream myfile;
	myfile.open(path.c_str());
	myfile.precision(15);

	myfile << t(0, 0) << " " << t(0, 1) << " " << t(0, 2) << " " << t(0, 3) << "\n"
			<< t(1, 0) << " " << t(1, 1) << " " << t(1, 2) << " " << t(1, 3) << "\n"
			<< t(2, 0) << " " << t(2, 1) << " " << t(2, 2) << " " << t(2, 3) << "\n"
			<< t(3, 0) << " " << t(3, 1) << " " << t(3, 2) << " " << t(3, 3) << "\n";

	myfile.close();

	ROS_INFO("Saved calibration in '%s'", path.c_str());
}

void CameraCalibration::visualize(const Eigen::Affine3d& matrix,
		const Eigen::Vector3d& markerPos)
{
	ros::NodeHandle nh("~");
	m_visPub = nh.advertise<visualization_msgs::MarkerArray>("result", 0);

	visualization_msgs::MarkerArray array;
	int id = 0;
	for (auto& it : m_markerPoses)
	{
		visualization_msgs::Marker marker;
		Eigen::Vector3d pos = it.translation();
		marker.header.frame_id = m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera_rgb_optical_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "marker_camera";
		marker.id = id++;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = pos.x();
		marker.pose.position.y = pos.y();
		marker.pose.position.z = pos.z();
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		array.markers.push_back(marker);
	}

	for (auto& it : m_robotPoses)
	{
		visualization_msgs::Marker marker;
		Eigen::Vector3d pos = it.translation();
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "eef";
		marker.id = id++;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = pos.x();
		marker.pose.position.y = pos.y();
		marker.pose.position.z = pos.z();
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		array.markers.push_back(marker);
	}

	for (auto& it : m_robotPoses)
	{
		visualization_msgs::Marker marker;
		Eigen::Vector3d pos = it * markerPos;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "marker_robot";
		marker.id = id++;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = pos.x();
		marker.pose.position.y = pos.y();
		marker.pose.position.z = pos.z();
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		array.markers.push_back(marker);
	}

	m_visPub.publish(array);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::transformEigenToTF(matrix, transform);

	ros::Rate r(10);
	while (ros::ok())
	{
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map",
				m_cameraPrefix + (m_cameraPrefix.empty() ? "" : "_") + "camera_link"));
		m_visPub.publish(array);
		ros::spinOnce();
		r.sleep();
	}
}

} /* namespace omnirob_platform_calibration */

