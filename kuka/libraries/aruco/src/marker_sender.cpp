#include "aruco/aruco_detector.h"
//#include "opencv2/highgui.hpp"


void ArucoDetector::set_tf_frame(const Eigen::Affine3d& marker)
{


	tf::Vector3 translation;
	translation.setX(marker(0, 3));
	translation.setY(marker(1, 3));
	translation.setZ(marker(2, 3));


	Eigen::Matrix3d rotation;
	tf::Quaternion quaternion;
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			rotation(i, j) = marker(i,j);
		}
	}
	Eigen::Quaterniond eig_quat(rotation);
    tf::quaternionEigenToTF(eig_quat.normalized(), quaternion);
    quaternion = quaternion.normalize();

//	std::cout << "position: " << translation.getX() << " " << translation.getY() << " " << translation.getZ() << std::endl;
//	std::cout << "orientation: " << quaternion.getX() << " " << quaternion.getY() << " " << quaternion.getZ()  << " " << quaternion.getW() << std::endl;



	m_transform.setOrigin(translation);
	Eigen::Quaterniond eig_q;
	m_transform.setRotation(quaternion);
	m_br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), "/kinect2_rgb_optical_frame", "/hand_frame"));

}


void callbackCameraInfoRGB(const boost::shared_ptr<sensor_msgs::CameraInfo const> rgbCameraInfo, ArucoDetector& ad)
{
	if (ad.getCameraInfoReceivedRGB())
	{
		return;
	}

	ad.setCameraInfoReceivedRGB(true);

	aruco::CameraParameters cameraParameters;

	cameraParameters.CamSize.height = rgbCameraInfo->height;
	cameraParameters.CamSize.width = rgbCameraInfo->width;

	cameraParameters.CameraMatrix = cv::Mat(3, 3, CV_32F);
	cameraParameters.CameraMatrix.at<float>(0, 0) = rgbCameraInfo->K[0]; //fx
	cameraParameters.CameraMatrix.at<float>(1, 1) = rgbCameraInfo->K[4]; //fy
	cameraParameters.CameraMatrix.at<float>(0, 2) = rgbCameraInfo->K[2]; //cx
	cameraParameters.CameraMatrix.at<float>(1, 2) = rgbCameraInfo->K[5]; //cy

	ad.setCameraParameters(cameraParameters);
//	m_cameraParameters.Distorsion = cv::Mat(4, 1, CV_32FC1);
//	m_cameraParameters.Distorsion.at<float>(0, 0) = 0;//rgbCameraInfo->D[0];
//	m_cameraParameters.Distorsion.at<float>(1, 0) = 0;//rgbCameraInfo->D[1];
//	m_cameraParameters.Distorsion.at<float>(2, 0) = 0;//rgbCameraInfo->D[2];
//	m_cameraParameters.Distorsion.at<float>(3, 0) = 0;//rgbCameraInfo->D[3];
//
//	m_cameraParameters.resize(cv::Size(1280, 1024));

	ad.getSubscirerCameraInfoRGB().shutdown();
}
//
////in callback:
void callbackImages(const boost::shared_ptr<sensor_msgs::Image const> imageRgb, ArucoDetector& ad)
{
    if (!ad.getCameraInfoReceivedRGB())
        return;

	//convert the color image
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(imageRgb, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat colorImage = cv_ptr->image;

	aruco::MarkerDetector md = ad.getMarkerDetector();
	std::vector<aruco::Marker> markers;
	md.detect(colorImage, markers, ad.getCameraParameters(), ad.getMarkerSize());


	if(markers.size() > 0)
	{
			//for debug
//			aruco::CvDrawingUtils::draw3dAxis(colorImage, markers.at(0), ad.getCameraParameters());
//			aruco::CvDrawingUtils::draw3dCube(colorImage, markers.at(0), ad.getCameraParameters());

//			cv::imshow("test", colorImage);
//			cv::waitKey(0);

			Eigen::Affine3d tMarker = aruco_helpers::toEigen(markers.at(0));
			ad.set_tf_frame(tMarker);
	}

}


int main (int argc, char** argv){
    
	ros::init(argc, argv, "aruco_marker_sender");
	ros::NodeHandle node;

	ArucoDetector ad;
	ad.setMarkerSize(0.12);

//	cv::namedWindow("test");

	boost::function<void(const boost::shared_ptr<sensor_msgs::CameraInfo const>&)> callback1 =
		boost::bind(callbackCameraInfoRGB, _1, boost::ref(ad));
	ad.setSubscirerCameraInfoRGB(node.subscribe("/kinect2/hd/camera_info", 1, callback1));

	boost::function<void(const boost::shared_ptr<sensor_msgs::Image const>&)> callback2 =
		boost::bind(callbackImages, _1, boost::ref(ad));
	ad.setSubscirerImage(node.subscribe("/kinect2/hd/image_color", 1, callback2));
    

	ros::spin();

	return 0;
}



