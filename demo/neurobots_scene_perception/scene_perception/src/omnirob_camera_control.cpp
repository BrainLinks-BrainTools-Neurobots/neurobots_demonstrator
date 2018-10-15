/*
 * omnirob_camera_control.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: kuhnerd
 */
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
message_filters::Subscriber<sensor_msgs::Image>* syncedDepthSub;
message_filters::Subscriber<sensor_msgs::Image>* syncedColorSub;
message_filters::Synchronizer<SyncPolicy>* syncronizer;

ros::Publisher pubColor, pubDepth, pubCameraInfoColor, pubCameraInfoDepth;
ros::Subscriber subCameraInfoRGB, subCameraInfoDepth;
ros::ServiceServer service;

bool sendData = true;

void rosCallbackRGBD(const sensor_msgs::Image::ConstPtr& depthImage,
		const sensor_msgs::Image::ConstPtr& rgbImage)
{
	if (!sendData)
		return;

	pubColor.publish(rgbImage);
	pubDepth.publish(depthImage);
}

void rosCallbackRGBCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& cameraInfo)
{
	if (!sendData)
		return;

	pubCameraInfoColor.publish(cameraInfo);
}

void rosCallbackDepthCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& cameraInfo)
{
	if (!sendData)
		return;

	pubCameraInfoDepth.publish(cameraInfo);
}

bool serviceCallback(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res)
{
	sendData = req.data;
	res.success = true;
	return true;
}

int main(int argc,
		char** argv)
{

	ros::init(argc, argv, "omnirob_camera_control");
	ros::NodeHandle n;

	pubColor = n.advertise<sensor_msgs::Image>("/omnirob_camera_bridge/rgb/image_rect_color", 1);
	pubDepth = n.advertise<sensor_msgs::Image>("/omnirob_camera_bridge/depth_registered/image_raw", 1);
	pubCameraInfoColor = n.advertise<sensor_msgs::CameraInfo>("/omnirob_camera_bridge/rgb/camera_info", 1);
	pubCameraInfoDepth = n.advertise<sensor_msgs::CameraInfo>("/omnirob_camera_bridge/depth_registered/camera_info", 1);

	subCameraInfoRGB = n.subscribe("/omnirob_camera/rgb/camera_info", 1, rosCallbackRGBCameraInfo);
	subCameraInfoDepth = n.subscribe("/omnirob_camera/depth_registered/camera_info", 1, rosCallbackDepthCameraInfo);

	service = n.advertiseService("/omnirob_camera_bridge/send_images", serviceCallback);

	syncedColorSub = new message_filters::Subscriber<sensor_msgs::Image>(n, "/omnirob_camera/rgb/image_rect_color", 1);
	syncedDepthSub = new message_filters::Subscriber<sensor_msgs::Image>(n, "/omnirob_camera/depth_registered/image_raw", 1);
	syncronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(2), *syncedDepthSub, *syncedColorSub);
	syncronizer->registerCallback(rosCallbackRGBD);

	ros::spin();
}

