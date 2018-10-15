/*
 * aruco_helpers.h
 *
 *  Created on: Jul 10, 2015
 *      Author: kuhnerd
 */

#ifndef KUKA_LIBRARIES_ARUCO_ARUCO_HELPERS_H_
#define KUKA_LIBRARIES_ARUCO_ARUCO_HELPERS_H_
#include <opencv2/calib3d/calib3d.hpp>
#include <marker.h>
#include <board.h>
#include <Eigen/Geometry>

namespace aruco_helpers
{

Eigen::Affine3d toEigen(const cv::Mat& tvec,
		const cv::Mat& rvec)
{
	cv::Mat rot;
	cv::Rodrigues(rvec, rot);

	Eigen::Affine3d tMarker;
	tMarker.matrix() << rot.at<float>(0, 0), rot.at<float>(0, 1), rot.at<float>(0, 2), tvec.at<float>(0, 0),
			rot.at<float>(1, 0), rot.at<float>(1, 1), rot.at<float>(1, 2), tvec.at<float>(1, 0),
			rot.at<float>(2, 0), rot.at<float>(2, 1), rot.at<float>(2, 2), tvec.at<float>(2, 0),
			0, 0, 0, 1;

	return tMarker;
}

Eigen::Affine3d toEigen(const aruco::Marker& marker)
{
	return toEigen(marker.Tvec, marker.Rvec);
}

Eigen::Affine3d toEigen(const aruco::Board& board)
{
	return toEigen(board.Tvec, board.Rvec);
}

}

#endif /* KUKA_LIBRARIES_ARUCO_ARUCO_HELPERS_H_ */
