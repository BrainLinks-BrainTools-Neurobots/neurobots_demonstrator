/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jun 14, 2018
 *      Author: kuhnerd
 * 	  Filename: controller_test.cpp
 */

#include <ais_log/log.h>
#include <ais_util/stop_watch.h>
#include <Eigen/Core>
#include <ais_util/time.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <string>
#include <opencv/cv.h>
#include <fstream>

struct Waypoint
{
	double t;
	Eigen::Vector3i color;
	Eigen::Vector3d pos;
	Eigen::Matrix3d orientation;
	int flag;

	inline Eigen::Quaterniond asQuaternion()
	{
		return Eigen::Quaterniond(orientation);
	}

	inline Eigen::Vector3d asRPY()
	{
		return orientation.eulerAngles(2, 1, 0);
	}

	inline Eigen::Vector3d asXYZ()
	{
		return orientation.eulerAngles(0, 1, 2);
	}

	void translate(const Eigen::Vector3d& t) {
		pos += t;
	}

	static Eigen::Matrix3d fromAxis(const Eigen::Vector3d& x,
			const Eigen::Vector3d& y,
			const Eigen::Vector3d& z)
	{
		Eigen::Matrix3d rot;
		rot.col(0) = x;
		rot.col(1) = y;
		rot.col(2) = z;
		return rot;
	}
};

//use HTML5 importer of inkscape and convert text etc. to path, remove curves with F2 or Extensions, Render, Hershels...:
//Select the Edit Path By Nodes tool (F2).
//Click on your path to select it
//Ctrl-A to select all the nodes in that path
//Click Make Selected Segments Lines (on the toolbar at the top - the icon is a straight diagonal line between two square nodes).
//simplify path
class HTML5LineImporter
{
public:
	struct Point
	{
		Eigen::Vector3d point;
		Eigen::Vector3i col;
	};
	std::vector<std::vector<Point>> m_points;

	HTML5LineImporter(const std::string& file)
	{
		std::ifstream infile(file);
		std::string line;
		std::vector<Point> points;

		boost::regex regex { "ctx\\.[a-zA-Z]+\\(([0-9.-]+), ([0-9.-]+)\\);" };
		boost::regex regexT { "ctx\\.transform\\(([0-9.-]+), ([0-9.-]+), ([0-9.-]+), ([0-9.-]+), ([0-9.-]+), ([0-9.-]+)\\);" };
		boost::regex regexC { "ctx\\.strokeStyle = 'rgb\\(([0-9]+), ([0-9]+), ([0-9]+)\\)';" };
		boost::smatch match;

		double maxX = std::numeric_limits<double>::lowest();
		double minX = std::numeric_limits<double>::max();
		double maxY = std::numeric_limits<double>::lowest();
		double minY = std::numeric_limits<double>::max();

		double x, y;

		std::vector<Eigen::Affine2d> offsets;
		std::vector<Eigen::Vector3i> colors { Eigen::Vector3i(0, 0, 0) };

		while (std::getline(infile, line))
		{
			if (boost::algorithm::contains(line, "ctx.strokeStyle") && boost::regex_search(line, match, regexC) && match.size() == 4)
			{
				colors.back() = Eigen::Vector3i(std::stod(match[1]), std::stod(match[2]), std::stod(match[3]));
			}

			if (boost::algorithm::contains(line, "ctx.save();"))
			{
				offsets.push_back(Eigen::Affine2d::Identity());
				colors.push_back(colors.back());
			}

			if (boost::algorithm::contains(line, "ctx.transform") && boost::regex_search(line, match, regexT) && match.size() == 7)
			{
				offsets.back()(0, 0) = std::stod(match[1]);
				offsets.back()(1, 0) = std::stod(match[2]);
				offsets.back()(0, 1) = std::stod(match[3]);
				offsets.back()(1, 1) = std::stod(match[4]);

				offsets.back()(0, 2) = std::stod(match[5]);
				offsets.back()(1, 2) = std::stod(match[6]);
			}

			if (boost::algorithm::contains(line, "ctx.restore();"))
			{
				offsets.pop_back();
				colors.pop_back();
			}

			Eigen::Affine2d offset = Eigen::Affine2d::Identity();
			for (auto& o : offsets)
			{
				offset = offset * o;
			}

			if (boost::algorithm::contains(line, "moveTo") || boost::algorithm::contains(line, "stroke"))
			{
				if (!points.empty())
				{
					m_points.push_back(points);
				}
				points.clear();
			}

			if (boost::regex_search(line, match, regex) && match.size() == 3)
			{
				Eigen::Vector2d pos(std::stod(match[1]), std::stod(match[2]));
				pos = offset * pos;
				double x = pos.x();
				double y = pos.y();

				//directly continue
				if (!m_points.empty() && points.empty() && x == m_points.back().back().point.x() && y == m_points.back().back().point.y())
				{
					points = m_points.back();
					m_points.pop_back();
					continue;
				}

				points.push_back( { Eigen::Vector3d(x, y, 0), colors.back() });

				if (x < minX)
					minX = x;
				if (x > maxX)
					maxX = x;
				if (y < minY)
					minY = y;
				if (y > maxY)
					maxY = y;
			}
		}

		for (auto& it : m_points)
		{
			for (auto& it2 : it)
			{
				double width2 = (maxX - minX) / 2.0;
				double height2 = (maxY - minY) / 2.0;

				it2.point.x() = width2 - (it2.point.x() - minX);
				it2.point.y() = height2 - (it2.point.y() - minY);
			}
		}
	}

	std::vector<Waypoint> getWaypoints(const Eigen::Affine3d& trans)
	{
		std::vector<Waypoint> wpts;
		const double linVel = 0.1;
		double t = 0.0;

		LOG_INFO(m_points.size() << " line segments");

		for (size_t line = 0; line < m_points.size(); ++line)
		{
			auto& it = m_points[line];

			Waypoint w { t, Eigen::Vector3i::Zero(), (trans * it.front().point) + trans.rotation() * Eigen::Vector3d(0, 0, -0.005), trans.rotation(), 0 };
			wpts.push_back(w);

			t += 0.05 / linVel;
			for (size_t i = 0; i < it.size(); ++i)
			{
				double len = 0;
				auto& it2 = it[i];
				if (i == it.size() - 1)
				{
					len = 0.05;
				}
				else
				{
					len = ((trans * it[i + 1].point) - (trans * it2.point)).norm();
				}
				Waypoint w { t, it2.col, trans * it2.point, trans.rotation(), i == 0 ? 0 : 1 };
				wpts.push_back(w);
				t += len / linVel;
			}

			Waypoint w2 { t, Eigen::Vector3i::Zero(), (trans * it.back().point) + trans.rotation() * Eigen::Vector3d(0, 0, -0.005), trans.rotation(), 0 };
			wpts.push_back(w2);

			if (line < m_points.size() - 1)
			{
				t += ((trans * m_points[line + 1].front().point) - (trans * it.back().point)).norm() / linVel;
			}
		}

		return wpts;
	}
};

class KUKAIIWA14820
{
public:
	Eigen::Isometry3d m_eeffk;
	Eigen::Isometry3d m_t1, m_t2, m_t3, m_t4, m_t5, m_t6, m_t7;
	Eigen::Isometry3d m_tr1, m_tr2, m_tr3, m_tr4, m_tr5, m_tr6, m_tr7;
	tf::TransformBroadcaster m_tf;

	KUKAIIWA14820() :
					m_t1(Eigen::Isometry3d::Identity()),
					m_t2(Eigen::Isometry3d::Identity()),
					m_t3(Eigen::Isometry3d::Identity()),
					m_t4(Eigen::Isometry3d::Identity()),
					m_t5(Eigen::Isometry3d::Identity()),
					m_t6(Eigen::Isometry3d::Identity()),
					m_t7(Eigen::Isometry3d::Identity())
	{
//		m_t1(2, 3) = 0.36;

//		m_t2(2, 1) = -1.0;
//		m_t2(2, 2) = 0.0;
//		m_t2(1, 1) = 0.0;
//		m_t2(2, 3) = 0.36;
//
//		m_t3(2, 1) = 1.0;
//		m_t3(2, 2) = 0.0;
//		m_t3(1, 1) = 0.0;
//		m_t3(2, 3) = 0.0;
//
//		m_t4(2, 1) = 1.0;
//		m_t4(2, 2) = 0.0;
//		m_t4(1, 1) = 0.0;
//		m_t4(2, 3) = 0.42;
//
//		m_t5(2, 1) = -1.0;
//		m_t5(2, 2) = 0.0;
//		m_t5(1, 1) = 0.0;
//
//		m_t6(2, 1) = -1.0;
//		m_t6(2, 2) = 0.0;
//		m_t6(1, 1) = 0.0;
//		m_t6(2, 3) = 0.4;
//
//		m_t7(2, 1) = 1.0;
//		m_t7(2, 2) = 0.0;
//		m_t7(1, 1) = 0.0;
		//m_t7(2, 3) = 0.126 + 0.03 + 0.12; //0.126, flange, 0.12 pen

		m_t1(1, 1) = 0;
		m_t1(2, 2) = 0;
		m_t1(2, 1) = 1;
		m_t1(2, 3) = 0.36;

		m_t2(2, 1) = -1.0;
		m_t2(2, 2) = 0.0;
		m_t2(1, 1) = 0.0;

		m_t3(2, 1) = -1.0;
		m_t3(2, 2) = 0.0;
		m_t3(1, 1) = 0.0;
		m_t3(2, 3) = 0.42;

		m_t4(2, 1) = 1.0;
		m_t4(2, 2) = 0.0;
		m_t4(1, 1) = 0.0;

		m_t5(2, 1) = 1.0;
		m_t5(2, 2) = 0.0;
		m_t5(1, 1) = 0.0;
		m_t5(2, 3) = 0.4;

		m_t6(2, 1) = -1.0;
		m_t6(2, 2) = 0.0;
		m_t6(1, 1) = 0.0;

		m_t7(2, 3) = 0.126;
		//m_t7(2, 3) = 0.126;
	}

//	void t1Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 1) = -sin(q);
//		t(1, 0) = sin(q);
//		t(1, 1) = cos(q);
//	}
//
//	void t2Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 2) = -sin(q);
//		t(1, 0) = sin(q);
//		t(1, 2) = cos(q);
//	}
//
//	void t3Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 2) = sin(q);
//		t(1, 0) = sin(q);
//		t(1, 2) = -cos(q);
//	}
//
//	void t4Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 2) = sin(q);
//		t(1, 0) = sin(q);
//		t(1, 2) = -cos(q);
//	}
//
//	void t5Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 2) = -sin(q);
//		t(1, 0) = sin(q);
//		t(1, 2) = cos(q);
//	}
//
//	void t6Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 2) = -sin(q);
//		t(1, 0) = sin(q);
//		t(1, 2) = cos(q);
//	}
//
//	void t7Matrix(const double& q,
//			Eigen::Isometry3d& t)
//	{
//		t(0, 0) = cos(q);
//		t(0, 2) = sin(q);
//		t(1, 0) = sin(q);
//		t(1, 2) = -cos(q);
//	}

	void t1Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 2) = sin(q);
		t(1, 0) = sin(q);
		t(1, 2) = -cos(q);
	}

	void t2Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 2) = -sin(q);
		t(1, 0) = sin(q);
		t(1, 2) = cos(q);
	}

	void t3Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 2) = -sin(q);
		t(1, 0) = sin(q);
		t(1, 2) = cos(q);
	}

	void t4Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 2) = sin(q);
		t(1, 0) = sin(q);
		t(1, 2) = -cos(q);
	}

	void t5Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 2) = sin(q);
		t(1, 0) = sin(q);
		t(1, 2) = -cos(q);
	}

	void t6Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 2) = -sin(q);
		t(1, 0) = sin(q);
		t(1, 2) = cos(q);
	}

	void t7Matrix(const double& q,
			Eigen::Isometry3d& t)
	{
		t(0, 0) = cos(q);
		t(0, 1) = -sin(q);
		t(1, 0) = sin(q);
		t(1, 1) = cos(q);
	}

	static std::vector<double> qmin, qmax;

	void updateFK(const Eigen::Matrix<double, 7, 1>& q)
	{
		t1Matrix(q[0], m_t1);
		t2Matrix(q[1], m_t2);
		t3Matrix(q[2], m_t3);
		t4Matrix(q[3], m_t4);
		t5Matrix(q[4], m_t5);
		t6Matrix(q[5], m_t6);
		t7Matrix(q[6], m_t7);
		m_tr1 = m_t1;
		m_tr2 = m_tr1 * m_t2;
		m_tr3 = m_tr2 * m_t3;
		m_tr4 = m_tr3 * m_t4;
		m_tr5 = m_tr4 * m_t5;
		m_tr6 = m_tr5 * m_t6;
		m_tr7 = m_tr6 * m_t7;
	}

	void publishFrames()
	{
		tf::Transform transform;
		tf::poseEigenToTF(m_tr1, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_1_link_KIN"));
		tf::poseEigenToTF(m_tr2, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_2_link_KIN"));
		tf::poseEigenToTF(m_tr3, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_3_link_KIN"));
		tf::poseEigenToTF(m_tr4, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_4_link_KIN"));
		tf::poseEigenToTF(m_tr5, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_5_link_KIN"));
		tf::poseEigenToTF(m_tr6, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_6_link_KIN"));
		tf::poseEigenToTF(m_tr7, transform);
		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_7_link_KIN"));
//
//		tf::poseEigenToTF(m_t1, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_0_link", "/iiwa/iiwa_1_link_KIN_REL"));
//		tf::poseEigenToTF(m_t2, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_1_link", "/iiwa/iiwa_2_link_KIN_REL"));
//		tf::poseEigenToTF(m_t3, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_2_link", "/iiwa/iiwa_3_link_KIN_REL"));
//		tf::poseEigenToTF(m_t4, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_3_link", "/iiwa/iiwa_4_link_KIN_REL"));
//		tf::poseEigenToTF(m_t5, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_4_link", "/iiwa/iiwa_5_link_KIN_REL"));
//		tf::poseEigenToTF(m_t6, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_5_link", "/iiwa/iiwa_6_link_KIN_REL"));
//		tf::poseEigenToTF(m_t7, transform);
//		m_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/iiwa/iiwa_6_link", "/iiwa/iiwa_7_link_KIN_REL"));
	}

	inline Eigen::Isometry3d fk()
	{
		return m_tr7;
	}

	inline Eigen::Vector3d fkPos()
	{
		return Eigen::Vector3d(m_tr7(0, 3), m_tr7(1, 3), m_tr7(2, 3));
	}

	inline Eigen::Matrix3d fkOrientationM()
	{
		return m_tr7.rotation();
	}

	inline Eigen::Quaterniond fkOrientationQ()
	{
		const Eigen::Matrix3d rot = fkOrientationM();
		return Eigen::Quaterniond(rot);
	}

	inline Eigen::Vector3d fkOrientationRPY()
	{
		const Eigen::Matrix3d rot = fkOrientationM();
		return rot.eulerAngles(2, 1, 0);
	}

	inline Eigen::Vector3d fkOrientationXYZ()
	{
		const Eigen::Matrix3d rot = fkOrientationM();
		return rot.eulerAngles(0, 1, 2);
	}

	Eigen::Matrix<double, 6, 7> geometricJacobian()
	{
		Eigen::Matrix<double, 6, 7> jac;
		jac.setZero();

		Eigen::Vector3d z1 = Eigen::Vector3d::UnitZ();
		const Eigen::Vector3d& z2 = m_tr1.rotation().col(2);
		const Eigen::Vector3d& z3 = m_tr2.rotation().col(2);
		const Eigen::Vector3d& z4 = m_tr3.rotation().col(2);
		const Eigen::Vector3d& z5 = m_tr4.rotation().col(2);
		const Eigen::Vector3d& z6 = m_tr5.rotation().col(2);
		const Eigen::Vector3d& z7 = m_tr6.rotation().col(2);

		Eigen::Vector3d p17 = m_tr7.translation() - m_tr1.translation();
		Eigen::Vector3d p27 = m_tr7.translation() - m_tr2.translation();
		Eigen::Vector3d p37 = m_tr7.translation() - m_tr3.translation();
		Eigen::Vector3d p47 = m_tr7.translation() - m_tr4.translation();
		Eigen::Vector3d p57 = m_tr7.translation() - m_tr5.translation();
		Eigen::Vector3d p67 = m_tr7.translation() - m_tr6.translation();
		Eigen::Vector3d p77 = Eigen::Vector3d::Zero();

		jac.block<3, 1>(0, 0) = z1.cross(p17);
		jac.block<3, 1>(0, 1) = z2.cross(p27);
		jac.block<3, 1>(0, 2) = z3.cross(p37);
		jac.block<3, 1>(0, 3) = z4.cross(p47);
		jac.block<3, 1>(0, 4) = z5.cross(p57);
		jac.block<3, 1>(0, 5) = z6.cross(p67);
		jac.block<3, 1>(0, 6) = z1.cross(p77);

		jac.block<3, 1>(3, 0) = z1;
		jac.block<3, 1>(3, 1) = z2;
		jac.block<3, 1>(3, 2) = z3;
		jac.block<3, 1>(3, 3) = z4;
		jac.block<3, 1>(3, 4) = z5;
		jac.block<3, 1>(3, 5) = z6;
		jac.block<3, 1>(3, 6) = z7;

		return jac;
	}

	//Euler: ZY'X'' convention, (roll pitch yaw)
	Eigen::Matrix<double, 6, 7> analyticJacobianRPY()
	{
		Eigen::Matrix<double, 6, 7> jac = geometricJacobian();
		Eigen::Vector3d rpy = fkOrientationRPY();

		//convert angular velocity part to RPY time derivatives
		Eigen::Matrix3d rotation;
		rotation << 0, -sin(rpy.z()), cos(rpy.y()) * cos(rpy.z()),
				0, cos(rpy.z()), cos(rpy.y()) * sin(rpy.z()),
				1, 0, -sin(rpy.y());

		Eigen::Matrix<double, 6, 6> t;
		t.setIdentity();
		t.block<3, 3>(3, 3) = rotation.inverse();

		return t * jac;
	}

	//xyz
	Eigen::Matrix<double, 6, 7> analyticJacobianXYZ()
	{
		Eigen::Matrix<double, 6, 7> jac = geometricJacobian();
		Eigen::Vector3d xyz = fkOrientationXYZ();

		//convert angular velocity part to RPY time derivatives
		Eigen::Matrix3d rotation;
		rotation << 1, 0, sin(xyz.y()),
				0, cos(xyz.x()), -sin(xyz.x()) * cos(xyz.y()),
				0, sin(xyz.x()), cos(xyz.x()) * cos(xyz.y());

		Eigen::Matrix<double, 6, 6> t;
		t.setIdentity();
		t.block<3, 3>(3, 3) = rotation.inverse();

		return t * jac;
	}

	Eigen::Matrix<double, 3, 7> geometricJacobianPosition()
	{
		Eigen::Matrix<double, 3, 7> jac;
		jac.setZero();

		Eigen::Vector3d z1 = Eigen::Vector3d::UnitZ();
		const Eigen::Vector3d& z2 = m_tr1.rotation().col(2);
		const Eigen::Vector3d& z3 = m_tr2.rotation().col(2);
		const Eigen::Vector3d& z4 = m_tr3.rotation().col(2);
		const Eigen::Vector3d& z5 = m_tr4.rotation().col(2);
		const Eigen::Vector3d& z6 = m_tr5.rotation().col(2);
		const Eigen::Vector3d& z7 = m_tr6.rotation().col(2);

		Eigen::Vector3d p17 = m_tr7.translation() - m_tr1.translation();
		Eigen::Vector3d p27 = m_tr7.translation() - m_tr2.translation();
		Eigen::Vector3d p37 = m_tr7.translation() - m_tr3.translation();
		Eigen::Vector3d p47 = m_tr7.translation() - m_tr4.translation();
		Eigen::Vector3d p57 = m_tr7.translation() - m_tr5.translation();
		Eigen::Vector3d p67 = m_tr7.translation() - m_tr6.translation();
		Eigen::Vector3d p77 = Eigen::Vector3d::Zero();

		jac.block<3, 1>(0, 0) = z1.cross(p17);
		jac.block<3, 1>(0, 1) = z2.cross(p27);
		jac.block<3, 1>(0, 2) = z3.cross(p37);
		jac.block<3, 1>(0, 3) = z4.cross(p47);
		jac.block<3, 1>(0, 4) = z5.cross(p57);
		jac.block<3, 1>(0, 5) = z6.cross(p67);
		jac.block<3, 1>(0, 6) = z1.cross(p77);

		return jac;
	}

	//https://math.stackexchange.com/questions/2282938/converting-from-quaternion-to-angular-velocity-then-back-to-quaternion
	//http://web.cs.iastate.edu/~cs577/handouts/quaternion.pdf
	static Eigen::Vector3d quaternionDiffToAngularVelocity(const Eigen::Quaterniond& oldQ,
			const Eigen::Quaterniond& newQ,
			double dt)
	{
//		Eigen::Quaterniond q = newQ * oldQ.conjugate(); //from old to new
		Eigen::Matrix<double, 3, 4> H;
		double a0 = oldQ.w();
		double a1 = oldQ.x();
		double a2 = oldQ.y();
		double a3 = oldQ.z();
		H << -a1, a0, -a3, a2, -a2, a3, a0, -a1, -a3, -a2, a1, a0;
//		LOG_INFO("c1: " << (2.0 * H * ((Eigen::Vector4d(newQ.w(), newQ.x(), newQ.y(), newQ.z()) - Eigen::Vector4d(oldQ.w(), oldQ.x(), oldQ.y(), oldQ.z()))/dt)).transpose());

//		double len = q.vec().norm();
//		double angle = 2.0 * atan2(len, q.w());
//
//		Eigen::Vector3d axis = q.vec();
//
//		if (len > 0)
//		{
//			axis /= len;
//		}

//		LOG_INFO("c2: " << ((axis * (angle / dt)).transpose()));
//		return axis * (angle / dt);
		return (2.0 * H * ((Eigen::Vector4d(newQ.w(), newQ.x(), newQ.y(), newQ.z()) - Eigen::Vector4d(oldQ.w(), oldQ.x(), oldQ.y(), oldQ.z())) / dt));
	}

	static double angleDiff(const double& ang1,
			const double&ang2)
	{
		double arg;
		static const double pi2 = M_PI + M_PI;

		arg = fmod(ang2 - ang1, pi2);
		if (arg < 0)
			arg = arg + pi2;
		if (arg > M_PI)
			arg = arg - pi2;

		return -arg;
	}

	static Eigen::Vector3d angleDiff(const Eigen::Vector3d& euler1,
			const Eigen::Vector3d& euler2)
	{
		Eigen::Vector3d diff;
		diff.x() = angleDiff(euler1.x(), euler2.x());
		diff.y() = angleDiff(euler1.y(), euler2.y());
		diff.z() = angleDiff(euler1.z(), euler2.z());
		return diff;
	}

};

class JacobianInvertors
{
private:
	template<int M, int N>
	static void svd(const Eigen::Matrix<double, M, N>& jac,
			Eigen::VectorXd& singular,
			Eigen::Matrix<double, M, M>& u,
			Eigen::Matrix<double, N, N>& v)
	{
		Eigen::JacobiSVD<Eigen::Matrix<double, M, N>> svd(jac, Eigen::ComputeFullU | Eigen::ComputeFullV);
		singular = svd.singularValues();
		u = svd.matrixU();
		v = svd.matrixV();
	}

	//Maciejewski et al 1988
	template<int M>
	static double dampingMaciejewski(const Eigen::Matrix<double, M, 1>& xvel,
			const double singularValueMin,
			const double qDotMax)
	{
		const double d = xvel.norm() / qDotMax;
		const double d2 = d / 2.0;
		LOG_INFO(d << " " << xvel.norm() << " " << singularValueMin);
		if (singularValueMin < d2)
		{
			return d2;
		}
		else if (singularValueMin < d)
		{
			return sqrt(singularValueMin * (d - singularValueMin));
		}
		else
		{
			return 0;
		}
	}

	//Caccavale et al 1988
	static double dampingCaccavale(const double singularValueMin,
			const double threshold,
			const double dampingFactor)
	{
		if (singularValueMin >= threshold)
		{
			return 0;
		}
		else
		{
			const double s = singularValueMin / threshold;
			return sqrt(1.0 - s * s) * dampingFactor;
		}
	}

public:
	//1.34719s @ 10000 runs
	template<int M, int N>
	static Eigen::Matrix<double, N, M> transposeInv(const Eigen::Matrix<double, M, N>& jac)
	{
		return jac.transpose();
	}

	//4.1742s @ 10000 runs
	template<int M, int N>
	static Eigen::Matrix<double, N, M> pseudoInv(const Eigen::Matrix<double, M, N>& jac)
	{
		const auto& jt = jac.transpose();
		return jt * (jac * jt).inverse();
	}

	//8.50947s @ 10000i
	template<int M, int N>
	static Eigen::Matrix<double, N, M> svdInv(const Eigen::Matrix<double, M, N>& jac)
	{
		Eigen::VectorXd singular;
		Eigen::Matrix<double, M, M> u;
		Eigen::Matrix<double, N, N> v;

		svd(jac, singular, u, v);

		Eigen::Matrix<double, N, M> sigmaInv;
		sigmaInv.setZero();
		for (int i = 0; i < M; ++i)
		{
			const auto& s = singular[i];
			if (s > 0)
				sigmaInv(i, i) = 1.0 / s;
		}

		return v * sigmaInv * u.transpose();
	}

	//8.52448s @ 10000i
	template<int M, int N>
	static Eigen::Matrix<double, N, M> dlsInv(const Eigen::Matrix<double, M, N>& jac,
			const Eigen::Matrix<double, M, 1>& xvel)
	{
		static std::ofstream file("/tmp/lambda.txt");
//		Eigen::VectorXd singular;
//		Eigen::Matrix<double, M, M> u;
//		Eigen::Matrix<double, N, N> v;
//
//		svd(jac, singular, u, v);
//
//		Eigen::Matrix<double, N, M> jinv;
//		jinv.setZero();
//		for (int i = 0; i < M; ++i)
//		{
//			const auto& s = singular[i];
//			jinv += 1.0 / (s) * (v.col(i) * u.col(i).transpose());
//		}

		Eigen::VectorXd singular;
		Eigen::Matrix<double, M, M> u;
		Eigen::Matrix<double, N, N> v;

		svd(jac, singular, u, v);

		Eigen::Matrix<double, N, M> jinv;
		jinv.setZero();
//		double lambda = dampingMaciejewski(xvel, singular(M - 1), 0.6);
		double lambda = dampingCaccavale(singular(M - 1), 0.04, 0.5);
		file << lambda << " " << singular(M - 1) << "\n";
//		LOG_INFO(singular(M-1) << " " << M << " " << singular(5) << " " << singular.transpose());
		for (int i = 0; i < M; ++i)
		{
			const auto& s = singular[i];
			jinv += (s / (s * s + lambda * lambda)) * (v.col(i) * u.col(i).transpose());
		}

		return jinv;

//		return v * sigmaInv * u.transpose();
	}
};

class PathVisualizer
{
	ros::Publisher m_pub;
	ros::NodeHandle m_n;

	visualization_msgs::Marker m_marker;

public:
	PathVisualizer(const std::string& topic,
			const std::string& frame,
			const std::string& ns)
	{
		m_pub = m_n.advertise<visualization_msgs::Marker>(topic, 1);

		m_marker.header.frame_id = frame;
		m_marker.header.stamp = ros::Time();
		m_marker.ns = ns;
		m_marker.id = 0;
		m_marker.type = visualization_msgs::Marker::LINE_STRIP;
		m_marker.action = visualization_msgs::Marker::ADD;
		m_marker.pose.position.x = 0;
		m_marker.pose.position.y = 0;
		m_marker.pose.position.z = 0;
		m_marker.pose.orientation.x = 0.0;
		m_marker.pose.orientation.y = 0.0;
		m_marker.pose.orientation.z = 0.0;
		m_marker.pose.orientation.w = 1.0;
		m_marker.scale.x = 0.002;
		m_marker.scale.y = 0.0;
		m_marker.scale.z = 0.0;
		m_marker.color.a = 1.0; // Don't forget to set the alpha!
		m_marker.color.r = 0.0;
		m_marker.color.g = 1.0;
		m_marker.color.b = 0.0;
	}

	void send(const std::list<geometry_msgs::Point>& positions,
			ros::Time t = ros::Time::now())
	{
		m_marker.points.resize(positions.size());
		m_marker.header.stamp = t;
		std::copy(positions.begin(), positions.end(), m_marker.points.begin());
		m_pub.publish(m_marker);
	}

	void send(const std::vector<Waypoint>& positions,
			ros::Time t = ros::Time::now())
	{
		if (m_pub.getNumSubscribers() == 0)
			return;

		m_marker.points.resize(positions.size());
		m_marker.header.stamp = t;
		for (size_t i = 0; i < positions.size(); ++i)
		{
			geometry_msgs::Point p;
			p.x = positions[i].pos.x();
			p.y = positions[i].pos.y();
			p.z = positions[i].pos.z();
			m_marker.points[i] = p;
		}
		m_pub.publish(m_marker);
	}
};

class MultiPathVisualizer
{
	ros::Publisher m_pub;
	ros::NodeHandle m_n;

	visualization_msgs::Marker m_marker;

	int m_id;

public:
	MultiPathVisualizer(const std::string& topic,
			const std::string& frame,
			const std::string& ns,
			const Eigen::Vector4f& color) :
					m_id(0)
	{
		m_pub = m_n.advertise<visualization_msgs::Marker>(topic, 1);

		m_marker.header.frame_id = frame;
		m_marker.header.stamp = ros::Time();
		m_marker.ns = ns;
		m_marker.id = 0;
		m_marker.type = visualization_msgs::Marker::LINE_LIST;
		m_marker.action = visualization_msgs::Marker::ADD;
		m_marker.pose.position.x = 0;
		m_marker.pose.position.y = 0;
		m_marker.pose.position.z = 0;
		m_marker.pose.orientation.x = 0.0;
		m_marker.pose.orientation.y = 0.0;
		m_marker.pose.orientation.z = 0.0;
		m_marker.pose.orientation.w = 1.0;
		m_marker.scale.x = 0.002;
		m_marker.scale.y = 0.0;
		m_marker.scale.z = 0.0;
		m_marker.color.a = color.w();
		m_marker.color.r = color.x();
		m_marker.color.g = color.y();
		m_marker.color.b = color.z();
	}

	void send(const std::list<geometry_msgs::Point>& positions,
			ros::Time t = ros::Time::now())
	{
		m_marker.id = m_id;
		m_marker.points.resize(positions.size());
		m_marker.header.stamp = t;
		std::copy(positions.begin(), positions.end(), m_marker.points.begin());
		m_pub.publish(m_marker);
	}

	void incId()
	{
		m_id++;
	}

	void zeroId()
	{
		m_id = 0;
	}
};

class ToolMarker
{
	ros::Publisher m_pub;
	ros::NodeHandle m_n;

	visualization_msgs::Marker m_marker;

public:
	ToolMarker(const std::string& topic,
			const std::string& frame,
			const std::string& ns,
			const Eigen::Vector4f& color)
	{
		m_pub = m_n.advertise<visualization_msgs::Marker>(topic, 1);

		m_marker.header.frame_id = frame;
		m_marker.header.stamp = ros::Time();
		m_marker.ns = ns;
		m_marker.id = 0;
		m_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		m_marker.action = visualization_msgs::Marker::ADD;
		m_marker.pose.position.x = 0;
		m_marker.pose.position.y = 0;
		m_marker.pose.position.z = 0;
		m_marker.pose.orientation.x = 0.0;
		m_marker.pose.orientation.y = 0.0;
		m_marker.pose.orientation.z = 0.0;
		m_marker.pose.orientation.w = 1.0;
		m_marker.scale.x = 1.0;
		m_marker.scale.y = 1.0;
		m_marker.scale.z = 1.0;
		m_marker.color.a = color.w();
		m_marker.color.r = color.x();
		m_marker.color.g = color.y();
		m_marker.color.b = color.z();
		m_marker.mesh_resource = "package://prm_planner_controller/models/pen.dae";
		m_marker.mesh_use_embedded_materials = true;
		m_marker.frame_locked = true;
	}

	void send()
	{
		m_marker.id = 0;
		m_pub.publish(m_marker);
	}
};

class BoardExporter
{
	ros::Publisher m_pub;
	ros::NodeHandle m_n;

	tf::TransformBroadcaster m_tb;
	ros::Publisher m_pubImage;

	cv::Mat m_image;

public:
	BoardExporter(const std::string& topic,
			const std::string& frame,
			const std::string& ns) :
					m_image(1000, 2000, CV_8UC3)
	{
		m_pubImage = m_n.advertise<sensor_msgs::Image>(topic, 1);
		m_image = cv::Scalar(255, 255, 255);
	}

	void draw(const Eigen::Vector3d& transformedPos,
			const Eigen::Vector3i& color)
	{
		if (fabs(transformedPos.z()) < 0.001)
		{
			const double pixelSize = 0.001;
			const double widthM = pixelSize * m_image.cols;
			const double heightM = pixelSize * m_image.rows;
			const double centerXM = widthM / 2.0;
			const double centerYM = heightM / 2.0;

			int x = (transformedPos.x() + centerXM) / pixelSize;
			int y = (transformedPos.y() + centerYM) / pixelSize;
			if (y > 0 && y < m_image.rows && x > 0 && x < m_image.cols)
			{
				m_image.at<cv::Vec3b>(y, x) = cv::Vec3b(color.x(), color.y(), color.z());
			}
		}
	}

	void publishBoard(const Eigen::Affine3d& t)
	{
		tf::Transform tr;
		Eigen::Quaterniond q((Eigen::Matrix3d(t.rotation())));
		tr.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
		tr.setOrigin(tf::Vector3(t(0, 3), t(1, 3), t(2, 3)));
		m_tb.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "/iiwa/iiwa_0_link", "board"));

		m_pubImage.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", m_image));
	}
};

std::vector<double> KUKAIIWA14820::qmin( { -2.96705972839, -2.09439510239, -2.96705972839, -2.09439510239, -2.96705972839, -2.09439510239, -3.05432619099 });
std::vector<double> KUKAIIWA14820::qmax( { 2.96705972839, 2.09439510239, 2.96705972839, 2.09439510239, 2.96705972839, 2.09439510239, 3.05432619099 });

std::vector<Waypoint> tSquare
{
		{ 2, Eigen::Vector3i::Zero(), Eigen::Vector3d(0.3, 0, 1.0), Waypoint::fromAxis( { 0, 0, 1 }, { 0, -1, 0 }, { 1, 0, 0 }), 1 },
		{ 4, Eigen::Vector3i::Zero(), Eigen::Vector3d(0.3, 0.2, 1.0), Waypoint::fromAxis( { 0, 0, 1 }, { 0, -1, 0 }, { 1, 0, 0 }), 1 },
		{ 6, Eigen::Vector3i::Zero(), Eigen::Vector3d(0.3, 0.2, 0.8), Waypoint::fromAxis( { 0, 0, 1 }, { 0, -1, 0 }, { 1, 0, 0 }), 1 },
		{ 8, Eigen::Vector3i::Zero(), Eigen::Vector3d(0.3, 0, 0.8), Waypoint::fromAxis( { 0, 0, 1 }, { 0, -1, 0 }, { 1, 0, 0 }), 1 },
		{ 12, Eigen::Vector3i::Zero(), Eigen::Vector3d(0.3, 0, 1.0), Waypoint::fromAxis( { 0, 0, 1 }, { 0, -1, 0 }, { 1, 0, 0 }), 1 }
};

std::vector<Waypoint> tPoint
{
		{ 2, Eigen::Vector3i::Zero(), Eigen::Vector3d(0, 0, 0.8), Waypoint::fromAxis( { 0, 0, 1 }, { 0, -1, 0 }, { 1, 0, 0 }), 1 },
};

std::vector<Waypoint> tPoint2
{
		{ 2, Eigen::Vector3i::Zero(), Eigen::Vector3d(0, 0, 0.8), Waypoint::fromAxis( { 0, 0, 1 }, { 0, 1, 0 }, { -1, 0, 0 }), 1 },
};

std::vector<Waypoint> mytrajectory
{
		{ 5, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.641269, 0.181471, 0.530171), Waypoint::fromAxis( { 0.995103, -0.0886936, 0.0436386 }, { -0.0468033, -0.0339191, 0.998328 }, { -0.0870651, -0.995481, -0.0379041 }), 1 },
		{ 5.04, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.643857, 0.185369, 0.539075), Waypoint::fromAxis( { 0.994469, -0.0949424, 0.0449045 }, { -0.0479715, -0.0302718, 0.99839 }, { -0.0934302, -0.995022, -0.0346589 }), 1 },
		{ 5.1, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.647464, 0.191231, 0.551746), Waypoint::fromAxis( { 0.993458, -0.104146, 0.046843 }, { -0.0496723, -0.0247395, 0.998459 }, { -0.102827, -0.994254, -0.0297509 }), 1 },
		{ 5.16, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.650781, 0.197082, 0.56364), Waypoint::fromAxis( { 0.992377, -0.113149, 0.048832 }, { -0.0513151, -0.0191367, 0.998499 }, { -0.112045, -0.993394, -0.0247971 }), 1 },
		{ 5.22, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.653847, 0.202894, 0.574804), Waypoint::fromAxis( { 0.991231, -0.121952, 0.0508739 }, { -0.0529043, -0.013467, 0.998509 }, { -0.121085, -0.992445, -0.0198007 }), 1 },
		{ 5.28, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.656697, 0.208639, 0.585286), Waypoint::fromAxis( { 0.990025, -0.130557, 0.0529711 }, { -0.0544437, -0.00773362, 0.998487 }, { -0.12995, -0.991411, -0.0147645 }), 1 },
		{ 5.34, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.659358, 0.214296, 0.595131), Waypoint::fromAxis( { 0.988762, -0.138967, 0.0551255 }, { -0.0559373, -0.00193988, 0.998432 }, { -0.138642, -0.990295, -0.0096915 }), 1 },
		{ 5.4, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.661854, 0.219845, 0.60438), Waypoint::fromAxis( { 0.987446, -0.147183, 0.0573386 }, { -0.0573885, 0.00391108, 0.998344 }, { -0.147163, -0.989102, -0.00458461 }), 1 },
		{ 5.46, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.664207, 0.225269, 0.613074), Waypoint::fromAxis( { 0.986082, -0.155207, 0.0596118 }, { -0.0588006, 0.00981624, 0.998221 }, { -0.155516, -0.987833, 0.000553336 }), 1 },
		{ 5.54, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.667147, 0.232284, 0.623867), Waypoint::fromAxis( { 0.984193, -0.165612, 0.0627379 }, { -0.0606281, 0.017769, 0.998002 }, { -0.166396, -0.986031, 0.00744741 }), 1 },
		{ 5.62, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.669892, 0.239026, 0.633824), Waypoint::fromAxis( { 0.982233, -0.175685, 0.0659742 }, { -0.0623985, 0.0258062, 0.997718 }, { -0.176986, -0.984108, 0.0143852 }), 1 },
		{ 5.7, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.672468, 0.245478, 0.643021), Waypoint::fromAxis( { 0.980209, -0.18543, 0.0693217 }, { -0.064118, 0.0339212, 0.997366 }, { -0.187293, -0.982072, 0.0213605 }), 1 },
		{ 5.78, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.674895, 0.251627, 0.651527), Waypoint::fromAxis( { 0.978128, -0.194853, 0.0727806 }, { -0.0657923, 0.0421079, 0.996944 }, { -0.197323, -0.979928, 0.0283671 }), 1 },
		{ 5.86, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.67719, 0.257464, 0.659408), Waypoint::fromAxis( { 0.975998, -0.20396, 0.0763503 }, { -0.0674264, 0.0503603, 0.996452 }, { -0.207081, -0.977683, 0.0353992 }), 1 },
		{ 5.96, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.67989, 0.264321, 0.668468), Waypoint::fromAxis( { 0.973273, -0.214904, 0.080967 }, { -0.0694197, 0.060759, 0.995736 }, { -0.218907, -0.974743, 0.0442166 }), 1 },
		{ 6.06, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.682422, 0.270691, 0.676741), Waypoint::fromAxis( { 0.970492, -0.22537, 0.0857523 }, { -0.0713657, 0.0712402, 0.994903 }, { -0.23033, -0.971665, 0.0530544 }), 1 },
		{ 6.18, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.685258, 0.277707, 0.685757), Waypoint::fromAxis( { 0.967095, -0.237312, 0.0917113 }, { -0.0736482, 0.0839113, 0.993748 }, { -0.243524, -0.967803, 0.0636726 }), 1 },
		{ 6.3, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.687894, 0.284064, 0.693906), Waypoint::fromAxis( { 0.963647, -0.248596, 0.0978981 }, { -0.0758833, 0.0966682, 0.99242 }, { -0.256175, -0.963771, 0.0742898 }), 1 },
		{ 6.44, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.690734, 0.290695, 0.702469), Waypoint::fromAxis( { 0.959582, -0.260953, 0.10539 }, { -0.0784426, 0.111637, 0.990648 }, { -0.270278, -0.958875, 0.086655 }), 1 },
		{ 6.58, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.693343, 0.296537, 0.710162), Waypoint::fromAxis( { 0.955489, -0.272463, 0.113157 }, { -0.080961, 0.126674, 0.988635 }, { -0.283701, -0.953791, 0.0989764 }), 1 },
		{ 6.74, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.696061, 0.302335, 0.718054), Waypoint::fromAxis( { 0.950802, -0.284617, 0.122346 }, { -0.0838007, 0.14391, 0.986036 }, { -0.298249, -0.947778, 0.112979 }), 1 },
		{ 6.92, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.698811, 0.307853, 0.725972), Waypoint::fromAxis( { 0.945546, -0.297059, 0.133038 }, { -0.0869588, 0.163325, 0.982732 }, { -0.313658, -0.940787, 0.1286 }), 1 },
		{ 7.14, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.701772, 0.313343, 0.734551), Waypoint::fromAxis( { 0.939172, -0.310598, 0.146576 }, { -0.0908027, 0.187038, 0.978147 }, { -0.331226, -0.931958, 0.147458 }), 1 },
		{ 7.34, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.703928, 0.317253, 0.744308), Waypoint::fromAxis( { 0.932199, -0.323398, 0.162541 }, { -0.0968016, 0.209955, 0.972907 }, { -0.348762, -0.922677, 0.164414 }), 1 },
		{ 7.54, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.704935, 0.320368, 0.754631), Waypoint::fromAxis( { 0.923301, -0.336932, 0.184369 }, { -0.107067, 0.235214, 0.966028 }, { -0.368852, -0.911675, 0.181099 }), 1 },
		{ 7.94, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.705615, 0.321611, 0.764642), Waypoint::fromAxis( { 0.907469, -0.354285, 0.225792 }, { -0.125373, 0.284585, 0.950417 }, { -0.400975, -0.890783, 0.213834 }), 1 },
		{ 8.48, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.705428, 0.314518, 0.772208), Waypoint::fromAxis( { 0.890489, -0.362457, 0.275053 }, { -0.144895, 0.347142, 0.926552 }, { -0.431318, -0.864938, 0.256608 }), 1 },
		{ 8.96, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.704774, 0.306548, 0.778544), Waypoint::fromAxis( { 0.878941, -0.360287, 0.312498 }, { -0.1581, 0.398073, 0.903627 }, { -0.449962, -0.843641, 0.292922 }), 1 },
		{ 9.5, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.704066, 0.299011, 0.785502), Waypoint::fromAxis( { 0.869004, -0.351566, 0.348186 }, { -0.169343, 0.449875, 0.87689 }, { -0.464925, -0.820984, 0.331407 }), 1 },
		{ 10.14, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.703611, 0.292252, 0.793115), Waypoint::fromAxis( { 0.860269, -0.33658, 0.38295 }, { -0.178978, 0.503949, 0.844987 }, { -0.477393, -0.795456, 0.373291 }), 1 },
		{ 10.92, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.703577, 0.286209, 0.801337), Waypoint::fromAxis( { 0.852645, -0.315576, 0.416422 }, { -0.18694, 0.559983, 0.807138 }, { -0.487903, -0.766049, 0.418473 }), 1 },
		{ 11.86, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.703967, 0.280916, 0.809828), Waypoint::fromAxis( { 0.846234, -0.290082, 0.446923 }, { -0.192961, 0.615008, 0.764546 }, { -0.496642, -0.733224, 0.464467 }), 1 },
		{ 13.06, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.704707, 0.276078, 0.818704), Waypoint::fromAxis( { 0.840638, -0.260534, 0.474816 }, { -0.197302, 0.669135, 0.716471 }, { -0.504381, -0.695975, 0.511096 }), 1 },
		{ 14.64, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.705679, 0.271668, 0.827633), Waypoint::fromAxis( { 0.835731, -0.228737, 0.499232 }, { -0.200037, 0.719847, 0.664685 }, { -0.511409, -0.655363, 0.555842 }), 1 },
		{ 17, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.706798, 0.267481, 0.836653), Waypoint::fromAxis( { 0.831172, -0.195136, 0.520649 }, { -0.201457, 0.767078, 0.609104 }, { -0.518236, -0.611158, 0.598262 }), 1 },
		{ 21.82, Eigen::Vector3i::Zero(), Eigen::Vector3d(-0.707997, 0.2634, 0.845728), Waypoint::fromAxis( { 0.826725, -0.160229, 0.539308 }, { -0.201784, 0.810366, 0.550082 }, { -0.525176, -0.563589, 0.637618 }), 1 }
};

class Examples
{
	MultiPathVisualizer pubEEf;
	PathVisualizer pubPath;
	BoardExporter board;
	ToolMarker tool;

	ros::Publisher jsPub;
	sensor_msgs::JointState js;
	KUKAIIWA14820 kin;

	ros::NodeHandle n;

	std::list<geometry_msgs::Point> eefPoints;
	Eigen::Matrix<double, 7, 1> q;
	Eigen::Matrix<double, 6, 1> x;
	Eigen::Matrix<double, 6, 1> error;
	Eigen::Matrix<double, 6, 1> K;
	const Eigen::Matrix<double, 7, 7> I;
	int rate;
	Eigen::Matrix<double, 7, 1> qNull;
	Eigen::Matrix<double, 7, 1> alpha;
	int c;
	int currentWaypoint;
	Eigen::Vector3d desiredPos;
	Eigen::Quaterniond desiredRot;
	std::vector<Waypoint> waypoints;
	Eigen::Affine3d trans, trans2;

	void publishJS()
	{
		js.position[0] = q(0);
		js.position[1] = q(1);
		js.position[2] = q(2);
		js.position[3] = q(3);
		js.position[4] = q(4);
		js.position[5] = q(5);
		js.position[6] = q(6);
		js.header.stamp = ros::Time::now();

		jsPub.publish(js);
	}

public:
	Examples() :
					pubEEf("/iiwa/eef_traj", "/iiwa/iiwa_0_link", "eef", Eigen::Vector4f(1.0, 0.0, 0.0, 1.0)),
					pubPath("/iiwa/desired_path", "/iiwa/iiwa_0_link", "path"),
					board("/iiwa/board", "/iiwa/iiwa_0_link", "board"),
					tool("/iiwa/tool", "/iiwa/iiwa_flange_link", "tool", Eigen::Vector4f(1.0, 0.0, 0.0, 1.0)),
					rate(2000),
					I(Eigen::Matrix<double, 7, 7>::Identity()),
					qNull(Eigen::Matrix<double, 7, 1>::Zero()),
					c(0),
					currentWaypoint(0)
	{
		jsPub = n.advertise<sensor_msgs::JointState>("/iiwa/joint_states", 10);
		js.name.reserve(7);
		js.position.resize(7);
		js.name.push_back("iiwa_1_joint");
		js.name.push_back("iiwa_2_joint");
		js.name.push_back("iiwa_3_joint");
		js.name.push_back("iiwa_4_joint");
		js.name.push_back("iiwa_5_joint");
		js.name.push_back("iiwa_6_joint");
		js.name.push_back("iiwa_7_joint");

		x.setZero();
		error.setZero();
		K << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1;
		alpha << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;

		//setup transformations for the board. We have two of them.
		//the first one is used to scale the graphic. Orientation and position should
		//be the same
		trans.linear() = Waypoint::fromAxis( { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 });
		trans.scale(0.007); //0.015
		trans.translation() = Eigen::Vector3d(0, 0.4, 0.8);

		trans2.linear() = Waypoint::fromAxis( { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 });
		trans2.translation() = Eigen::Vector3d(0, 0.4, 0.8);

		//import waypoints
//		waypoints = HTML5LineImporter("/home/kuhnerd/Desktop/drawing2.html").getWaypoints(trans);
//		waypoints = tSquare;
		waypoints = mytrajectory;
		for (auto& it: waypoints) {
			it.translate({0.2, 0, 0});
		}
//		waypoints = tPoint2;
		LOG_INFO("Got " << waypoints.size() << " waypoints");

		//add 5 seconds to each waypoint to enable the robot to move to the first waypoint
		for (auto& it : waypoints)
		{
			it.t += 5.0;
		}
	}

	void geometricDrawHTML5File()
	{
		//init joint state
		q << M_PI_4, 0.1, 0, M_PI_2, 0, M_PI_4, -M_PI_4;
//		q << M_PI_4, 0, 0, 0, 0, 0, 0;

		ros::Rate r(rate);

		pubPath.send(waypoints);

		kin.updateFK(q);
//		LOG_INFO(kin.fk().matrix());
//		LOG_INFO(kin.m_tr1.matrix());
//		LOG_INFO(kin.m_tr2.matrix());
//		LOG_INFO(kin.m_tr3.matrix());
//		LOG_INFO(kin.m_tr4.matrix());
//		LOG_INFO(kin.m_tr5.matrix());
//		LOG_INFO(kin.m_tr6.matrix());
//		LOG_INFO(kin.m_tr7.matrix());
		Eigen::Isometry3d fk = kin.fk();

		Waypoint currentGoal { 0, Eigen::Vector3i::Zero(), Eigen::Vector3d(fk(0, 3), fk(1, 3), fk(2, 3)), kin.fkOrientationM() };
		Waypoint oldGoal = currentGoal;
		desiredPos = currentGoal.pos;
		desiredRot = currentGoal.asQuaternion();

		const double tStart = ros::Time::now().toSec();
		const double cycleTime = r.expectedCycleTime().toSec();
		const int visAddPointsAll = (int) (rate / 50);
		bool drawNextLine = false;
		for (double t = tStart; ros::ok(); t = ros::Time::now().toSec(), c++)
		{
			if (t >= currentGoal.t + tStart)
			{
				oldGoal = currentGoal;
				currentGoal = waypoints[currentWaypoint];
				if (currentWaypoint < waypoints.size() - 1)
				{
					++currentWaypoint;
					if (currentGoal.flag == 0)
						drawNextLine = false;
				}
			}

			if (currentGoal.t == oldGoal.t)
			{
				x.setZero();
				desiredPos = currentGoal.pos;
				desiredRot = currentGoal.asQuaternion();
			}
			else
			{
				double dt = currentGoal.t - oldGoal.t;
				double currentWaypointProgress = ((t - tStart - oldGoal.t) / dt);

				//linear
				x.head(3) = (currentGoal.pos - oldGoal.pos) / dt;
				desiredPos = oldGoal.pos + (currentWaypointProgress * (currentGoal.pos - oldGoal.pos));

				//rotation
				x.tail(3) = KUKAIIWA14820::quaternionDiffToAngularVelocity(oldGoal.asQuaternion(), currentGoal.asQuaternion(), dt);
				desiredRot = oldGoal.asQuaternion().slerp(currentWaypointProgress, currentGoal.asQuaternion());
			}

			error.head(3) = desiredPos - kin.fkPos();
			error.tail(3) = KUKAIIWA14820::quaternionDiffToAngularVelocity(kin.fkOrientationQ(), desiredRot, 1.0);
			LOG_INFO(error.transpose())

			Eigen::Matrix<double, 6, 1> xCorrected = cycleTime * x + K.cwiseProduct(error);

			kin.updateFK(q);
			const auto& j = kin.geometricJacobian();
			const auto& jinv = JacobianInvertors::dlsInv(j, xCorrected);
			//		const auto& jinv = JacobianInvertors::svdInv(j);

			//null space projector
			auto nProj = I - jinv * j;

			//nullspace motions: joint range
			for (int i = 0; i < 7; ++i)
			{
				qNull(i) = -alpha(i) * (1.0 / 7.0) * (q(i) / (KUKAIIWA14820::qmax[i] - KUKAIIWA14820::qmin[i]));
			}

			q += jinv * xCorrected + nProj * qNull;

			kin.updateFK(q);
			const auto newPose = kin.fkPos();
			board.draw(trans2.inverse() * newPose, currentGoal.color);

			if (c % (rate / 10) == 0)
			{
				tool.send();
				board.publishBoard(trans);
				pubPath.send(waypoints); //resend
			}

			publishJS();
			kin.publishFrames();

			r.sleep();
		}
	}

	void analyticDrawHTML5File()
	{
		//init joint state
		q << 0, M_PI_4, 0, M_PI_2, 0, M_PI_4, -M_PI_4;
//		q << 0, 0, 0, 0, 0, 0, 0;
		K << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1;

		ros::Rate r(rate);

		kin.updateFK(q);
		Eigen::Isometry3d fk = kin.fk();

		Waypoint currentGoal { 0, Eigen::Vector3i::Zero(), Eigen::Vector3d(fk(0, 3), fk(1, 3), fk(2, 3)), kin.fkOrientationM() };
		Waypoint oldGoal = currentGoal;
		desiredPos = currentGoal.pos;

		const double tStart = ros::Time::now().toSec();
		const double cycleTime = r.expectedCycleTime().toSec();
		const int visAddPointsAll = (int) (rate / 50);
		bool drawNextLine = false;
		for (double t = tStart; ros::ok(); t = ros::Time::now().toSec(), c++)
		{
			if (t >= currentGoal.t + tStart)
			{
				oldGoal = currentGoal;
				currentGoal = waypoints[currentWaypoint];
//				LOG_INFO(currentGoal.asXYZ().transpose());
				if (currentWaypoint < waypoints.size() - 1)
				{
					++currentWaypoint;
					if (currentGoal.flag == 0)
						drawNextLine = false;
				}
			}

			if (currentGoal.t == oldGoal.t)
			{
				x.setZero();
				desiredPos = currentGoal.pos;
				desiredRot = currentGoal.asQuaternion();
			}
			else
			{
				double dt = currentGoal.t - oldGoal.t;
				double currentWaypointProgress = ((t - tStart - oldGoal.t) / dt);

				//linear
				x.head(3) = (currentGoal.pos - oldGoal.pos) / dt;
				desiredPos = oldGoal.pos + (currentWaypointProgress * (currentGoal.pos - oldGoal.pos));

				//rotation
//				x.tail(3) = (currentGoal.asQuaternion() * oldGoal.asQuaternion().conjugate()).matrix().eulerAngles(0, 1, 2) / dt; //KUKAIIWA14820::angleDiff(currentGoal.asXYZ(), kin.fkOrientationXYZ()) / dt;
				x.tail(3) = KUKAIIWA14820::angleDiff(currentGoal.asXYZ(), oldGoal.asXYZ()) / dt;
				desiredRot = oldGoal.asQuaternion().slerp(currentWaypointProgress, currentGoal.asQuaternion());
			}

//			LOG_INFO(desiredRot.coeffs().transpose());
			//			LOG_INFO((currentGoal.asRPY() - kin.fkOrientationRPY()).transpose());
//			LOG_INFO(currentGoal.t - oldGoal.t);
//			LOG_INFO(currentGoal.asXYZ().transpose());
//			LOG_INFO(kin.fkOrientationXYZ().transpose());

			error.head(3) = desiredPos - kin.fkPos();
			error.tail(3) = KUKAIIWA14820::angleDiff(desiredRot.matrix().eulerAngles(0, 1, 2), kin.fkOrientationXYZ());

			Eigen::Matrix<double, 6, 1> xCorrected = cycleTime * x + K.cwiseProduct(error);

			kin.updateFK(q);
			const auto& j = kin.analyticJacobianXYZ();
			const auto& jinv = JacobianInvertors::dlsInv(j, xCorrected);
			//		const auto& jinv = JacobianInvertors::svdInv(j);

			//null space projector
			auto nProj = I - jinv * j;

			//nullspace motions: joint range
			for (int i = 0; i < 7; ++i)
			{
				qNull(i) = -alpha(i) * (1.0 / 7.0) * (q(i) / (KUKAIIWA14820::qmax[i] - KUKAIIWA14820::qmin[i]));
			}

			q += jinv * xCorrected + nProj * qNull;

			kin.updateFK(q);
			const auto newPose = kin.fkPos();
			board.draw(trans2.inverse() * newPose, currentGoal.color);

			if (c % (rate / 10) == 0)
			{
				tool.send();
				board.publishBoard(trans);
				pubPath.send(waypoints); //resend
			}

			publishJS();

			r.sleep();
		}
	}

	void constrainedAnalyticDraw()
	{
		//init joint state
		q << 0, M_PI_4, 0, M_PI_2, 0, M_PI_4, -M_PI_4;
		//		q << 0, 0, 0, 0, 0, 0, 0;
		K << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1;

		ros::Rate r(rate);

		kin.updateFK(q);
		Eigen::Isometry3d fk = kin.fk();

		Waypoint currentGoal { 0, Eigen::Vector3i::Zero(), Eigen::Vector3d(fk(0, 3), fk(1, 3), fk(2, 3)), kin.fkOrientationM() };
		Waypoint oldGoal = currentGoal;
		desiredPos = currentGoal.pos;

		const double tStart = ros::Time::now().toSec();
		const double cycleTime = r.expectedCycleTime().toSec();
		const int visAddPointsAll = (int) (rate / 50);
		bool drawNextLine = false;
		for (double t = tStart; ros::ok(); t = ros::Time::now().toSec(), c++)
		{
			if (t >= currentGoal.t + tStart)
			{
				oldGoal = currentGoal;
				currentGoal = waypoints[currentWaypoint];
				//				LOG_INFO(currentGoal.asXYZ().transpose());
				if (currentWaypoint < waypoints.size() - 1)
				{
					++currentWaypoint;
					if (currentGoal.flag == 0)
						drawNextLine = false;
				}
			}

			if (currentGoal.t == oldGoal.t)
			{
				x.setZero();
				desiredPos = currentGoal.pos;
				desiredRot = currentGoal.asQuaternion();
			}
			else
			{
				double dt = currentGoal.t - oldGoal.t;
				double currentWaypointProgress = ((t - tStart - oldGoal.t) / dt);

				//linear
				x.head(3) = (currentGoal.pos - oldGoal.pos) / dt;
				desiredPos = oldGoal.pos + (currentWaypointProgress * (currentGoal.pos - oldGoal.pos));

				//rotation
				//				x.tail(3) = (currentGoal.asQuaternion() * oldGoal.asQuaternion().conjugate()).matrix().eulerAngles(0, 1, 2) / dt; //KUKAIIWA14820::angleDiff(currentGoal.asXYZ(), kin.fkOrientationXYZ()) / dt;
				x.tail(3) = KUKAIIWA14820::angleDiff(currentGoal.asXYZ(), oldGoal.asXYZ()) / dt;
				desiredRot = oldGoal.asQuaternion().slerp(currentWaypointProgress, currentGoal.asQuaternion());
			}

			//			LOG_INFO(desiredRot.coeffs().transpose());
			//			LOG_INFO((currentGoal.asRPY() - kin.fkOrientationRPY()).transpose());
			//			LOG_INFO(currentGoal.t - oldGoal.t);
			//			LOG_INFO(currentGoal.asXYZ().transpose());
			//			LOG_INFO(kin.fkOrientationXYZ().transpose());

			error.head(3) = desiredPos - kin.fkPos();
			error.tail(3) = KUKAIIWA14820::angleDiff(desiredRot.matrix().eulerAngles(0, 1, 2), kin.fkOrientationXYZ());

			Eigen::Matrix<double, 5, 1> xConstrained = x.head(5);
			Eigen::Matrix<double, 5, 1> errorConstrained = error.head(5);

			Eigen::Matrix<double, 5, 1> xCorrected = cycleTime * xConstrained + K.head(5).cwiseProduct(errorConstrained);

			kin.updateFK(q);
			Eigen::Matrix<double, 5, 7> j = kin.analyticJacobianXYZ().block<5, 7>(0, 0);
			const auto& jinv = JacobianInvertors::dlsInv(j, xCorrected);
			//		const auto& jinv = JacobianInvertors::svdInv(j);

			//null space projector
			auto nProj = I - jinv * j;

			//nullspace motions: joint range
			for (int i = 0; i < 7; ++i)
			{
				qNull(i) = -alpha(i) * (1.0 / 7.0) * (q(i) / (KUKAIIWA14820::qmax[i] - KUKAIIWA14820::qmin[i]));
			}

			q += jinv * xCorrected + nProj * qNull;

			kin.updateFK(q);
			const auto newPose = kin.fkPos();
			board.draw(trans2.inverse() * newPose, currentGoal.color);

			if (c % (rate / 10) == 0)
			{
				tool.send();
				board.publishBoard(trans);
				pubPath.send(waypoints); //resend
			}

			publishJS();

			r.sleep();
		}
	}
};

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle n;

	Examples examples;
	examples.geometricDrawHTML5File();
//	examples.analyticDrawHTML5File();
//	examples.constrainedAnalyticDraw();

	return 0;
}
