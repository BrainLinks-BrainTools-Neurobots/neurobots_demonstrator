#ifndef KNOWLEDGE_BASE_H
#define KNOWLEDGE_BASE_H

#include <ros/ros.h>
#include <boost/variant.hpp>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/variant.hpp>

using boost::serialization::split_free;

namespace boost
{
/**
 * Serialization method for Eigen::Vector3d
 */
template<class Archive, typename _Scalar, int _Options>
inline void serialize(
		Archive & ar,
		Eigen::Quaternion<_Scalar, _Options> & t,
		const unsigned int file_version
		)
{
	ar & t.x();
	ar & t.y();
	ar & t.z();
	ar & t.w();
}

//for Eigen::Matrix
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void save(Archive & ar,
		const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m,
		const unsigned int version)
{
	int rows = m.rows(), cols = m.cols();
	ar & rows;
	ar & cols;
	for (size_t i = 0; i < rows * cols; ++i)
	{
	        bool nan = std::isnan(m.data()[i]);
		_Scalar value = nan ? 0 : m.data()[i];
		ar & nan & value;
	}
}

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void load(Archive & ar,
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m,
		const unsigned int version)
{
	int rows, cols;
	ar & rows;
	ar & cols;
	m.resize(rows, cols);
	for (size_t i = 0; i < rows * cols; ++i)
	{
		bool nan;
		_Scalar value;
		ar & nan & value;

		if (nan)
		{
			m.data()[i] = std::numeric_limits<_Scalar>::quiet_NaN();
		}
		else
		{
			m.data()[i] = value;
		}
	}
}

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void serialize(Archive & ar,
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m,
		const unsigned int version)
{
	split_free(ar, m, version);
}

//for Eigen::Transform
template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void serialize(
		Archive & ar,
		Eigen::Transform<_Scalar, _Dim, _Mode, _Options> & t,
		const unsigned int file_version
		)
{
	ar & t.matrix();
}

}

//Datastructure for Knowledge Base Objects
typedef std::vector<boost::variant<double, int, bool, std::string, Eigen::Vector3d, Eigen::Quaterniond, Eigen::Affine3d, std::vector<double>>> AttributeValues; /**< Attribute values*/
typedef std::unordered_map<std::string, AttributeValues> Object; /**< Attribute names and values*/
typedef std::unordered_map<std::string, Object> ObjectMap; /**< World objects and attributes*/

/**
 * Knowledge Base representation of the PDDL world description
 */
struct World
{
	std::string domain;
	ObjectMap objects;
	std::vector<std::string> locatables;
	std::vector<std::string> perceptibles;
};

#endif // KNOWLEDGE_BASE_H
