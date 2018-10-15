//------------------ HELPER STUFF ------------------------

#include <ros/ros.h>

#include <cstdint>

#include <moveit_msgs/OrientedBoundingBox.h>
#include <moveit_msgs/CollisionObject.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace geometry_msgs
{

Vector3 CreateVector3(double x, double y, double z)
{
    Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

const Vector3 ZeroVector3()
{
    return CreateVector3(0.0, 0.0, 0.0);
}

Point CreatePoint(double x, double y, double z)
{
    Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

const Point ZeroPoint()
{
    return CreatePoint(0.0, 0.0, 0.0);
}

Point32 CreatePoint32(float x, float y, float z)
{
    Point32 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

const Point32 ZeroPoint32()
{
    return CreatePoint32(0.0f, 0.0f, 0.0f);
}

Quaternion CreateQuaternion(double w, double x, double y, double z)
{
    Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

const Quaternion IdentityQuaternion()
{
    return CreateQuaternion(1.0, 0.0, 0.0, 0.0);
}

Pose CreatePose(const Point& position, const Quaternion& orientation)
{
    Pose p;
    p.position = position;
    p.orientation = orientation;
    return p;
}

const Pose IdentityPose()
{
    return CreatePose(ZeroPoint(), IdentityQuaternion());
}

} // namespace geometry_msgs


const char* to_string(XmlRpc::XmlRpcValue::Type type)
{
    switch (type) {
    case XmlRpc::XmlRpcValue::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::TypeInt:
        return "Integer";
    case XmlRpc::XmlRpcValue::TypeDouble:
        return "Double";
    case XmlRpc::XmlRpcValue::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:
        return "Base64";
    case XmlRpc::XmlRpcValue::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::TypeStruct:
        return "Struct";
    }
}
