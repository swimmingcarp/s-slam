#pragma once

#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_types/internal_types.hpp"

namespace rslidar_ros
{
// Point layout published by rslidar_sdk when built with POINT_TYPE=XYZIRT.
// `timestamp` is absolute seconds; the Fairy adapter converts it to scan-relative ms.
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rslidar_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp))

namespace sensor_adapter
{
class RoboSenseFairyAdapter
{
public:
    bool convert(const sensor_msgs::msg::PointCloud2 &msg, InternalScan &scan) const;

private:
    bool validateContract(const sensor_msgs::msg::PointCloud2 &msg) const;
    bool hasPointField(const sensor_msgs::msg::PointCloud2 &msg,
                       const std::string &name,
                       std::uint8_t datatype) const;
    bool hasFiniteXYZ(const rslidar_ros::Point &point) const;
    bool hasValidTimestamp(const rslidar_ros::Point &point) const;
    void fillPoint(const rslidar_ros::Point &src, double scan_start_time, InternalPointType &dst) const;
};
}  // namespace sensor_adapter
