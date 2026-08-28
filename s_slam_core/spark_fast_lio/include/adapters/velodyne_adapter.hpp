#pragma once

#include <cstdint>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_types/internal_types.hpp"

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, time, time)
                                  (std::uint16_t, ring, ring))

namespace sensor_adapter
{
class VelodyneAdapter
{
public:
    bool convert(const sensor_msgs::msg::PointCloud2 &msg,
                 int scan_lines,
                 int scan_rate,
                 float time_unit_scale,
                 bool &has_offset_time,
                 InternalScan &scan) const;

private:
    void fillPoint(const velodyne_ros::Point &src,
                   float time_unit_scale,
                   float time_offset,
                   InternalPointType &dst) const;
};
}  // namespace sensor_adapter
