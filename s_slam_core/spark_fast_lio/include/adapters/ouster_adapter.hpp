#pragma once

#include <cstdint>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_types/internal_types.hpp"

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (std::uint32_t, t, t)
                                    (std::uint16_t, reflectivity, reflectivity)
                                    (std::uint8_t, ring, ring)
                                    (std::uint16_t, ambient, ambient)
                                    (std::uint32_t, range, range)
)
// clang-format on

namespace sensor_adapter
{
class OusterAdapter
{
public:
    bool convert(const sensor_msgs::msg::PointCloud2 &msg,
                 float time_unit_scale,
                 InternalScan &scan) const;

private:
    void fillPoint(const ouster_ros::Point &src, float time_unit_scale, InternalPointType &dst) const;
};
}  // namespace sensor_adapter
