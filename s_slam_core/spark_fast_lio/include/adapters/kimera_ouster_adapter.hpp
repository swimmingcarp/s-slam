#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adapters/ouster_adapter.hpp"

namespace sensor_adapter
{
class KimeraOusterAdapter
{
public:
    bool convert(const sensor_msgs::msg::PointCloud2 &msg,
                 float time_unit_scale,
                 InternalScan &scan) const;

private:
    void fillPoint(const ouster_ros::Point &src, float time_unit_scale, InternalPointType &dst) const;
};
}  // namespace sensor_adapter
