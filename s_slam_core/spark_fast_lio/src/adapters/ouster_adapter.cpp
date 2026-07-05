#include "adapters/ouster_adapter.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace sensor_adapter
{
bool OusterAdapter::convert(const sensor_msgs::msg::PointCloud2 &msg,
                            const float time_unit_scale,
                            InternalScan &scan) const
{
    scan.points.clear();
    scan.start_time = -1.0;
    scan.end_time   = -1.0;

    pcl::PointCloud<ouster_ros::Point> raw_points;
    pcl::fromROSMsg(msg, raw_points);
    if (raw_points.points.empty())
    {
        return false;
    }

    scan.points.reserve(raw_points.points.size());
    for (std::size_t i = 0; i < raw_points.points.size(); ++i)
    {
        InternalPoint dst;
        fillPoint(raw_points.points[i], time_unit_scale, dst.point);
        dst.ring         = raw_points.points[i].ring;
        dst.source_index = i;
        scan.points.push_back(dst);
    }
    return true;
}

void OusterAdapter::fillPoint(const ouster_ros::Point &src,
                              const float time_unit_scale,
                              InternalPointType &dst) const
{
    assignXYZI(src, dst);
    dst.curvature = src.t * time_unit_scale;
}
}  // namespace sensor_adapter
