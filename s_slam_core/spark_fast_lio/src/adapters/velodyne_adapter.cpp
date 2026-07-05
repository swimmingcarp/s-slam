#include "adapters/velodyne_adapter.hpp"

#include <cmath>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

namespace sensor_adapter
{
bool VelodyneAdapter::convert(const sensor_msgs::msg::PointCloud2 &msg,
                              const int scan_lines,
                              const int scan_rate,
                              const float time_unit_scale,
                              bool &has_offset_time,
                              InternalScan &scan) const
{
    scan.points.clear();
    scan.start_time = -1.0;
    scan.end_time   = -1.0;
    has_offset_time = false;

    pcl::PointCloud<velodyne_ros::Point> raw_points;
    pcl::fromROSMsg(msg, raw_points);
    if (raw_points.points.empty())
    {
        return false;
    }

    has_offset_time = raw_points.points.back().time > 0;
    const double omega_l = 0.361 * scan_rate;
    std::vector<bool> is_first(scan_lines, true);
    std::vector<double> yaw_fp(scan_lines, 0.0);
    std::vector<float> time_last(scan_lines, 0.0);

    scan.points.reserve(raw_points.points.size());
    for (std::size_t i = 0; i < raw_points.points.size(); ++i)
    {
        const auto &src = raw_points.points[i];
        const int layer = src.ring;
        if (!has_offset_time && layer >= scan_lines)
        {
            continue;
        }

        InternalPoint dst;
        fillPoint(src, time_unit_scale, dst.point);

        if (!has_offset_time)
        {
            const double yaw_angle = std::atan2(dst.point.y, dst.point.x) * 57.2957;
            if (is_first[layer])
            {
                yaw_fp[layer]         = yaw_angle;
                is_first[layer]       = false;
                dst.point.curvature   = 0.0;
                time_last[layer]      = dst.point.curvature;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer])
            {
                dst.point.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            }
            else
            {
                dst.point.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (dst.point.curvature < time_last[layer])
            {
                dst.point.curvature += 360.0 / omega_l;
            }

            time_last[layer] = dst.point.curvature;
        }

        dst.ring         = src.ring;
        dst.source_index = i;
        scan.points.push_back(dst);
    }
    return true;
}

void VelodyneAdapter::fillPoint(const velodyne_ros::Point &src,
                                const float time_unit_scale,
                                InternalPointType &dst) const
{
    assignXYZI(src, dst);
    dst.curvature = src.time * time_unit_scale;
}
}  // namespace sensor_adapter
