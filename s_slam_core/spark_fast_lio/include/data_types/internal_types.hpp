#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sensor_adapter
{
using InternalPointType = pcl::PointXYZINormal;

struct InternalPoint
{
    InternalPointType point;
    std::uint16_t ring = 0;
    std::size_t source_index = 0;
};

struct InternalScan
{
    std::vector<InternalPoint> points;
    double start_time = -1.0;
    double end_time = -1.0;
};

// Copies the sensor-agnostic geometry/intensity payload shared by every adapter.
// Each adapter is then only responsible for its per-sensor `curvature` (relative time).
template <typename SrcPoint>
inline void assignXYZI(const SrcPoint &src, InternalPointType &dst)
{
    dst.normal_x  = 0;
    dst.normal_y  = 0;
    dst.normal_z  = 0;
    dst.x         = src.x;
    dst.y         = src.y;
    dst.z         = src.z;
    dst.intensity = src.intensity;
}
}  // namespace sensor_adapter
