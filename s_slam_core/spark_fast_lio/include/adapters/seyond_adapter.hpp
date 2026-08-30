#pragma once

#include <cstdint>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_types/internal_types.hpp"

namespace sensor_adapter
{

// Converts Seyond Falcon PointCloud2 messages with absolute per-point timestamps.
class SeyondAdapter
{
public:
    bool convertToFilteredCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                pcl::PointCloud<InternalPointType> &output,
                                int point_filter_stride,
                                double blind_distance,
                                double maximum_scan_duration,
                                double &scan_start_time,
                                double &scan_end_time) const;
};

}  // namespace sensor_adapter
