#pragma once

#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_types/internal_types.hpp"

namespace sensor_adapter
{
class RoboSenseFairyAdapter
{
public:
    bool convert(const sensor_msgs::msg::PointCloud2 &msg, InternalScan &scan) const;
    bool convertToFilteredCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                pcl::PointCloud<InternalPointType> &output,
                                std::uint16_t scan_line_count,
                                int point_filter_num,
                                double blind,
                                double &scan_start_time,
                                double &scan_end_time) const;
};
}  // namespace sensor_adapter
