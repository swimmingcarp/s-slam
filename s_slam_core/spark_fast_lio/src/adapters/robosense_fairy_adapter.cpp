#include "adapters/robosense_fairy_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

namespace sensor_adapter
{
bool RoboSenseFairyAdapter::convert(const sensor_msgs::msg::PointCloud2 &msg, InternalScan &scan) const
{
    scan.points.clear();
    scan.start_time = -1.0;
    scan.end_time   = -1.0;

    if (!validateContract(msg))
    {
        return false;
    }

    pcl::PointCloud<rslidar_ros::Point> raw_points;
    pcl::fromROSMsg(msg, raw_points);
    if (raw_points.points.empty())
    {
        return false;
    }

    // RoboSense Fairy publishes absolute per-point timestamps in seconds.
    double scan_start_time = std::numeric_limits<double>::max();
    double scan_end_time   = -std::numeric_limits<double>::max();
    for (const auto &point : raw_points.points)
    {
        if (!hasValidTimestamp(point) || !hasFiniteXYZ(point))
        {
            continue;
        }
        scan_start_time = std::min(scan_start_time, point.timestamp);
        scan_end_time   = std::max(scan_end_time, point.timestamp);
    }

    if (scan_start_time == std::numeric_limits<double>::max())
    {
        RCLCPP_ERROR(rclcpp::get_logger("Preprocess"),
                     "RoboSense XYZIRT point cloud has no finite point with a valid per-point "
                     "timestamp. Drop this scan.");
        return false;
    }

    scan.start_time = scan_start_time;
    scan.end_time   = scan_end_time;
    scan.points.reserve(raw_points.points.size());
    for (std::size_t i = 0; i < raw_points.points.size(); ++i)
    {
        const auto &src = raw_points.points[i];
        if (!hasValidTimestamp(src) || !hasFiniteXYZ(src))
        {
            continue;
        }

        InternalPoint dst;
        fillPoint(src, scan_start_time, dst.point);
        dst.ring         = src.ring;
        dst.source_index = i;
        scan.points.push_back(dst);
    }
    return true;
}

bool RoboSenseFairyAdapter::convertToFilteredCloud(
    const sensor_msgs::msg::PointCloud2 &msg,
    pcl::PointCloud<InternalPointType> &output,
    const std::uint16_t scan_line_count,
    const int point_filter_num,
    const double blind,
    double &scan_start_time,
    double &scan_end_time) const
{
    output.clear();
    scan_start_time = -1.0;
    scan_end_time   = -1.0;

    if (!validateContract(msg))
    {
        return false;
    }

    pcl::PointCloud<rslidar_ros::Point> raw_points;
    pcl::fromROSMsg(msg, raw_points);
    if (raw_points.points.empty())
    {
        return false;
    }

    const std::size_t point_stride = static_cast<std::size_t>(std::max(1, point_filter_num));
    const double blind_squared     = blind * blind;
    double first_timestamp         = std::numeric_limits<double>::max();
    double last_timestamp          = -std::numeric_limits<double>::max();
    std::size_t output_size        = 0;

    for (std::size_t index = 0; index < raw_points.points.size(); ++index)
    {
        const auto &point = raw_points.points[index];
        if (!hasValidTimestamp(point) || !hasFiniteXYZ(point))
        {
            continue;
        }

        first_timestamp = std::min(first_timestamp, point.timestamp);
        last_timestamp  = std::max(last_timestamp, point.timestamp);
        if (index % point_stride != 0 || point.ring >= scan_line_count)
        {
            continue;
        }

        const double distance_squared =
            point.x * point.x + point.y * point.y + point.z * point.z;
        if (distance_squared > blind_squared)
        {
            ++output_size;
        }
    }

    if (first_timestamp == std::numeric_limits<double>::max())
    {
        RCLCPP_ERROR(rclcpp::get_logger("Preprocess"),
                     "RoboSense XYZIRT point cloud has no finite point with a valid per-point "
                     "timestamp. Drop this scan.");
        return false;
    }

    scan_start_time = first_timestamp;
    scan_end_time   = last_timestamp;
    output.reserve(output_size);

    for (std::size_t index = 0; index < raw_points.points.size(); ++index)
    {
        const auto &point = raw_points.points[index];
        if (!hasValidTimestamp(point) || !hasFiniteXYZ(point) ||
            index % point_stride != 0 || point.ring >= scan_line_count)
        {
            continue;
        }

        InternalPointType converted_point;
        fillPoint(point, scan_start_time, converted_point);
        const double distance_squared = converted_point.x * converted_point.x +
                                        converted_point.y * converted_point.y +
                                        converted_point.z * converted_point.z;
        if (distance_squared > blind_squared)
        {
            output.points.push_back(converted_point);
        }
    }
    return true;
}

bool RoboSenseFairyAdapter::validateContract(const sensor_msgs::msg::PointCloud2 &msg) const
{
    using sensor_msgs::msg::PointField;
    const std::pair<const char *, std::uint8_t> required_fields[] = {
        {"x", PointField::FLOAT32},
        {"y", PointField::FLOAT32},
        {"z", PointField::FLOAT32},
        {"intensity", PointField::FLOAT32},
        {"ring", PointField::UINT16},
        {"timestamp", PointField::FLOAT64},
    };

    std::ostringstream missing;
    bool ok = true;
    for (const auto &[name, datatype] : required_fields)
    {
        if (!hasPointField(msg, name, datatype))
        {
            if (!ok)
            {
                missing << ", ";
            }
            missing << name;
            ok = false;
        }
    }

    if (!ok)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("Preprocess"),
            "RoboSense Fairy point cloud must be rslidar_sdk POINT_TYPE=XYZIRT "
            "(missing/wrong fields: %s). Drop this scan.",
            missing.str().c_str());
    }
    return ok;
}

bool RoboSenseFairyAdapter::hasPointField(const sensor_msgs::msg::PointCloud2 &msg,
                                          const std::string &name,
                                          const std::uint8_t datatype) const
{
    return std::any_of(msg.fields.begin(), msg.fields.end(), [&](const auto &field) {
        return field.name == name && field.datatype == datatype && field.count == 1;
    });
}

bool RoboSenseFairyAdapter::hasFiniteXYZ(const rslidar_ros::Point &point) const
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool RoboSenseFairyAdapter::hasValidTimestamp(const rslidar_ros::Point &point) const
{
    return std::isfinite(point.timestamp) && point.timestamp > 0.0;
}

void RoboSenseFairyAdapter::fillPoint(const rslidar_ros::Point &src,
                                      const double scan_start_time,
                                      InternalPointType &dst) const
{
    assignXYZI(src, dst);
    dst.curvature = static_cast<float>((src.timestamp - scan_start_time) * 1000.0);
}
}  // namespace sensor_adapter
