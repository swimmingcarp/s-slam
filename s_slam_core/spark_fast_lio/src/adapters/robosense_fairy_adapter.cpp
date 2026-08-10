#include "adapters/robosense_fairy_adapter.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace sensor_adapter
{
namespace
{
struct RoboSensePoint
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    std::uint16_t ring = 0;
    double timestamp = 0.0;
};

struct XYZIRTFieldLayout
{
    std::uint32_t x_offset = 0;
    std::uint32_t y_offset = 0;
    std::uint32_t z_offset = 0;
    std::uint32_t intensity_offset = 0;
    std::uint32_t ring_offset = 0;
    std::uint32_t timestamp_offset = 0;
};

constexpr bool hostIsBigEndian()
{
    return std::endian::native == std::endian::big;
}

template <typename T>
T readField(const std::uint8_t *const field_data, const bool message_is_bigendian)
{
    T value;
    std::memcpy(&value, field_data, sizeof(value));
    if (message_is_bigendian != hostIsBigEndian())
    {
        std::array<std::uint8_t, sizeof(T)> bytes;
        std::memcpy(bytes.data(), &value, sizeof(value));
        std::reverse(bytes.begin(), bytes.end());
        std::memcpy(&value, bytes.data(), sizeof(value));
    }
    return value;
}

std::optional<XYZIRTFieldLayout> getXYZIRTFieldLayout(
    const sensor_msgs::msg::PointCloud2 &msg)
{
    using sensor_msgs::msg::PointField;
    struct RequiredField
    {
        const char *name;
        std::uint8_t datatype;
        std::size_t size;
        std::uint32_t XYZIRTFieldLayout::*offset;
    };
    constexpr RequiredField required_fields[] = {
        {"x", PointField::FLOAT32, sizeof(float), &XYZIRTFieldLayout::x_offset},
        {"y", PointField::FLOAT32, sizeof(float), &XYZIRTFieldLayout::y_offset},
        {"z", PointField::FLOAT32, sizeof(float), &XYZIRTFieldLayout::z_offset},
        {"intensity", PointField::FLOAT32, sizeof(float), &XYZIRTFieldLayout::intensity_offset},
        {"ring", PointField::UINT16, sizeof(std::uint16_t), &XYZIRTFieldLayout::ring_offset},
        {"timestamp", PointField::FLOAT64, sizeof(double), &XYZIRTFieldLayout::timestamp_offset},
    };

    XYZIRTFieldLayout layout;
    std::ostringstream missing;
    bool valid = true;
    for (const auto &required_field : required_fields)
    {
        const auto field = std::find_if(
            msg.fields.begin(), msg.fields.end(), [&](const sensor_msgs::msg::PointField &field) {
                return field.name == required_field.name && field.datatype == required_field.datatype &&
                       field.count == 1;
            });
        if (field == msg.fields.end() || field->offset > msg.point_step ||
            required_field.size > msg.point_step - field->offset)
        {
            if (!valid)
            {
                missing << ", ";
            }
            missing << required_field.name;
            valid = false;
            continue;
        }
        layout.*(required_field.offset) = field->offset;
    }

    if (!valid)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("LidarProcessor"),
            "RoboSense Fairy point cloud must be rslidar_sdk POINT_TYPE=XYZIRT "
            "(missing/wrong fields: %s). Drop this scan.",
            missing.str().c_str());
        return std::nullopt;
    }

    const std::size_t point_count = static_cast<std::size_t>(msg.width) * msg.height;
    const std::size_t minimum_row_size = static_cast<std::size_t>(msg.width) * msg.point_step;
    if (msg.point_step == 0 || msg.row_step < minimum_row_size ||
        msg.data.size() < static_cast<std::size_t>(msg.height) * msg.row_step)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "RoboSense XYZIRT point cloud has an invalid PointCloud2 layout. Drop this scan.");
        return std::nullopt;
    }
    if (point_count == 0)
    {
        return std::nullopt;
    }
    return layout;
}

template <typename PointVisitor>
void forEachXYZIRTPoint(const sensor_msgs::msg::PointCloud2 &msg,
                        const XYZIRTFieldLayout &layout,
                        PointVisitor &&visitor)
{
    std::size_t point_index = 0;
    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        const std::uint8_t *const row_data =
            msg.data.data() + static_cast<std::size_t>(row) * msg.row_step;
        for (std::uint32_t column = 0; column < msg.width; ++column, ++point_index)
        {
            const std::uint8_t *const point_data =
                row_data + static_cast<std::size_t>(column) * msg.point_step;
            visitor(point_index,
                    RoboSensePoint{readField<float>(point_data + layout.x_offset, msg.is_bigendian),
                                   readField<float>(point_data + layout.y_offset, msg.is_bigendian),
                                   readField<float>(point_data + layout.z_offset, msg.is_bigendian),
                                   readField<float>(point_data + layout.intensity_offset, msg.is_bigendian),
                                   readField<std::uint16_t>(point_data + layout.ring_offset,
                                                            msg.is_bigendian),
                                   readField<double>(point_data + layout.timestamp_offset,
                                                     msg.is_bigendian)});
        }
    }
}

bool hasFiniteXYZ(const RoboSensePoint &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool hasValidTimestamp(const RoboSensePoint &point)
{
    return std::isfinite(point.timestamp) && point.timestamp > 0.0;
}

void fillPoint(const RoboSensePoint &src,
               const double scan_start_time,
               InternalPointType &dst)
{
    dst           = InternalPointType{};
    dst.data[3]   = 1.0F;
    dst.data_n[3] = 0.0F;
    dst.x         = src.x;
    dst.y         = src.y;
    dst.z         = src.z;
    dst.intensity = src.intensity;
    dst.curvature = static_cast<float>((src.timestamp - scan_start_time) * 1000.0);
}

bool isValidPoint(const RoboSensePoint &point)
{
    return hasValidTimestamp(point) && hasFiniteXYZ(point);
}

void logInvalidTimestampCloud()
{
    RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                 "RoboSense XYZIRT point cloud has no finite point with a valid per-point "
                 "timestamp. Drop this scan.");
}
}  // namespace

bool RoboSenseFairyAdapter::convert(const sensor_msgs::msg::PointCloud2 &msg, InternalScan &scan) const
{
    scan.points.clear();
    scan.start_time = -1.0;
    scan.end_time   = -1.0;

    const auto layout = getXYZIRTFieldLayout(msg);
    if (!layout)
    {
        return false;
    }

    // RoboSense Fairy publishes absolute per-point timestamps in seconds.
    double scan_start_time = std::numeric_limits<double>::max();
    double scan_end_time   = -std::numeric_limits<double>::max();
    forEachXYZIRTPoint(msg, *layout, [&](const std::size_t, const RoboSensePoint &point) {
        if (!isValidPoint(point))
        {
            return;
        }
        scan_start_time = std::min(scan_start_time, point.timestamp);
        scan_end_time   = std::max(scan_end_time, point.timestamp);
    });

    if (scan_start_time == std::numeric_limits<double>::max())
    {
        logInvalidTimestampCloud();
        return false;
    }

    scan.start_time = scan_start_time;
    scan.end_time   = scan_end_time;
    scan.points.reserve(static_cast<std::size_t>(msg.width) * msg.height);
    forEachXYZIRTPoint(msg, *layout, [&](const std::size_t point_index, const RoboSensePoint &src) {
        if (!isValidPoint(src))
        {
            return;
        }

        InternalPoint dst;
        fillPoint(src, scan_start_time, dst.point);
        dst.ring         = src.ring;
        dst.source_index = point_index;
        scan.points.push_back(dst);
    });
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

    const auto layout = getXYZIRTFieldLayout(msg);
    if (!layout)
    {
        return false;
    }

    const std::size_t point_stride = static_cast<std::size_t>(std::max(1, point_filter_num));
    const double blind_squared     = blind * blind;
    double first_timestamp         = std::numeric_limits<double>::max();
    double last_timestamp          = -std::numeric_limits<double>::max();
    std::size_t output_size        = 0;

    forEachXYZIRTPoint(msg, *layout, [&](const std::size_t point_index, const RoboSensePoint &point) {
        if (!isValidPoint(point))
        {
            return;
        }

        first_timestamp = std::min(first_timestamp, point.timestamp);
        last_timestamp  = std::max(last_timestamp, point.timestamp);
        if (point_index % point_stride != 0 || point.ring >= scan_line_count)
        {
            return;
        }

        const double distance_squared =
            point.x * point.x + point.y * point.y + point.z * point.z;
        if (distance_squared > blind_squared)
        {
            ++output_size;
        }
    });

    if (first_timestamp == std::numeric_limits<double>::max())
    {
        logInvalidTimestampCloud();
        return false;
    }

    scan_start_time = first_timestamp;
    scan_end_time   = last_timestamp;
    output.reserve(output_size);

    forEachXYZIRTPoint(msg, *layout, [&](const std::size_t point_index, const RoboSensePoint &point) {
        if (!isValidPoint(point) || point_index % point_stride != 0 ||
            point.ring >= scan_line_count)
        {
            return;
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
    });
    return true;
}
}  // namespace sensor_adapter
