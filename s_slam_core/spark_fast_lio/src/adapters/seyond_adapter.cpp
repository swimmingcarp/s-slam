#include "adapters/seyond_adapter.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace sensor_adapter
{
namespace
{
struct FieldLayout
{
    std::uint32_t x_offset = 0;
    std::uint32_t y_offset = 0;
    std::uint32_t z_offset = 0;
    std::uint32_t intensity_offset = 0;
    std::uint32_t timestamp_offset = 0;
};

struct Point
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    double timestamp = 0.0;
};

struct TimestampWindow
{
    double start_time = -1.0;
    double end_time = -1.0;
};

constexpr bool hostIsBigEndian()
{
    return std::endian::native == std::endian::big;
}

template <typename T>
T readField(const std::uint8_t *field_data, const bool message_is_bigendian)
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

std::optional<FieldLayout> getFieldLayout(const sensor_msgs::msg::PointCloud2 &msg)
{
    using sensor_msgs::msg::PointField;
    struct RequiredField
    {
        const char *name;
        std::uint8_t datatype;
        std::size_t size;
        std::uint32_t FieldLayout::*offset;
    };
    constexpr RequiredField required_fields[] = {
        {"x", PointField::FLOAT32, sizeof(float), &FieldLayout::x_offset},
        {"y", PointField::FLOAT32, sizeof(float), &FieldLayout::y_offset},
        {"z", PointField::FLOAT32, sizeof(float), &FieldLayout::z_offset},
        {"intensity", PointField::FLOAT32, sizeof(float), &FieldLayout::intensity_offset},
        {"timestamp", PointField::FLOAT64, sizeof(double), &FieldLayout::timestamp_offset},
    };

    if (msg.point_step == 0 ||
        msg.row_step < static_cast<std::size_t>(msg.width) * msg.point_step ||
        msg.data.size() < static_cast<std::size_t>(msg.height) * msg.row_step)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "Seyond Falcon point cloud has an invalid PointCloud2 layout. Drop this scan.");
        return std::nullopt;
    }

    FieldLayout layout;
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
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "Seyond Falcon point cloud is missing/wrong fields: %s. Drop this scan.",
                     missing.str().c_str());
        return std::nullopt;
    }
    if (static_cast<std::size_t>(msg.width) * msg.height == 0)
    {
        return std::nullopt;
    }
    return layout;
}

template <typename PointVisitor>
void forEachPoint(const sensor_msgs::msg::PointCloud2 &msg,
                  const FieldLayout &layout,
                  PointVisitor &&visitor)
{
    std::size_t point_index = 0;
    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        const std::uint8_t *row_data =
            msg.data.data() + static_cast<std::size_t>(row) * msg.row_step;
        for (std::uint32_t column = 0; column < msg.width; ++column, ++point_index)
        {
            const std::uint8_t *point_data =
                row_data + static_cast<std::size_t>(column) * msg.point_step;
            visitor(point_index,
                    Point{readField<float>(point_data + layout.x_offset, msg.is_bigendian),
                          readField<float>(point_data + layout.y_offset, msg.is_bigendian),
                          readField<float>(point_data + layout.z_offset, msg.is_bigendian),
                          readField<float>(point_data + layout.intensity_offset, msg.is_bigendian),
                          readField<double>(point_data + layout.timestamp_offset, msg.is_bigendian)});
        }
    }
}

bool isValidPoint(const Point &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           std::isfinite(point.intensity) && std::isfinite(point.timestamp) && point.timestamp > 0.0;
}

std::optional<TimestampWindow> findTimestampWindow(const sensor_msgs::msg::PointCloud2 &msg,
                                                    const FieldLayout &layout,
                                                    const double maximum_scan_duration)
{
    if (!std::isfinite(maximum_scan_duration) || maximum_scan_duration <= 0.0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "Seyond maximum scan duration must be positive and finite. Drop this scan.");
        return std::nullopt;
    }

    double first_timestamp = std::numeric_limits<double>::max();
    double last_timestamp = -std::numeric_limits<double>::max();
    std::size_t valid_count = 0;
    forEachPoint(msg, layout, [&](std::size_t, const Point &point) {
        if (!isValidPoint(point))
        {
            return;
        }
        first_timestamp = std::min(first_timestamp, point.timestamp);
        last_timestamp = std::max(last_timestamp, point.timestamp);
        ++valid_count;
    });
    if (valid_count == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "Seyond Falcon point cloud has no finite point with a valid per-point timestamp. "
                     "Drop this scan.");
        return std::nullopt;
    }
    if (last_timestamp - first_timestamp <= maximum_scan_duration)
    {
        return TimestampWindow{first_timestamp, last_timestamp};
    }

    std::vector<double> timestamps;
    timestamps.reserve(valid_count);
    forEachPoint(msg, layout, [&](std::size_t, const Point &point) {
        if (isValidPoint(point))
        {
            timestamps.push_back(point.timestamp);
        }
    });
    std::sort(timestamps.begin(), timestamps.end());

    std::size_t window_begin = 0;
    std::size_t best_begin = 0;
    std::size_t best_end = 0;
    for (std::size_t window_end = 0; window_end < timestamps.size(); ++window_end)
    {
        while (timestamps[window_end] - timestamps[window_begin] > maximum_scan_duration)
        {
            ++window_begin;
        }
        if (window_end - window_begin > best_end - best_begin)
        {
            best_begin = window_begin;
            best_end = window_end;
        }
    }

    const std::size_t inlier_count = best_end - best_begin + 1;
    if (inlier_count <= timestamps.size() / 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "Seyond Falcon point cloud has no dominant timestamp window. Drop this scan.");
        return std::nullopt;
    }
    RCLCPP_WARN(rclcpp::get_logger("LidarProcessor"),
                "Seyond Falcon point cloud has %zu timestamp outlier points; ignore them.",
                timestamps.size() - inlier_count);
    return TimestampWindow{timestamps[best_begin], timestamps[best_end]};
}

}  // namespace

bool SeyondAdapter::convertToFilteredCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                           pcl::PointCloud<InternalPointType> &output,
                                           const int point_filter_stride,
                                           const double blind_distance,
                                           const double maximum_scan_duration,
                                           double &scan_start_time,
                                           double &scan_end_time) const
{
    output.clear();
    scan_start_time = -1.0;
    scan_end_time = -1.0;

    const auto layout = getFieldLayout(msg);
    if (!layout || point_filter_stride <= 0 || !std::isfinite(blind_distance) ||
        blind_distance < 0.0)
    {
        return false;
    }
    const auto timestamp_window = findTimestampWindow(msg, *layout, maximum_scan_duration);
    if (!timestamp_window)
    {
        return false;
    }

    scan_start_time = timestamp_window->start_time;
    scan_end_time = timestamp_window->end_time;
    const double blind_distance_squared = blind_distance * blind_distance;
    output.reserve(static_cast<std::size_t>(msg.width) * msg.height /
                   static_cast<std::size_t>(point_filter_stride));
    forEachPoint(msg, *layout, [&](const std::size_t point_index, const Point &point) {
        if (!isValidPoint(point) || point.timestamp < scan_start_time ||
            point.timestamp > scan_end_time ||
            point_index % static_cast<std::size_t>(point_filter_stride) != 0)
        {
            return;
        }

        const double distance_squared =
            point.x * point.x + point.y * point.y + point.z * point.z;
        if (distance_squared <= blind_distance_squared)
        {
            return;
        }

        InternalPointType converted_point{};
        converted_point.data[3] = 1.0F;
        converted_point.x = point.x;
        converted_point.y = point.y;
        converted_point.z = point.z;
        converted_point.intensity = point.intensity;
        converted_point.curvature =
            static_cast<float>((point.timestamp - scan_start_time) * 1000.0);
        output.points.push_back(converted_point);
    });
    return !output.empty();
}

}  // namespace sensor_adapter
