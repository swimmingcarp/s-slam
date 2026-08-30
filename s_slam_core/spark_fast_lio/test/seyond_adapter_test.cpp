#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

#include "adapters/seyond_adapter.hpp"
#include "data_processors/lidar_processor.hpp"

namespace
{
struct SeyondPoint
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    double timestamp = 0.0;
};

template <typename T>
void writeField(std::vector<std::uint8_t> &data, const std::size_t offset, const T value)
{
    std::memcpy(data.data() + offset, &value, sizeof(value));
}

sensor_msgs::msg::PointField makeField(const std::string &name,
                                       const std::uint32_t offset,
                                       const std::uint8_t datatype)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    return field;
}

sensor_msgs::msg::PointCloud2 makeSeyondCloud(const std::vector<SeyondPoint> &points)
{
    using sensor_msgs::msg::PointField;
    constexpr std::uint32_t kPointStep = 48;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.point_step = kPointStep;
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.fields = {
        makeField("x", 0, PointField::FLOAT32),
        makeField("y", 4, PointField::FLOAT32),
        makeField("z", 8, PointField::FLOAT32),
        makeField("timestamp", 16, PointField::FLOAT64),
        makeField("intensity_raw", 24, PointField::UINT16),
        makeField("flags", 26, PointField::UINT8),
        makeField("elongation", 27, PointField::UINT8),
        makeField("scan_id", 28, PointField::UINT16),
        makeField("scan_idx", 30, PointField::UINT16),
        makeField("is_2nd_return", 32, PointField::UINT8),
        makeField("intensity", 36, PointField::FLOAT32),
        makeField("time", 40, PointField::FLOAT32),
    };
    cloud.data.resize(cloud.row_step);
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const std::size_t offset = index * kPointStep;
        const auto &point = points[index];
        writeField(cloud.data, offset, point.x);
        writeField(cloud.data, offset + 4, point.y);
        writeField(cloud.data, offset + 8, point.z);
        writeField(cloud.data, offset + 16, point.timestamp);
        writeField(cloud.data, offset + 36, point.intensity);
    }
    return cloud;
}

}  // namespace

TEST(SeyondAdapter, ConvertsTimestampedPointsWithoutAssumingScanLines)
{
    const sensor_msgs::msg::PointCloud2 input = makeSeyondCloud({
        {1.0F, 0.0F, 0.0F, 4.0F, 10.000},
        {2.0F, 0.0F, 0.0F, 5.0F, 10.020},
        {3.0F, 0.0F, 0.0F, 6.0F, 10.040},
        {0.1F, 0.0F, 0.0F, 7.0F, 10.060},
        {4.0F, 0.0F, 0.0F, 8.0F, 12.000},
    });
    sensor_adapter::SeyondAdapter adapter;
    pcl::PointCloud<sensor_adapter::InternalPointType> output;
    double scan_start_time = -1.0;
    double scan_end_time = -1.0;

    ASSERT_TRUE(adapter.convertToFilteredCloud(
        input, output, 1, 0.5, 0.08, scan_start_time, scan_end_time));
    EXPECT_DOUBLE_EQ(scan_start_time, 10.000);
    EXPECT_DOUBLE_EQ(scan_end_time, 10.060);
    ASSERT_EQ(output.size(), 3U);
    EXPECT_FLOAT_EQ(output.points[0].intensity, 4.0F);
    EXPECT_FLOAT_EQ(output.points[2].curvature, 40.0F);
}

TEST(LidarProcessor, ProcessesSeyondTimestampedCloud)
{
    const sensor_msgs::msg::PointCloud2 input = makeSeyondCloud({
        {1.0F, 0.0F, 0.0F, 4.0F, 10.000},
        {2.0F, 0.0F, 0.0F, 5.0F, 10.020},
    });
    LidarProcessor::Config config;
    config.lidar_type = LidarType::kSeyond;
    config.scan_line_count = 1;
    config.scan_rate_hz = 15;
    config.point_filter_stride = 1;
    config.blind_distance = 0.5;
    LidarProcessor processor(config);
    auto output = std::make_shared<PointCloudXYZI>();

    ASSERT_TRUE(processor.process(input, output));
    EXPECT_DOUBLE_EQ(processor.scanStartTime(), 10.000);
    EXPECT_DOUBLE_EQ(processor.scanEndTime(), 10.020);
    ASSERT_EQ(output->size(), 2U);
}
