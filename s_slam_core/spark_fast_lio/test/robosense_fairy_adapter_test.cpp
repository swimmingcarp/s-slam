#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

#include "adapters/robosense_fairy_adapter.hpp"
#include "data_processors/lidar_processor.hpp"

namespace
{
struct XYZIRTPoint
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    std::uint16_t ring = 0;
    double timestamp = 0.0;
};

template <typename T>
void writeField(std::vector<std::uint8_t> &data,
                const std::size_t offset,
                const T value)
{
    std::memcpy(data.data() + offset, &value, sizeof(value));
}

sensor_msgs::msg::PointField makeField(const std::string &name,
                                       const std::uint32_t offset,
                                       const std::uint8_t datatype)
{
    sensor_msgs::msg::PointField field;
    field.name     = name;
    field.offset   = offset;
    field.datatype = datatype;
    field.count    = 1;
    return field;
}

sensor_msgs::msg::PointCloud2 makeXYZIRTCloud(const std::vector<XYZIRTPoint> &points)
{
    using sensor_msgs::msg::PointField;
    constexpr std::uint32_t kPointStep = 32;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height       = 1;
    cloud.width        = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.point_step   = kPointStep;
    cloud.row_step     = cloud.width * cloud.point_step;
    cloud.fields = {
        makeField("x", 0, PointField::FLOAT32),
        makeField("y", 4, PointField::FLOAT32),
        makeField("z", 8, PointField::FLOAT32),
        makeField("intensity", 16, PointField::FLOAT32),
        makeField("ring", 20, PointField::UINT16),
        makeField("timestamp", 24, PointField::FLOAT64),
    };
    cloud.data.resize(cloud.row_step);

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const std::size_t offset = index * kPointStep;
        const auto &point        = points[index];
        writeField(cloud.data, offset, point.x);
        writeField(cloud.data, offset + 4, point.y);
        writeField(cloud.data, offset + 8, point.z);
        writeField(cloud.data, offset + 16, point.intensity);
        writeField(cloud.data, offset + 20, point.ring);
        writeField(cloud.data, offset + 24, point.timestamp);
    }
    return cloud;
}
}  // namespace

TEST(RoboSenseFairyAdapter, PreservesXYZIRTFilteringAndFeatureInputSemantics)
{
    const sensor_msgs::msg::PointCloud2 input = makeXYZIRTCloud({
        {1.0F, 0.0F, 0.0F, 10.0F, 0, 10.002},
        {1.0F, 1.0F, 0.0F, 11.0F, 0, 10.000},
        {2.0F, 0.0F, 0.0F, 12.0F, 1, 10.004},
        {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 13.0F, 0, 10.006},
        {4.0F, 0.0F, 0.0F, 14.0F, 0, 10.009},
        {0.1F, 0.0F, 0.0F, 15.0F, 0, 10.011},
    });
    sensor_adapter::RoboSenseFairyAdapter adapter;

    sensor_adapter::InternalScan scan;
    ASSERT_TRUE(adapter.convert(input, 0.12, scan));
    EXPECT_DOUBLE_EQ(scan.start_time, 10.000);
    EXPECT_DOUBLE_EQ(scan.end_time, 10.011);
    ASSERT_EQ(scan.points.size(), 5U);
    EXPECT_EQ(scan.points[0].source_index, 0U);
    EXPECT_EQ(scan.points[2].source_index, 2U);
    EXPECT_EQ(scan.points[4].source_index, 5U);
    EXPECT_FLOAT_EQ(scan.points[0].point.curvature, 2.0F);
    EXPECT_FLOAT_EQ(scan.points[3].point.curvature, 9.0F);

    pcl::PointCloud<sensor_adapter::InternalPointType> filtered_cloud;
    double scan_start_time = -1.0;
    double scan_end_time   = -1.0;
    ASSERT_TRUE(adapter.convertToFilteredCloud(
        input, filtered_cloud, 1, 2, 0.5, 0.12, scan_start_time, scan_end_time));
    EXPECT_DOUBLE_EQ(scan_start_time, 10.000);
    EXPECT_DOUBLE_EQ(scan_end_time, 10.011);
    ASSERT_EQ(filtered_cloud.size(), 2U);
    EXPECT_FLOAT_EQ(filtered_cloud.points[0].x, 1.0F);
    EXPECT_FLOAT_EQ(filtered_cloud.points[0].intensity, 10.0F);
    EXPECT_FLOAT_EQ(filtered_cloud.points[0].curvature, 2.0F);
    EXPECT_FLOAT_EQ(filtered_cloud.points[1].x, 4.0F);
    EXPECT_FLOAT_EQ(filtered_cloud.points[1].intensity, 14.0F);
    EXPECT_FLOAT_EQ(filtered_cloud.points[1].curvature, 9.0F);
}

TEST(RoboSenseFairyAdapter, IgnoresSingleTimestampOutlier)
{
    const sensor_msgs::msg::PointCloud2 input = makeXYZIRTCloud({
        {1.0F, 0.0F, 0.0F, 10.0F, 0, 10.000},
        {2.0F, 0.0F, 0.0F, 11.0F, 0, 10.025},
        {3.0F, 0.0F, 0.0F, 12.0F, 0, 10.050},
        {4.0F, 0.0F, 0.0F, 13.0F, 0, 10.090},
        {5.0F, 0.0F, 0.0F, 14.0F, 0, 12.000},
    });
    sensor_adapter::RoboSenseFairyAdapter adapter;

    sensor_adapter::InternalScan scan;
    ASSERT_TRUE(adapter.convert(input, 0.12, scan));
    EXPECT_DOUBLE_EQ(scan.start_time, 10.000);
    EXPECT_DOUBLE_EQ(scan.end_time, 10.090);
    ASSERT_EQ(scan.points.size(), 4U);
    EXPECT_EQ(scan.points.back().source_index, 3U);
    EXPECT_FLOAT_EQ(scan.points.back().point.curvature, 90.0F);

    pcl::PointCloud<sensor_adapter::InternalPointType> filtered_cloud;
    double scan_start_time = -1.0;
    double scan_end_time   = -1.0;
    ASSERT_TRUE(adapter.convertToFilteredCloud(
        input, filtered_cloud, 1, 1, 0.5, 0.12, scan_start_time, scan_end_time));
    EXPECT_DOUBLE_EQ(scan_start_time, 10.000);
    EXPECT_DOUBLE_EQ(scan_end_time, 10.090);
    ASSERT_EQ(filtered_cloud.size(), 4U);
    EXPECT_FLOAT_EQ(filtered_cloud.points.back().x, 4.0F);
    EXPECT_FLOAT_EQ(filtered_cloud.points.back().curvature, 90.0F);
}

TEST(LidarProcessor, FiltersRoboSenseTimestampOutlierBeforeSynchronization)
{
    const sensor_msgs::msg::PointCloud2 input = makeXYZIRTCloud({
        {1.0F, 0.0F, 0.0F, 10.0F, 0, 10.000},
        {2.0F, 0.0F, 0.0F, 11.0F, 0, 10.025},
        {3.0F, 0.0F, 0.0F, 12.0F, 0, 10.050},
        {4.0F, 0.0F, 0.0F, 13.0F, 0, 10.090},
        {5.0F, 0.0F, 0.0F, 14.0F, 0, 12.000},
    });
    LidarProcessor::Config config;
    config.lidar_type          = LidarType::kRoboSense;
    config.scan_line_count     = 1;
    config.scan_rate_hz        = 10;
    config.point_filter_stride = 1;
    config.blind_distance      = 0.5;
    LidarProcessor processor(config);
    auto output = std::make_shared<PointCloudXYZI>();

    ASSERT_TRUE(processor.process(input, output));
    EXPECT_DOUBLE_EQ(processor.scanStartTime(), 10.000);
    EXPECT_DOUBLE_EQ(processor.scanEndTime(), 10.090);
    ASSERT_EQ(output->size(), 4U);
    EXPECT_FLOAT_EQ(output->points.back().x, 4.0F);
    EXPECT_FLOAT_EQ(output->points.back().curvature, 90.0F);
}

TEST(RoboSenseFairyAdapter, RejectsAmbiguousTimestampWindows)
{
    const sensor_msgs::msg::PointCloud2 input = makeXYZIRTCloud({
        {1.0F, 0.0F, 0.0F, 10.0F, 0, 10.000},
        {2.0F, 0.0F, 0.0F, 11.0F, 0, 10.025},
        {3.0F, 0.0F, 0.0F, 12.0F, 0, 12.000},
        {4.0F, 0.0F, 0.0F, 13.0F, 0, 12.025},
    });
    sensor_adapter::RoboSenseFairyAdapter adapter;

    sensor_adapter::InternalScan scan;
    EXPECT_FALSE(adapter.convert(input, 0.12, scan));
    EXPECT_TRUE(scan.points.empty());
    EXPECT_DOUBLE_EQ(scan.start_time, -1.0);
    EXPECT_DOUBLE_EQ(scan.end_time, -1.0);

    pcl::PointCloud<sensor_adapter::InternalPointType> filtered_cloud;
    double scan_start_time = -1.0;
    double scan_end_time   = -1.0;
    EXPECT_FALSE(adapter.convertToFilteredCloud(
        input, filtered_cloud, 1, 1, 0.5, 0.12, scan_start_time, scan_end_time));
    EXPECT_TRUE(filtered_cloud.empty());
    EXPECT_DOUBLE_EQ(scan_start_time, -1.0);
    EXPECT_DOUBLE_EQ(scan_end_time, -1.0);
}
