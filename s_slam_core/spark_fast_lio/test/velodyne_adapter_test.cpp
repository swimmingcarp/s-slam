#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

#include "adapters/velodyne_adapter.hpp"
#include "data_processors/lidar_processor.hpp"

namespace
{
struct VelodynePoint
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    float time = 0.0F;
    std::uint16_t ring = 0;
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
    field.name     = name;
    field.offset   = offset;
    field.datatype = datatype;
    field.count    = 1;
    return field;
}

sensor_msgs::msg::PointCloud2 makeVelodyneCloud(const std::vector<VelodynePoint> &points)
{
    using sensor_msgs::msg::PointField;
    constexpr std::uint32_t kPointStep = 24;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp.sec     = 1000;
    cloud.header.stamp.nanosec = 500000000U;
    cloud.height               = 1;
    cloud.width                = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian         = false;
    cloud.point_step           = kPointStep;
    cloud.row_step             = cloud.width * cloud.point_step;
    cloud.fields = {
        makeField("x", 0, PointField::FLOAT32),
        makeField("y", 4, PointField::FLOAT32),
        makeField("z", 8, PointField::FLOAT32),
        makeField("intensity", 12, PointField::FLOAT32),
        makeField("time", 16, PointField::FLOAT32),
        makeField("ring", 20, PointField::UINT16),
    };
    cloud.data.resize(cloud.row_step);

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const std::size_t offset = index * kPointStep;
        const auto &point        = points[index];
        writeField(cloud.data, offset, point.x);
        writeField(cloud.data, offset + 4, point.y);
        writeField(cloud.data, offset + 8, point.z);
        writeField(cloud.data, offset + 12, point.intensity);
        writeField(cloud.data, offset + 16, point.time);
        writeField(cloud.data, offset + 20, point.ring);
    }
    return cloud;
}

LidarProcessor makeVelodyneProcessor()
{
    LidarProcessor::Config config;
    config.lidar_type          = LidarType::kVelo16;
    config.timestamp_unit      = TimestampUnit::kSeconds;
    config.scan_line_count     = 16;
    config.scan_rate_hz        = 10;
    config.point_filter_stride = 1;
    config.blind_distance      = 0.1;
    return LidarProcessor(config);
}
}  // namespace

TEST(VelodyneAdapter, NormalizesEndReferencedPointTimesAndScanWindow)
{
    const auto input = makeVelodyneCloud({
        {1.0F, 0.0F, 0.0F, 1.0F, -0.097867012F, 0},
        {2.0F, 0.0F, 0.0F, 2.0F, -0.050000000F, 0},
        {3.0F, 0.0F, 0.0F, 3.0F, 0.001306368F, 0},
    });
    sensor_adapter::VelodyneAdapter adapter;
    sensor_adapter::InternalScan scan;
    bool has_offset_time = false;

    ASSERT_TRUE(adapter.convert(input, 16, 10, 1000.0F, has_offset_time, scan));
    ASSERT_TRUE(has_offset_time);
    ASSERT_EQ(scan.points.size(), 3U);
    EXPECT_NEAR(scan.start_time, 1000.402132988, 1.0e-6);
    EXPECT_NEAR(scan.end_time, 1000.501306368, 1.0e-6);
    EXPECT_FLOAT_EQ(scan.points[0].point.curvature, 0.0F);
    EXPECT_NEAR(scan.points[1].point.curvature, 47.867012F, 1.0e-4F);
    EXPECT_NEAR(scan.points[2].point.curvature, 99.17338F, 1.0e-4F);
}

TEST(LidarProcessor, PreservesVelodyneEndReferencedScanTimesForSynchronization)
{
    const auto input = makeVelodyneCloud({
        {1.0F, 0.0F, 0.0F, 1.0F, -0.097867012F, 0},
        {2.0F, 0.0F, 0.0F, 2.0F, -0.050000000F, 0},
        {3.0F, 0.0F, 0.0F, 3.0F, 0.001306368F, 0},
    });
    auto processor = makeVelodyneProcessor();
    auto output = std::make_shared<PointCloudXYZI>();

    ASSERT_TRUE(processor.process(input, output));
    ASSERT_TRUE(processor.hasScanTime());
    EXPECT_NEAR(processor.scanStartTime(), 1000.402132988, 1.0e-6);
    EXPECT_NEAR(processor.scanEndTime(), 1000.501306368, 1.0e-6);
    ASSERT_EQ(output->size(), 3U);
    EXPECT_FLOAT_EQ(output->front().curvature, 0.0F);
    EXPECT_NEAR(output->back().curvature, 99.17338F, 1.0e-4F);
}

TEST(VelodyneAdapter, PreservesHeaderReferencedAndMissingPointTimeConventions)
{
    sensor_adapter::VelodyneAdapter adapter;
    sensor_adapter::InternalScan scan;
    bool has_offset_time = false;

    const auto header_referenced = makeVelodyneCloud({
        {1.0F, 0.0F, 0.0F, 1.0F, 0.000000F, 0},
        {2.0F, 0.0F, 0.0F, 2.0F, 0.050000F, 0},
        {3.0F, 0.0F, 0.0F, 3.0F, 0.100000F, 0},
    });
    ASSERT_TRUE(adapter.convert(header_referenced, 16, 10, 1000.0F, has_offset_time, scan));
    ASSERT_TRUE(has_offset_time);
    EXPECT_NEAR(scan.start_time, 1000.5, 1.0e-6);
    EXPECT_NEAR(scan.end_time, 1000.6, 1.0e-6);
    EXPECT_FLOAT_EQ(scan.points.front().point.curvature, 0.0F);
    EXPECT_FLOAT_EQ(scan.points.back().point.curvature, 100.0F);

    const auto no_point_time = makeVelodyneCloud({
        {1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0},
        {0.0F, 1.0F, 0.0F, 2.0F, 0.0F, 0},
    });
    ASSERT_TRUE(adapter.convert(no_point_time, 16, 10, 1000.0F, has_offset_time, scan));
    EXPECT_FALSE(has_offset_time);
    EXPECT_DOUBLE_EQ(scan.start_time, -1.0);
    EXPECT_DOUBLE_EQ(scan.end_time, -1.0);
}
