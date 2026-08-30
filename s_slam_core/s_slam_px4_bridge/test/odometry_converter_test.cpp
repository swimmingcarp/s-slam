#include "s_slam_px4_bridge/odometry_converter.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace s_slam_px4_bridge
{
namespace
{
Eigen::Matrix3d enuToNed()
{
    Eigen::Matrix3d rotation;
    rotation << 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0;
    return rotation;
}

Eigen::Matrix3d fluFromFrd()
{
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0;
    return rotation;
}

s_slam_interfaces::msg::Px4Odometry validOdometry()
{
    s_slam_interfaces::msg::Px4Odometry odometry;
    odometry.header.stamp.sec = 10;
    odometry.header.stamp.nanosec = 500000000U;
    odometry.pose.position.x = 4.0;
    odometry.pose.position.y = 2.0;
    odometry.pose.position.z = 3.0;
    odometry.pose.orientation.w = 1.0;
    odometry.pose_covariance[0] = 1.0;
    odometry.pose_covariance[7] = 4.0;
    odometry.pose_covariance[14] = 9.0;
    odometry.pose_covariance[21] = 0.01;
    odometry.pose_covariance[28] = 0.04;
    odometry.pose_covariance[35] = 0.09;
    odometry.linear_velocity.x = 1.0;
    odometry.linear_velocity.y = 2.0;
    odometry.linear_velocity.z = 3.0;
    odometry.linear_velocity_covariance[0] = 16.0;
    odometry.linear_velocity_covariance[4] = 25.0;
    odometry.linear_velocity_covariance[8] = 36.0;
    return odometry;
}
}  // namespace

TEST(OdometryConverter, ConvertsPoseVelocityAndCovarianceIntoConfiguredFrames)
{
    OdometryConverter converter(enuToNed(), fluFromFrd());

    const std::optional<ConvertedOdometry> converted = converter.convert(validOdometry());

    ASSERT_TRUE(converted.has_value());
    EXPECT_EQ(converted->sample_time_ns, 10500000000LL);
    EXPECT_TRUE(converted->position.isApprox(Eigen::Vector3d(2.0, 4.0, -3.0)));
    EXPECT_TRUE(converted->orientation.toRotationMatrix().isApprox(enuToNed() * fluFromFrd()));
    EXPECT_TRUE(converted->position_variance.isApprox(Eigen::Vector3d(4.0, 1.0, 9.0)));
    EXPECT_TRUE(converted->orientation_variance.isApprox(Eigen::Vector3d(0.01, 0.04, 0.09)));
    EXPECT_TRUE(converted->linear_velocity.isApprox(Eigen::Vector3d(2.0, 1.0, -3.0)));
    EXPECT_TRUE(converted->linear_velocity_variance.isApprox(Eigen::Vector3d(25.0, 16.0, 36.0)));
}

TEST(OdometryConverter, RejectsInvalidInputAndFrameRotation)
{
    EXPECT_THROW(OdometryConverter(Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity()),
                 std::invalid_argument);

    OdometryConverter converter(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    s_slam_interfaces::msg::Px4Odometry odometry = validOdometry();
    odometry.pose.orientation.w = 0.0;
    EXPECT_FALSE(converter.convert(odometry).has_value());
}

TEST(OdometryConverter, ConvertsMapFrameAttitudeCovarianceIntoBodyFrame)
{
    OdometryConverter converter(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    s_slam_interfaces::msg::Px4Odometry odometry = validOdometry();
    odometry.pose.orientation.z = std::sqrt(0.5);
    odometry.pose.orientation.w = std::sqrt(0.5);
    odometry.pose_covariance[21] = 1.0;
    odometry.pose_covariance[28] = 4.0;
    odometry.pose_covariance[35] = 9.0;

    const std::optional<ConvertedOdometry> converted = converter.convert(odometry);

    ASSERT_TRUE(converted.has_value());
    EXPECT_TRUE(converted->orientation_variance.isApprox(Eigen::Vector3d(4.0, 1.0, 9.0)));
}

TEST(OdometryConverter, RejectsInvalidCovariance)
{
    OdometryConverter converter(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    s_slam_interfaces::msg::Px4Odometry odometry = validOdometry();
    odometry.linear_velocity_covariance[0] = -0.1;

    EXPECT_FALSE(converter.convert(odometry).has_value());
}

TEST(VehicleOdometry, CarriesTheConvertedStateAndResetMetadata)
{
    ConvertedOdometry converted;
    converted.sample_time_ns = 1234567000LL;
    converted.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    converted.orientation = Eigen::Quaterniond::Identity();
    converted.position_variance = Eigen::Vector3d(0.1, 0.2, 0.3);
    converted.orientation_variance = Eigen::Vector3d(0.01, 0.02, 0.03);
    converted.linear_velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
    converted.linear_velocity_variance = Eigen::Vector3d(0.4, 0.5, 0.6);

    const px4_msgs::msg::VehicleOdometry vehicle_odometry =
        toVehicleOdometry(converted, 7654321U, 9U);

    EXPECT_EQ(vehicle_odometry.timestamp, 7654321U);
    EXPECT_EQ(vehicle_odometry.timestamp_sample, 1234567U);
    EXPECT_EQ(vehicle_odometry.pose_frame, px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD);
    EXPECT_EQ(vehicle_odometry.velocity_frame,
              px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD);
    EXPECT_EQ(vehicle_odometry.position[0], 1.0F);
    EXPECT_EQ(vehicle_odometry.velocity[2], 6.0F);
    EXPECT_EQ(vehicle_odometry.velocity_variance[1], 0.5F);
    EXPECT_TRUE(std::isnan(vehicle_odometry.angular_velocity[0]));
    EXPECT_EQ(vehicle_odometry.reset_counter, 9U);
    EXPECT_EQ(vehicle_odometry.quality, 0);
}

TEST(SampleRateSelector, SelectsNearestSamplesAtTheConfiguredRate)
{
    SampleRateSelector selector;
    std::vector<std::int64_t> selected;
    std::int64_t previous_sample_time = 100;

    for (const std::int64_t sample_time : {100LL, 103LL, 106LL, 109LL, 112LL, 115LL, 118LL,
                                            121LL, 124LL, 127LL, 130LL})
    {
        const SampleRateSelector::Selection selection = selector.select(sample_time, 10);
        if (selection == SampleRateSelector::Selection::kCurrent)
        {
            selected.push_back(sample_time);
        }
        else if (selection == SampleRateSelector::Selection::kPrevious)
        {
            selected.push_back(previous_sample_time);
        }
        previous_sample_time = sample_time;
    }

    EXPECT_EQ(selected, (std::vector<std::int64_t>{100, 109, 121, 130}));
}

TEST(SampleRateSelector, DropsOutOfOrderSamplesAndDoesNotRepeatAfterAGap)
{
    SampleRateSelector selector;

    EXPECT_EQ(selector.select(100, 10), SampleRateSelector::Selection::kCurrent);
    EXPECT_EQ(selector.select(110, 10), SampleRateSelector::Selection::kCurrent);
    EXPECT_EQ(selector.select(105, 10), SampleRateSelector::Selection::kNone);
    EXPECT_EQ(selector.select(145, 10), SampleRateSelector::Selection::kCurrent);

    selector.reset();
    EXPECT_EQ(selector.select(90, 10), SampleRateSelector::Selection::kCurrent);
}

}  // namespace s_slam_px4_bridge
