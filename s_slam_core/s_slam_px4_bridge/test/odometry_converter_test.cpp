#include "s_slam_px4_bridge/odometry_converter.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

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

Eigen::Matrix3d shippedFlipYZ()
{
    // Must stay equal to both rotations in config/px4_odometry_bridge.yaml.
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
    odometry.angular_velocity.x = 0.1;
    odometry.angular_velocity.y = 0.2;
    odometry.angular_velocity.z = 0.3;
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
    EXPECT_TRUE(converted->angular_velocity.isApprox(Eigen::Vector3d(0.1, -0.2, -0.3)));
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
    converted.angular_velocity = Eigen::Vector3d(0.7, 0.8, 0.9);

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
    EXPECT_FLOAT_EQ(vehicle_odometry.angular_velocity[0], 0.7F);
    EXPECT_EQ(vehicle_odometry.reset_counter, 9U);
    EXPECT_EQ(vehicle_odometry.quality, 0);
}

TEST(PropagateOdometry, AdvancesPoseAndSampleTimeByTheHoldInterval)
{
    ConvertedOdometry odometry;
    odometry.sample_time_ns = 1000000000LL;
    odometry.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    odometry.orientation = Eigen::Quaterniond::Identity();
    odometry.linear_velocity = Eigen::Vector3d(10.0, 0.0, -4.0);
    odometry.angular_velocity = Eigen::Vector3d(0.0, 0.0, M_PI);

    const ConvertedOdometry propagated = propagateOdometry(odometry, 100000000LL);

    EXPECT_EQ(propagated.sample_time_ns, 1100000000LL);
    EXPECT_TRUE(propagated.position.isApprox(Eigen::Vector3d(2.0, 2.0, 2.6)));
    EXPECT_NEAR(propagated.orientation.angularDistance(
                    Eigen::Quaterniond(Eigen::AngleAxisd(0.1 * M_PI, Eigen::Vector3d::UnitZ()))),
                0.0,
                1e-12);
    EXPECT_NEAR(propagated.orientation.norm(), 1.0, 1e-12);
    // Everything the source measured directly is carried through unchanged.
    EXPECT_TRUE(propagated.linear_velocity.isApprox(odometry.linear_velocity));
    EXPECT_TRUE(propagated.angular_velocity.isApprox(odometry.angular_velocity));
}

TEST(PropagateOdometry, LeavesAStateUntouchedWithoutAHoldInterval)
{
    ConvertedOdometry odometry;
    odometry.sample_time_ns = 1000000000LL;
    odometry.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    odometry.linear_velocity = Eigen::Vector3d(10.0, 0.0, -4.0);
    odometry.angular_velocity = Eigen::Vector3d(0.0, 0.0, M_PI);

    const ConvertedOdometry propagated = propagateOdometry(odometry, 0);

    EXPECT_EQ(propagated.sample_time_ns, odometry.sample_time_ns);
    EXPECT_TRUE(propagated.position.isApprox(odometry.position));
    EXPECT_NEAR(propagated.orientation.angularDistance(odometry.orientation), 0.0, 1e-12);
}

// The shipped configuration is the only rotation pair that reaches a flight
// controller, and nothing else in this suite loads it. Identity there keeps
// every field well formed while inverting height, so pin the shipped values
// and the sign map they produce.
TEST(ShippedFrameRotations, SendAClimbToPX4AsAClimb)
{
    const OdometryConverter converter(shippedFlipYZ(), shippedFlipYZ());

    s_slam_interfaces::msg::Px4Odometry odometry = validOdometry();
    // A FAST-LIO base pose in a z-up map: rising, drifting left, yawing left.
    odometry.pose.position.x = 2.0;
    odometry.pose.position.y = 3.0;
    odometry.pose.position.z = 5.0;
    odometry.linear_velocity.x = 0.5;
    odometry.linear_velocity.y = 0.25;
    odometry.linear_velocity.z = 1.5;
    odometry.angular_velocity.x = 0.1;
    odometry.angular_velocity.y = 0.2;
    odometry.angular_velocity.z = 0.3;

    const std::optional<ConvertedOdometry> converted = converter.convert(odometry);
    ASSERT_TRUE(converted.has_value());

    // Forward is shared; left becomes right and up becomes down.
    EXPECT_DOUBLE_EQ(converted->position.x(), 2.0);
    EXPECT_DOUBLE_EQ(converted->position.y(), -3.0);
    EXPECT_DOUBLE_EQ(converted->position.z(), -5.0);
    EXPECT_DOUBLE_EQ(converted->linear_velocity.x(), 0.5);
    EXPECT_DOUBLE_EQ(converted->linear_velocity.y(), -0.25);
    EXPECT_DOUBLE_EQ(converted->linear_velocity.z(), -1.5);
    EXPECT_DOUBLE_EQ(converted->angular_velocity.x(), 0.1);
    EXPECT_DOUBLE_EQ(converted->angular_velocity.y(), -0.2);
    EXPECT_DOUBLE_EQ(converted->angular_velocity.z(), -0.3);

    // A sign flip is a diagonal similarity, so the reported variances stay on
    // their own axis and keep their value.
    EXPECT_DOUBLE_EQ(converted->position_variance.x(), 1.0);
    EXPECT_DOUBLE_EQ(converted->position_variance.y(), 4.0);
    EXPECT_DOUBLE_EQ(converted->position_variance.z(), 9.0);
    EXPECT_DOUBLE_EQ(converted->linear_velocity_variance.x(), 16.0);
    EXPECT_DOUBLE_EQ(converted->linear_velocity_variance.y(), 25.0);
    EXPECT_DOUBLE_EQ(converted->linear_velocity_variance.z(), 36.0);
}

TEST(ShippedFrameRotations, SeparateAClimbFromTheIdentityPlaceholder)
{
    const OdometryConverter shipped(shippedFlipYZ(), shippedFlipYZ());
    const OdometryConverter identity(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());

    s_slam_interfaces::msg::Px4Odometry climbing = validOdometry();
    climbing.pose.position.z = 1.29;

    const std::optional<ConvertedOdometry> shipped_climb = shipped.convert(climbing);
    const std::optional<ConvertedOdometry> identity_climb = identity.convert(climbing);
    ASSERT_TRUE(shipped_climb.has_value());
    ASSERT_TRUE(identity_climb.has_value());

    // PX4 reads +z as down, so only the shipped pair reports the climb as one.
    EXPECT_LT(shipped_climb->position.z(), 0.0);
    EXPECT_GT(identity_climb->position.z(), 0.0);
}

TEST(ShippedFrameRotations, AreProperRotations)
{
    EXPECT_TRUE(OdometryConverter::isProperRotation(shippedFlipYZ()));
}

// Everything above pins the conversion. This pins the value the vehicle is
// actually shipped, so editing the configuration back towards identity fails
// here rather than in flight.
TEST(ShippedFrameRotations, MatchTheCheckedInConfiguration)
{
    const YAML::Node parameters = YAML::LoadFile(SHIPPED_BRIDGE_CONFIG)["/**"]["ros__parameters"];
    ASSERT_TRUE(parameters) << "cannot read " << SHIPPED_BRIDGE_CONFIG;

    const Eigen::Matrix3d expected = shippedFlipYZ();
    for (const char *name : {"px4_world_from_source_world", "source_body_from_px4_body"})
    {
        const std::vector<double> values = parameters[name].as<std::vector<double>>();
        ASSERT_EQ(values.size(), 9U) << name;
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                EXPECT_DOUBLE_EQ(values[row * 3 + column], expected(row, column))
                    << name << " element [" << row << "][" << column << "]";
            }
        }
    }
}

}  // namespace s_slam_px4_bridge
