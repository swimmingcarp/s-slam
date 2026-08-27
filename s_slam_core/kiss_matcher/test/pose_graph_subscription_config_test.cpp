#include <stdexcept>

#include <gtest/gtest.h>

#include "slam/input_config.hpp"

namespace
{
TEST(PoseGraphSubscriptionConfig, CreatesVolatileInputQos)
{
    const auto odom_qos = kiss_matcher::makeInputQos(
        23, kiss_matcher::parseInputReliability("best_effort", "input.odom_qos_reliability"),
        "input.odom_qos_depth");
    const auto &odom_profile = odom_qos.get_rmw_qos_profile();
    EXPECT_EQ(odom_profile.depth, 23U);
    EXPECT_EQ(odom_profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    EXPECT_EQ(odom_profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto cloud_qos = kiss_matcher::makeInputQos(
        41, kiss_matcher::parseInputReliability("reliable", "input.cloud_qos_reliability"),
        "input.cloud_qos_depth");
    const auto &cloud_profile = cloud_qos.get_rmw_qos_profile();
    EXPECT_EQ(cloud_profile.depth, 41U);
    EXPECT_EQ(cloud_profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(cloud_profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    EXPECT_EQ(kiss_matcher::inputQueueSize(17, "input.sync_queue_size"), 17U);
}

TEST(PoseGraphSubscriptionConfig, RejectsInvalidInputQosParameters)
{
    EXPECT_THROW(kiss_matcher::makeInputQos(
                     0, rclcpp::ReliabilityPolicy::Reliable, "input.odom_qos_depth"),
                 std::invalid_argument);
    EXPECT_THROW(kiss_matcher::parseInputReliability("lossless", "input.cloud_qos_reliability"),
                 std::invalid_argument);
    EXPECT_THROW(kiss_matcher::inputQueueSize(0, "input.sync_queue_size"), std::invalid_argument);
}
}  // namespace
