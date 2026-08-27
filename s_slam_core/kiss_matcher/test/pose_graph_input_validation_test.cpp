#include <limits>

#include <gtest/gtest.h>

#include <pcl_conversions/pcl_conversions.h>

#include "slam/pose_graph_node.hpp"

namespace kiss_matcher
{
namespace
{
nav_msgs::msg::Odometry validOdometry()
{
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp.sec = 10;
    odom.pose.pose.orientation.w = 1.0;
    return odom;
}

pcl::PointCloud<PointType> validCloud()
{
    pcl::PointCloud<PointType> scan;
    scan.emplace_back(1.0F, 2.0F, 3.0F);
    return scan;
}

sensor_msgs::msg::PointCloud2 validCloudMessage()
{
    sensor_msgs::msg::PointCloud2 scan;
    pcl::toROSMsg(validCloud(), scan);
    scan.header.frame_id = "odom";
    scan.header.stamp.sec = 10;
    return scan;
}
}  // namespace

TEST(PoseGraphInputValidation, AcceptsSynchronizedFiniteInput)
{
    EXPECT_TRUE(PoseGraphNode::hasValidInput(
        validOdometry(), validCloudMessage(), validCloud(), 0.05));
}

TEST(PoseGraphInputValidation, RejectsInvalidPoseFramePointAndTimestamp)
{
    auto odom = validOdometry();
    auto scan_message = validCloudMessage();
    auto scan = validCloud();

    odom.pose.pose.orientation.w = 0.0;
    EXPECT_FALSE(PoseGraphNode::hasValidInput(odom, scan_message, scan, 0.05));

    odom = validOdometry();
    scan_message.header.frame_id = "unexpected_frame";
    EXPECT_FALSE(PoseGraphNode::hasValidInput(odom, scan_message, scan, 0.05));

    scan_message = validCloudMessage();
    scan_message.header.stamp.sec += 1;
    EXPECT_FALSE(PoseGraphNode::hasValidInput(odom, scan_message, scan, 0.05));

    scan_message = validCloudMessage();
    scan.points.front().x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(PoseGraphNode::hasValidInput(odom, scan_message, scan, 0.05));

    scan_message = validCloudMessage();
    scan_message.fields.clear();
    EXPECT_FALSE(PoseGraphNode::hasValidMessage(odom, scan_message, 0.05));

    scan_message = validCloudMessage();
    scan_message.data.pop_back();
    EXPECT_FALSE(PoseGraphNode::hasValidMessage(odom, scan_message, 0.05));

    scan_message = validCloudMessage();
    scan_message.data.push_back(0U);
    EXPECT_FALSE(PoseGraphNode::hasValidMessage(odom, scan_message, 0.05));

    scan_message = validCloudMessage();
    auto x_field = std::find_if(scan_message.fields.begin(),
                                scan_message.fields.end(),
                                [](const sensor_msgs::msg::PointField &field)
                                { return field.name == "x"; });
    ASSERT_NE(x_field, scan_message.fields.end());
    x_field->datatype = sensor_msgs::msg::PointField::UINT32;
    EXPECT_FALSE(PoseGraphNode::hasValidMessage(odom, scan_message, 0.05));

    scan_message = validCloudMessage();
    scan_message.is_bigendian = true;
    EXPECT_FALSE(PoseGraphNode::hasValidMessage(odom, scan_message, 0.05));

    odom = validOdometry();
    odom.pose.pose.orientation.w = std::numeric_limits<double>::max();
    EXPECT_FALSE(PoseGraphNode::hasValidMessage(odom, validCloudMessage(), 0.05));
}

TEST(PoseGraphInputValidation, NormalizesAcceptedQuaternion)
{
    auto odom = validOdometry();
    odom.pose.pose.orientation.w = 2.0;

    PoseGraphNode node(odom, validCloud(), 0, 0.3);

    EXPECT_NEAR(node.pose_(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(node.pose_(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(node.pose_(2, 2), 1.0, 1e-12);
}

TEST(PoseGraphInputValidation, PreservesPoseStampedTimestamp)
{
    auto odom = validOdometry();
    odom.header.stamp.sec = 123;
    odom.header.stamp.nanosec = 456789012U;
    const PoseGraphNode node(odom, validCloud(), 0, 0.3);
    const rclcpp::Time timestamp(node.timestamp_);

    EXPECT_EQ(node.timestamp_.sec, 123);
    EXPECT_EQ(node.timestamp_.nanosec, 456789012U);

    const auto eigen_pose = kiss_matcher::eigenToPoseStamped(node.pose_, "map", timestamp);
    EXPECT_EQ(eigen_pose.header.stamp.sec, 123);
    EXPECT_EQ(eigen_pose.header.stamp.nanosec, 456789012U);

    const gtsam::Pose3 gtsam_pose(gtsam::Rot3::Identity(), gtsam::Point3(1.0, 2.0, 3.0));
    const auto gtsam_pose_stamped =
        kiss_matcher::gtsamToPoseStamped(gtsam_pose, "map", timestamp);
    EXPECT_EQ(gtsam_pose_stamped.header.stamp.sec, 123);
    EXPECT_EQ(gtsam_pose_stamped.header.stamp.nanosec, 456789012U);
}
}  // namespace kiss_matcher
