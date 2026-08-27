#pragma once

#ifndef KISS_MATCHER_POSE_GRAPH_NODE_HPP
#define KISS_MATCHER_POSE_GRAPH_NODE_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <pcl/common/point_tests.h>

#include "slam/utils.hpp"

namespace kiss_matcher
{
struct PoseGraphNode
{
    pcl::PointCloud<PointType> scan_;
    pcl::PointCloud<PointType> voxelized_scan_; // Used for map visualization
    Eigen::Matrix4d pose_           = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_ = Eigen::Matrix4d::Identity();
    builtin_interfaces::msg::Time timestamp_;
    size_t idx_;
    bool nnsearch_processed_      = false;
    bool loop_detector_processed_ = false;

    PoseGraphNode()
    {
    }

    static bool hasValidMessage(const nav_msgs::msg::Odometry &odom,
                                const sensor_msgs::msg::PointCloud2 &scan_msg,
                                const double max_sync_interval)
    {
        if (max_sync_interval < 0.0 || odom.header.frame_id.empty() ||
            scan_msg.header.frame_id.empty() || odom.header.frame_id != scan_msg.header.frame_id)
        {
            return false;
        }

        const auto &position = odom.pose.pose.position;
        const auto &orientation = odom.pose.pose.orientation;
        const Eigen::Vector3d translation(position.x, position.y, position.z);
        const Eigen::Quaterniond rotation(
            orientation.w, orientation.x, orientation.y, orientation.z);
        const double rotation_squared_norm = rotation.squaredNorm();
        if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
            !std::isfinite(rotation_squared_norm) ||
            rotation_squared_norm <= std::numeric_limits<double>::epsilon())
        {
            return false;
        }

        const double stamp_delta = std::abs(
            (rclcpp::Time(odom.header.stamp) - rclcpp::Time(scan_msg.header.stamp)).seconds());
        if (stamp_delta > max_sync_interval)
        {
            return false;
        }

        if (scan_msg.is_bigendian || scan_msg.width == 0 || scan_msg.height == 0 ||
            scan_msg.point_step < sizeof(float))
        {
            return false;
        }

        const size_t width      = scan_msg.width;
        const size_t point_step = scan_msg.point_step;
        if (width > std::numeric_limits<size_t>::max() / point_step)
        {
            return false;
        }
        const size_t required_row_size = width * point_step;
        if (required_row_size > scan_msg.row_step)
        {
            return false;
        }

        const size_t height   = scan_msg.height;
        const size_t row_step = scan_msg.row_step;
        if (height > std::numeric_limits<size_t>::max() / row_step)
        {
            return false;
        }
        const size_t required_data_size = height * row_step;
        if (required_data_size != scan_msg.data.size())
        {
            return false;
        }

        const auto has_coordinate_field = [&scan_msg](const std::string &field_name)
        {
            return std::any_of(
                scan_msg.fields.begin(),
                scan_msg.fields.end(),
                [&field_name, &scan_msg](const sensor_msgs::msg::PointField &field)
                {
                    return field.name == field_name &&
                           field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1 &&
                           field.offset <= scan_msg.point_step - sizeof(float);
                });
        };
        return has_coordinate_field("x") && has_coordinate_field("y") && has_coordinate_field("z");
    }

    static bool hasValidInput(const nav_msgs::msg::Odometry &odom,
                              const sensor_msgs::msg::PointCloud2 &scan_msg,
                              const pcl::PointCloud<PointType> &scan,
                              const double max_sync_interval)
    {
        if (!hasValidMessage(odom, scan_msg, max_sync_interval) || scan.empty())
        {
            return false;
        }

        return std::all_of(scan.begin(), scan.end(), [](const PointType &point)
                           { return pcl::isFinite(point); });
    }

    inline PoseGraphNode(const nav_msgs::msg::Odometry &odom,
                         pcl::PointCloud<PointType> scan,
                         const size_t idx,
                         const double voxel_size,
                         const bool store_voxelized_scan = false,
                         const bool is_wrt_lidar_frame   = false)
    {
        tf2::Quaternion q;
        q.setX(odom.pose.pose.orientation.x);
        q.setY(odom.pose.pose.orientation.y);
        q.setZ(odom.pose.pose.orientation.z);
        q.setW(odom.pose.pose.orientation.w);
        q.normalize();

        tf2::Matrix3x3 rot_tf(q);
        Eigen::Matrix3d rot;
        matrixTF2ToEigen(rot_tf, rot);

        pose_.block<3, 3>(0, 0) = rot;
        pose_(0, 3)             = odom.pose.pose.position.x;
        pose_(1, 3)             = odom.pose.pose.position.y;
        pose_(2, 3)             = odom.pose.pose.position.z;

        // This will be updated after pose graph optimization
        pose_corrected_ = pose_;

        if (store_voxelized_scan)
        {
            scan = *voxelize(scan, voxel_size);
        }
        if (is_wrt_lidar_frame)
        {
            scan_ = std::move(scan);
        }
        else
        {
            scan_ = transformPcd(scan, pose_.inverse());
        }

        timestamp_ = odom.header.stamp;
        idx_       = idx;
    }
};
}  // namespace kiss_matcher

#endif  // KISS_MATCHER_POSE_GRAPH_NODE_HPP
