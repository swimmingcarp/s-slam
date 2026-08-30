#include "spark_fast_lio.h"

#include "common/gravity_alignment.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>

#include <pcl_conversions/pcl_conversions.h>

namespace spark_fast_lio
{
namespace
{
constexpr int kPositionStateIndex             = 0;
constexpr int kOrientationStateIndex          = 3;
constexpr int kExtrinsicRotationStateIndex    = 6;
constexpr int kExtrinsicTranslationStateIndex = 9;
constexpr int kVelocityStateIndex             = 12;
constexpr int kGyroscopeBiasStateIndex        = 15;
}

SPARKFastLIO2::PoseCovariance SPARKFastLIO2::poseCovariance(
    const state_ikfom &state,
    const StateCovariance &state_covariance,
    const M3D &world_rotation,
    const std::string &body_frame) const
{
    Eigen::Matrix<double, 6, state_ikfom::DOF> pose_jacobian =
        Eigen::Matrix<double, 6, state_ikfom::DOF>::Zero();
    const M3D imu_rotation_in_world = world_rotation * state.rot.toRotationMatrix();
    const M3D lidar_rotation_in_imu = state.offset_R_L_I.toRotationMatrix();

    // The EKF's SO3 errors are right perturbations. ROS pose covariance uses
    // fixed map-frame axes, so rotate every pose error into the published map frame.
    pose_jacobian.template block<3, 3>(0, kPositionStateIndex) = world_rotation;
    pose_jacobian.template block<3, 3>(3, kOrientationStateIndex) = imu_rotation_in_world;

    if (body_frame == "lidar")
    {
        pose_jacobian.template block<3, 3>(0, kOrientationStateIndex) =
            -imu_rotation_in_world * skew_sym_mat(state.offset_T_L_I);
        pose_jacobian.template block<3, 3>(0, kExtrinsicTranslationStateIndex) =
            imu_rotation_in_world;
        pose_jacobian.template block<3, 3>(3, kExtrinsicRotationStateIndex) =
            imu_rotation_in_world * lidar_rotation_in_imu;
    }
    else if (body_frame == "base")
    {
        const M3D base_rotation_in_lidar = lidar_rotation_in_base_.transpose();
        const M3D base_rotation_in_imu   = lidar_rotation_in_imu * base_rotation_in_lidar;
        const V3D base_translation_in_imu =
            state.offset_T_L_I - base_rotation_in_imu * lidar_translation_in_base_;
        const V3D base_to_lidar_translation_in_lidar =
            base_rotation_in_lidar * lidar_translation_in_base_;

        pose_jacobian.template block<3, 3>(0, kOrientationStateIndex) =
            -imu_rotation_in_world * skew_sym_mat(base_translation_in_imu);
        pose_jacobian.template block<3, 3>(0, kExtrinsicRotationStateIndex) =
            imu_rotation_in_world * lidar_rotation_in_imu *
            skew_sym_mat(base_to_lidar_translation_in_lidar);
        pose_jacobian.template block<3, 3>(0, kExtrinsicTranslationStateIndex) =
            imu_rotation_in_world;
        pose_jacobian.template block<3, 3>(3, kExtrinsicRotationStateIndex) =
            imu_rotation_in_world * lidar_rotation_in_imu;
    }
    else if (body_frame != "imu")
    {
        throw std::invalid_argument("Invalid visualization frame has been given");
    }

    const PoseCovariance covariance =
        pose_jacobian * state_covariance * pose_jacobian.transpose();
    return 0.5 * (covariance + covariance.transpose());
}

M3D SPARKFastLIO2::baseLinearVelocityCovariance(const state_ikfom &state,
                                                 const StateCovariance &state_covariance,
                                                 const V3D &angular_velocity,
                                                 const M3D &world_rotation) const
{
    const M3D lidar_rotation_in_imu = state.offset_R_L_I.toRotationMatrix();
    const M3D base_rotation_in_lidar = lidar_rotation_in_base_.transpose();
    const M3D base_rotation_in_imu = lidar_rotation_in_imu * base_rotation_in_lidar;
    const V3D base_translation_in_imu =
        state.offset_T_L_I - base_rotation_in_imu * lidar_translation_in_base_;
    const V3D base_to_lidar_translation_in_lidar =
        base_rotation_in_lidar * lidar_translation_in_base_;
    const V3D angular_lever_arm_velocity = angular_velocity.cross(base_translation_in_imu);
    const M3D imu_rotation_in_world = world_rotation * state.rot.toRotationMatrix();

    Eigen::Matrix<double, 3, state_ikfom::DOF> velocity_jacobian =
        Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
    velocity_jacobian.template block<3, 3>(0, kOrientationStateIndex) =
        -imu_rotation_in_world * skew_sym_mat(angular_lever_arm_velocity);
    velocity_jacobian.template block<3, 3>(0, kExtrinsicRotationStateIndex) =
        imu_rotation_in_world * skew_sym_mat(angular_velocity) * lidar_rotation_in_imu *
        skew_sym_mat(base_to_lidar_translation_in_lidar);
    velocity_jacobian.template block<3, 3>(0, kExtrinsicTranslationStateIndex) =
        imu_rotation_in_world * skew_sym_mat(angular_velocity);
    velocity_jacobian.template block<3, 3>(0, kVelocityStateIndex) = world_rotation;
    velocity_jacobian.template block<3, 3>(0, kGyroscopeBiasStateIndex) =
        imu_rotation_in_world * skew_sym_mat(base_translation_in_imu);

    const M3D covariance = velocity_jacobian * state_covariance * velocity_jacobian.transpose();
    return 0.5 * (covariance + covariance.transpose());
}

void SPARKFastLIO2::publishPx4Odometry(const state_ikfom &state,
                                       const StateCovariance &state_covariance,
                                       const V3D &angular_velocity,
                                       const M3D &world_rotation,
                                       const rclcpp::Time &stamp)
{
    if (!pub_px4_odometry_)
    {
        return;
    }

    const M3D lidar_rotation_in_imu = state.offset_R_L_I.toRotationMatrix();
    const M3D base_rotation_in_lidar = lidar_rotation_in_base_.transpose();
    const M3D base_rotation_in_imu = lidar_rotation_in_imu * base_rotation_in_lidar;
    const V3D base_translation_in_imu =
        state.offset_T_L_I - base_rotation_in_imu * lidar_translation_in_base_;
    const state_ikfom output_state =
        world_rotation.isApprox(M3D::Identity()) ? state : gravityAlignedState(state, world_rotation);
    const PoseStruct base_pose = transformPoseToBaseFrame(output_state);
    const V3D base_linear_velocity = world_rotation *
        (state.vel + state.rot * angular_velocity.cross(base_translation_in_imu));
    const PoseCovariance pose_covariance =
        poseCovariance(state, state_covariance, world_rotation, "base");
    const M3D linear_velocity_covariance =
        baseLinearVelocityCovariance(state, state_covariance, angular_velocity, world_rotation);

    s_slam_interfaces::msg::Px4Odometry px4_odometry;
    px4_odometry.header.frame_id = map_frame_;
    px4_odometry.header.stamp = stamp;
    px4_odometry.body_frame = base_frame_;
    px4_odometry.pose.position.x = base_pose.position_.x();
    px4_odometry.pose.position.y = base_pose.position_.y();
    px4_odometry.pose.position.z = base_pose.position_.z();
    px4_odometry.pose.orientation.x = base_pose.orientation_.x();
    px4_odometry.pose.orientation.y = base_pose.orientation_.y();
    px4_odometry.pose.orientation.z = base_pose.orientation_.z();
    px4_odometry.pose.orientation.w = base_pose.orientation_.w();
    px4_odometry.linear_velocity.x = base_linear_velocity.x();
    px4_odometry.linear_velocity.y = base_linear_velocity.y();
    px4_odometry.linear_velocity.z = base_linear_velocity.z();
    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            px4_odometry.pose_covariance[row * 6 + column] = pose_covariance(row, column);
        }
    }
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            px4_odometry.linear_velocity_covariance[row * 3 + column] =
                linear_velocity_covariance(row, column);
        }
    }
    px4_odometry.reset_counter = odom_reset_counter_;
    pub_px4_odometry_->publish(px4_odometry);
}

void SPARKFastLIO2::publishOdometry(
    const state_ikfom &state,
    const PoseCovariance &pose_covariance,
    const rclcpp::Time &stamp,
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &publisher,
    bool publish_tf)
{
    odomAftMapped_.header.frame_id = map_frame_;
    odomAftMapped_.header.stamp    = stamp;

    setPoseStamp(state, odomAftMapped_.pose, viz_frame_);  // our template function

    if (viz_frame_ == "lidar")
    {
        odomAftMapped_.child_frame_id = lidar_frame_;
    }
    else if (viz_frame_ == "base")
    {
        odomAftMapped_.child_frame_id = base_frame_;
    }
    else if (viz_frame_ == "imu")
    {
        odomAftMapped_.child_frame_id = imu_frame_;
    }
    else
    {
        throw std::invalid_argument("Invalid visualization frame has been given");
    }

    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            odomAftMapped_.pose.covariance[row * 6 + column] = pose_covariance(row, column);
        }
    }

    // publish
    publisher->publish(odomAftMapped_);
    if (!publish_tf)
    {
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = odomAftMapped_.header.stamp;
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id  = odomAftMapped_.child_frame_id;

    transform_stamped.transform.translation.x = odomAftMapped_.pose.pose.position.x;
    transform_stamped.transform.translation.y = odomAftMapped_.pose.pose.position.y;
    transform_stamped.transform.translation.z = odomAftMapped_.pose.pose.position.z;
    transform_stamped.transform.rotation      = odomAftMapped_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform_stamped);
}

void SPARKFastLIO2::publishPath(const state_ikfom &state)
{
    setPoseStamp(state, msg_body_pose_, viz_frame_);
    msg_body_pose_.header.stamp    = rclcpp::Time(lidar_end_time_ * 1e9);
    msg_body_pose_.header.frame_id = map_frame_;

    ++path_publish_counter_;
    if (path_publish_counter_ % 10 == 0)
    {
        path_msg_.poses.push_back(msg_body_pose_);
        const rclcpp::Time newest_pose_time(msg_body_pose_.header.stamp);
        const rclcpp::Time cutoff =
            newest_pose_time - rclcpp::Duration::from_seconds(path_history_duration_s_);
        const auto oldest_pose = std::lower_bound(
            path_msg_.poses.begin(),
            path_msg_.poses.end(),
            cutoff,
            [](const geometry_msgs::msg::PoseStamped &pose, const rclcpp::Time &time)
            {
                return rclcpp::Time(pose.header.stamp) < time;
            });
        path_msg_.poses.erase(path_msg_.poses.begin(), oldest_pose);
        pub_path_->publish(path_msg_);
    }
}

void SPARKFastLIO2::publishCurrentFrame(const state_ikfom &state,
                                        const rclcpp::Time &stamp,
                                        bool insert_into_map,
                                        bool append_path)
{
    const M3D world_rotation =
        is_gravity_aligned_ ? gravity_alignment_rotation_ : M3D::Identity();
    const PoseCovariance pose_covariance =
        poseCovariance(state, kf_.get_P(), world_rotation, viz_frame_);

    std::optional<state_ikfom> aligned_state;
    const state_ikfom *output_state = &state;
    if (is_gravity_aligned_)
    {
        aligned_state.emplace(gravityAlignedState(state, gravity_alignment_rotation_));
        output_state = &*aligned_state;
    }

    publishOdometry(*output_state, pose_covariance, stamp, pub_odom_, true);

    if (insert_into_map)
    {
        insertScanIntoMap(state);
    }

    if (append_path && path_enabled_)
    {
        publishPath(*output_state);
    }

    if (!scan_publish_enabled_)
    {
        return;
    }

    publishMapScan(pub_cloud_full_, *output_state);
    if (scan_lidar_frame_publish_enabled_)
    {
        publishScan(pub_cloud_lidar_, "lidar");
    }
    if (scan_body_frame_publish_enabled_)
    {
        publishScan(pub_cloud_body_, "imu");
    }
    if (scan_base_frame_publish_enabled_)
    {
        publishScan(pub_cloud_base_, "base");
    }
}

bool SPARKFastLIO2::topicSubscribed(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) const
{
    return publisher &&
           (publisher->get_subscription_count() > 0 ||
            publisher->get_intra_process_subscription_count() > 0);
}

void SPARKFastLIO2::publishMapScan(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud,
    const state_ikfom &state)
{
    if (!scan_publish_enabled_)
    {
        return;
    }

    if (topicSubscribed(pubCloud))
    {
        // choose which cloud to publish
        PointCloudXYZI::Ptr laserCloudFullRes(
            dense_publish_enabled_ ? full_points_ : feats_down_body_);

        int size = laserCloudFullRes->points.size();
        // Use the same public map coordinates as odometry and path. After
        // gravity alignment, `state` is the aligned output-state copy while
        // the internal matching map remains in the raw EKF frame.
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; ++i)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i], state);
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*laserCloudWorld, cloud_msg);
        // use lidar_end_time_ for the timestamp
        cloud_msg.header.stamp    = rclcpp::Time(lidar_end_time_ * 1e9);  // from seconds
        cloud_msg.header.frame_id = map_frame_;

        pubCloud->publish(cloud_msg);
    }

    // Optionally save accumulated point clouds.
    if (pcd_save_enabled_)
    {
        int nsize = full_points_->points.size();
        PointCloudXYZI::Ptr laserCloudWorld2(new PointCloudXYZI(nsize, 1));

        for (int i = 0; i < nsize; ++i)
        {
            pointBodyToWorld(&full_points_->points[i], &laserCloudWorld2->points[i], state);
        }
        if (pcd_save_interval_ > 0)
        {
            *cloud_to_be_saved_ += *laserCloudWorld2;  // see below if you store that as a member
        }

        ++pcd_scan_wait_count_;
        if (cloud_to_be_saved_->size() > 0 && pcd_save_interval_ > 0 &&
            pcd_scan_wait_count_ >= pcd_save_interval_)
        {
            ++pcd_index_;
            std::string all_points_dir(std::string(ROOT_DIR) + "PCD/scans_" +
                                       std::to_string(pcd_index_) + ".pcd");
            pcl::PCDWriter pcd_writer;
            std::cout << "Current scan saved to /PCD/ " << all_points_dir << std::endl;
            pcd_writer.writeBinary(all_points_dir, *cloud_to_be_saved_);
            cloud_to_be_saved_->clear();
            pcd_scan_wait_count_ = 0;
        }
    }
}

void SPARKFastLIO2::publishScan(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud,
    const std::string &frame)
{
    if (!topicSubscribed(pubCloud))
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (frame == "lidar")
    {
        pcl::toROSMsg(*full_points_, cloud_msg);
        cloud_msg.header.frame_id = lidar_frame_;
    }
    else if (frame == "imu")
    {
        const int size = full_points_->points.size();
        PointCloudXYZI::Ptr transformed_cloud(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; ++i)
        {
            pclPointBodyLidarToIMU(&full_points_->points[i], &transformed_cloud->points[i]);
        }
        pcl::toROSMsg(*transformed_cloud, cloud_msg);
        cloud_msg.header.frame_id = imu_frame_;
    }
    else if (frame == "base")
    {
        const int size = full_points_->points.size();
        PointCloudXYZI::Ptr transformed_cloud(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; ++i)
        {
            pclPointBodyLidarToBase(&full_points_->points[i], &transformed_cloud->points[i]);
        }
        pcl::toROSMsg(*transformed_cloud, cloud_msg);
        cloud_msg.header.frame_id = base_frame_;
    }
    else
    {
        throw std::invalid_argument("Invalid frame has been given");
    }

    cloud_msg.header.stamp = rclcpp::Time(lidar_end_time_ * 1e9);
    pubCloud->publish(cloud_msg);
}

PoseStruct SPARKFastLIO2::transformPoseToLidarFrame(const state_ikfom &state) const
{
    // T_W_L = T_W_I * T_I_L. The map frame is never re-expressed for display.
    const Eigen::Vector3d lidar_position = state.rot * state.offset_T_L_I + state.pos;
    const Eigen::Quaterniond lidar_orientation(state.rot * state.offset_R_L_I);

    PoseStruct output;
    output.position_    = lidar_position;
    output.orientation_ = lidar_orientation;
    return output;
}
PoseStruct SPARKFastLIO2::transformPoseToBaseFrame(const state_ikfom &state) const
{
    const Eigen::Matrix3d rotation_imu_base =
        state.offset_R_L_I * lidar_rotation_in_base_.inverse();
    const Eigen::Vector3d translation_imu_base =
        state.offset_T_L_I - rotation_imu_base * lidar_translation_in_base_;
    const Eigen::Vector3d base_position = state.rot * translation_imu_base + state.pos;
    const Eigen::Quaterniond base_orientation(state.rot * rotation_imu_base);

    PoseStruct output;
    output.position_    = base_position;
    output.orientation_ = base_orientation;
    return output;
}
}  // namespace spark_fast_lio
