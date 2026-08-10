#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "common/so3_math.h"
#include "ikd_Tree.h"
#include "data_processors/imu_processor.hpp"
#include "data_processors/lidar_processor.hpp"

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace spark_fast_lio
{

inline constexpr double kInitializationTimeSec = 0.1;
inline constexpr double kLaserPointCovariance  = 0.001;
inline constexpr double kMapMoveThreshold      = 1.5;

struct PoseStruct
{
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
};

class SPARKFastLIO2Test;

class SPARKFastLIO2 : public rclcpp::Node
{
public:
    explicit SPARKFastLIO2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    friend class SPARKFastLIO2Test;

    M3D computeRelativeRotation(const Eigen::Vector3d &gravity_from,
                                const Eigen::Vector3d &gravity_to);

    bool tryLookupBaseExtrinsics(V3D &lidar_translation_in_base,
                                 M3D &lidar_rotation_in_base,
                                 std::string &error);
    void retryBaseExtrinsics();
    void activateSensorProcessing();

    void pointBodyToWorld(PointType const *const input_point,
                          PointType *const output_point,
                          const state_ikfom &state);

    template <typename T>
    void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &input_point,
                          Eigen::Matrix<T, 3, 1> &output_point,
                          const state_ikfom &state) const
    {
        V3D point_in_body(input_point[0], input_point[1], input_point[2]);
        V3D point_in_world(
            state.rot * (state.offset_R_L_I * point_in_body + state.offset_T_L_I) + state.pos);

        output_point[0] = point_in_world(0);
        output_point[1] = point_in_world(1);
        output_point[2] = point_in_world(2);
    }

    void pclPointBodyToWorld(PointType const *const input_point,
                             PointType *const output_point,
                             const state_ikfom &state);

    void pclPointBodyLidarToIMU(PointType const *const input_point, PointType *const output_point);

    void pclPointBodyLidarToBase(PointType const *const input_point, PointType *const output_point);

    void collectRemovedPoints();

    void standardLiDARCallback(const sensor_msgs::msg::PointCloud2 &msg);

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    void livoxLiDARCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
#endif

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    // How resetEstimatorState treats the next IMU initialization.
    // kCold is the safe default: everything is discarded and a fresh
    // stationary initialization is required — use it whenever the estimator
    // state itself may be corrupted. kWarmRecovery is reserved for failures
    // external to the estimate (e.g. sensor timestamp glitches): the pre-reset
    // gravity/bias/velocity are re-seeded so recovery works in motion.
    enum class ResetMode
    {
        kCold,
        kWarmRecovery,
    };

    void resetEstimatorState(const std::string &reason, ResetMode mode = ResetMode::kCold);

    void integrateIMU(esekfom::esekf<state_ikfom, 12, input_ikfom> &state,
                      const sensor_msgs::msg::Imu &msg);

    void computeMeasurementModel(state_ikfom &state,
                                 esekfom::dyn_share_datastruct<double> &ekfom_data);

    void updateLocalMapWindow();

    void insertScanIntoMap(const state_ikfom &state);

    void publishOdometry(const state_ikfom &state, const rclcpp::Time &stamp);
    void publishOdometry(
        const state_ikfom &state,
        const rclcpp::Time &stamp,
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &publisher,
        bool publish_tf);

    void publishPath(const state_ikfom &state);

    void publishCurrentFrame(const state_ikfom &state,
                             const rclcpp::Time &stamp,
                             bool insert_into_map,
                             bool append_path);

    bool topicSubscribed(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) const;

    void publishMapScan(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud,
                        const state_ikfom &state);

    void publishScan(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud,
                     const std::string &frame);

    PoseStruct transformPoseToLidarFrame(const state_ikfom &state) const;

    PoseStruct transformPoseToBaseFrame(const state_ikfom &state) const;

    template <typename T>
    void setPoseStamp(const state_ikfom &state, T &out, const std::string &frame) const
    {
        if (frame == "imu")
        {
            out.pose.position.x    = state.pos(0);
            out.pose.position.y    = state.pos(1);
            out.pose.position.z    = state.pos(2);
            const auto quat        = state.rot.coeffs();
            out.pose.orientation.x = quat[0];
            out.pose.orientation.y = quat[1];
            out.pose.orientation.z = quat[2];
            out.pose.orientation.w = quat[3];
        }
        else if (frame == "lidar")
        {
            const auto &p          = transformPoseToLidarFrame(state);
            out.pose.position.x    = p.position_(0);
            out.pose.position.y    = p.position_(1);
            out.pose.position.z    = p.position_(2);
            out.pose.orientation.x = p.orientation_.x();
            out.pose.orientation.y = p.orientation_.y();
            out.pose.orientation.z = p.orientation_.z();
            out.pose.orientation.w = p.orientation_.w();
        }
        else if (frame == "base")
        {
            const auto &p          = transformPoseToBaseFrame(state);
            out.pose.position.x    = p.position_(0);
            out.pose.position.y    = p.position_(1);
            out.pose.position.z    = p.position_(2);
            out.pose.orientation.x = p.orientation_.x();
            out.pose.orientation.y = p.orientation_.y();
            out.pose.orientation.z = p.orientation_.z();
            out.pose.orientation.w = p.orientation_.w();
        }
        else
        {
            throw std::invalid_argument("Invalid visualization frame has been given");
        }
    }

    void main();

    void processPendingMeasurements();

    bool syncPackages(MeasureGroup &measurements, bool verbose);

    bool isMotionStopped(const V3D &acc_ref, const V3D &acc_curr, const double acc_diff_thr);

    void processLidarAndImu(MeasureGroup &measures);

    struct PropagationCheckpoint;
    struct MotionQualityReport
    {
        double lidar_time = 0.0;
        double delta_time = 0.0;
        double state_step = 0.0;
        double state_speed = 0.0;
        double correction_step = 0.0;
        double correction_step_ratio = 1.0;
        double velocity_norm = 0.0;
        double rotation_correction_deg = 0.0;
        double effective_feature_ratio = 0.0;
        V3D mean_acceleration = Zero3d;
        V3D pre_gravity_residual = Zero3d;
        V3D post_gravity_residual = Zero3d;
        bool finite_state = false;
        bool high_pre_gravity_residual = false;
        bool high_post_gravity_residual = false;
        bool weak_lidar_update = false;
        bool weak_lidar_constraints = false;
        bool suspicious_large_correction = false;
        bool unsupported_recovery_step = false;
        bool reject = false;
    };

    PropagationCheckpoint propagateLidarFrame(const MeasureGroup &measures);
    void restorePropagatedFrame(const PropagationCheckpoint &checkpoint);
    void restorePrePropagationFrame(const PropagationCheckpoint &checkpoint);
    void publishPropagatedFrame();
    PointCloudXYZI::ConstPtr selectMatchingPoints();
    bool prepareLioUpdate(MeasureGroup &measures,
                          const PointCloudXYZI::ConstPtr &matching_points,
                          state_ikfom &propagated_state);
    void updateGravityAlignmentBeforeLio(MeasureGroup &measures);
    bool initializeLocalMapIfNeeded();
    void runLioUpdate();
    void updateGravityAlignmentAfterLio();
    MotionQualityReport evaluateMotionQuality(MeasureGroup &measures,
                                              const state_ikfom &propagated_state);
    void rejectMotionFrame(const MeasureGroup &measures,
                           const PropagationCheckpoint &checkpoint,
                           const MotionQualityReport &quality);
    void commitOdometryUpdate(const MeasureGroup &measures,
                              const state_ikfom &propagated_state,
                              const MotionQualityReport &quality);
    void logLargeStateJump(const MeasureGroup &measures,
                           const state_ikfom &propagated_state,
                           const MotionQualityReport &quality);

private:
    std::mutex buffer_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_livox_;
#endif

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_full_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_base_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_predicted_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    rclcpp::TimerBase::SharedPtr extrinsics_retry_timer_;

    /*** Time Log Variables ***/
    double map_insertion_time_ = 0.0;
    double map_search_time_    = 0.0;
    double map_removal_time_   = 0.0;

    double match_time_         = 0.0;
    double solve_time_         = 0.0;

    bool runtime_pos_log_ = false;
    /**************************/
    int inserted_point_count_    = 0;
    int removed_map_point_count_ = 0;
    bool local_map_initialized_  = false;
    // Data-driven (frame count) gate for the periodic LIO state log; clock
    // throttling would make the formatted-frame subset timing-dependent and
    // break bit-reproducible replay.
    int lio_state_log_counter_ = 0;

    bool pcd_save_enabled_                 = false;
    bool time_sync_enabled_                = false;
    bool extrinsic_est_enabled_            = false;
    bool path_enabled_                     = true;
    double path_history_duration_s_        = 600.0;
    bool scan_publish_enabled_             = false;
    bool dense_publish_enabled_            = false;
    bool scan_lidar_frame_publish_enabled_ = false;
    bool scan_body_frame_publish_enabled_  = false;
    bool scan_base_frame_publish_enabled_  = false;
    bool imu_predicted_odometry_enabled_   = true;
    bool process_on_callback_              = false;
    bool sensor_processing_active_         = false;
    bool motion_quality_gate_enabled_      = false;
    int imu_qos_depth_                     = 1000;
    int lidar_qos_depth_                   = 10;
    rclcpp::ReliabilityPolicy lidar_qos_reliability_ = rclcpp::ReliabilityPolicy::Reliable;
    rclcpp::ReliabilityPolicy imu_qos_reliability_   = rclcpp::ReliabilityPolicy::Reliable;
    std::size_t lidar_buffer_capacity_     = 20;
    std::size_t imu_buffer_capacity_       = 1000;

    bool verbose_     = false;
    bool pcl_verbose_ = true;

    bool enable_gravity_alignment_ = false;
    bool is_gravity_aligned_       = false;

    std::vector<float> point_residuals_;
    float detection_range_ = 300.0f;

    std::string map_file_path_;
    std::string save_dir_;
    std::string sequence_name_;
    std::string map_frame_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string imu_frame_;
    std::string viz_frame_;

    double mean_residual_  = 0.05;
    double total_residual_ = 0.0;
    rclcpp::Time last_lidar_timestamp_;
    rclcpp::Time last_imu_timestamp_;
    bool has_last_lidar_timestamp_ = false;
    bool has_last_imu_timestamp_   = false;
    int64_t last_not_enough_imu_log_timestamp_ns_ = -1;
    int64_t lidar_imu_time_offset_                = 0;

    double gyroscope_covariance_      = 0.1;
    double accelerometer_covariance_  = 0.1;
    double gyroscope_bias_covariance_ = 0.0001;
    double accelerometer_bias_covariance_ = 0.0001;
    double motion_gate_max_pre_grav_residual_ = 3.0;
    double motion_gate_suspect_frame_step_    = 0.5;
    double motion_gate_max_update_step_       = 0.15;
    double motion_gate_max_update_step_ratio_ = 0.2;
    double motion_gate_min_effective_ratio_   = 0.25;
    bool motion_gate_reject_weak_lidar_       = true;

    double filter_size_map_min_   = 0.0;
    double fov_deg_               = 0.0;
    double local_map_side_length_ = 0.0;
    double lidar_end_time_        = 0.0;
    double first_lidar_time_      = 0.0;

    int effective_feature_count_ = 0;
    int scan_count_              = 0;
    int path_publish_counter_    = 0;
    int imu_gap_lidar_skip_count_ = 0;

    int downsampled_point_count_            = 0;
    int max_iterations_                     = 0;
    int pcd_save_interval_                  = -1;
    int pcd_index_                          = 0;
    int pcd_scan_wait_count_                = 0;
    int point_filter_num_                   = 4;  // empirically, 4 showed the best performance
    int motion_gate_min_effective_features_ = 100;
    int motion_gate_reject_count_              = 0;
    int motion_gate_consecutive_reject_count_ = 0;
    double mean_scan_duration_ = 0.0;
    int scan_duration_sample_count_ = 0;
    std::size_t verbose_lidar_buffer_size_ = 0;
    std::size_t verbose_imu_buffer_size_   = 0;

    double acceleration_difference_threshold_ = 0.2;
    int moving_frame_threshold_               = 10;
    int gravity_measurement_threshold_        = 10;
    std::vector<std::uint8_t> surface_point_selected_;
    bool lidar_pushed_        = false;
    bool is_first_lidar_scan_ = true;
    bool filter_initialized_       = false;
    bool time_offset_initialized_ = false;
    int num_consecutive_moving_frames_ = 0;

    std::deque<V3D> global_gravity_directions_;

    BoxPointType local_map_bounds_;
    std::vector<BoxPointType> map_boxes_to_remove_;

    std::vector<PointVector> nearest_map_points_;
    std::vector<double> extrinsic_translation_{0.0, 0.0, 0.0};
    std::vector<double> extrinsic_rotation_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    double extrinsics_timeout_s_ = 10.0;
    std::chrono::steady_clock::time_point extrinsics_wait_started_;
    std::chrono::steady_clock::time_point last_extrinsics_wait_log_;
    bool extrinsics_timeout_reported_ = false;

    struct BufferedLidarFrame
    {
        PointCloudXYZI::Ptr cloud;
        double begin_time = 0.0;
        double end_time   = 0.0;
    };

    boost::circular_buffer<BufferedLidarFrame> lidar_buffer_;
    boost::circular_buffer<std::shared_ptr<const sensor_msgs::msg::Imu>> imu_buffer_;
    std::deque<sensor_msgs::msg::Imu> imu_integration_queue_;

    PointCloudXYZI::Ptr full_points_;
    PointCloudXYZI::Ptr sampled_points_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    PointCloudXYZI::Ptr point_normals_;
    PointCloudXYZI::Ptr selected_points_;
    PointCloudXYZI::Ptr selected_normals_;
    PointCloudXYZI::Ptr cloud_to_be_saved_;

    pcl::VoxelGrid<PointType> down_size_filter_;
    KD_TREE<PointType> ikd_tree_;

    V3F xaxis_point_body_;
    V3F xaxis_point_world_;
    V3D base_gravity_;
    V3D stationary_mean_acceleration_;
    V3D position_last_;
    M3D gravity_alignment_rotation_;

    /*** Only used for integration with the Hydra system ***/
    V3D lidar_translation_in_base_;
    M3D lidar_rotation_in_base_;

    /*** EKF inputs and output ***/
    MeasureGroup measures_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    std::optional<esekfom::esekf<state_ikfom, 12, input_ikfom>> kf_for_preintegration_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> last_good_kf_;
    std::optional<ImuProcessor::Snapshot> last_good_imu_processor_snapshot_;
    // Always remains in the raw EKF/map frame. Publishing applies gravity
    // alignment to a copy so matching and map insertion stay in one frame.
    state_ikfom latest_state_;
    state_ikfom last_good_state_;
    bool have_last_good_state_ = false;
    bool have_last_lio_debug_state_ = false;
    V3D last_lio_debug_pos_         = Zero3d;
    double last_lio_debug_time_     = 0.0;

    nav_msgs::msg::Path path_msg_;
    nav_msgs::msg::Odometry odomAftMapped_;
    geometry_msgs::msg::PoseStamped msg_body_pose_;

    std::shared_ptr<LidarProcessor> lidar_processor_;
    std::shared_ptr<ImuProcessor> imu_processor_;
};

}  // namespace spark_fast_lio
