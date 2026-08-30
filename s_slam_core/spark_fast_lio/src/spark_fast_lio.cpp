#include "spark_fast_lio.h"

#include "common/gravity_alignment.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

namespace spark_fast_lio
{
namespace
{
constexpr double kMaximumImuGapScanPeriods = 1.2;
constexpr double kExtrinsicRotationTolerance = 1.0e-3;
constexpr std::size_t kPredictedOdometryQueueDepth = 100;
constexpr std::size_t kPx4OdometryQueueDepth = 10;

bool hasFiniteState(const state_ikfom &state)
{
    const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
    return state.pos.allFinite() && state.vel.allFinite() && state.bg.allFinite() &&
           state.ba.allFinite() && gravity.allFinite() &&
           state.rot.toRotationMatrix().allFinite() &&
           state.offset_R_L_I.toRotationMatrix().allFinite() && state.offset_T_L_I.allFinite();
}

bool canSeedWarmRecovery(const state_ikfom &state, const V3D &mean_acceleration)
{
    const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
    return hasFiniteState(state) && mean_acceleration.allFinite() &&
           gravity.norm() > 0.5 * G_m_s2 && mean_acceleration.norm() >= kMinInitAccNorm;
}

rclcpp::ReliabilityPolicy parseReliabilityPolicy(const std::string &value,
                                                  const std::string &parameter_name)
{
    if (value == "reliable")
    {
        return rclcpp::ReliabilityPolicy::Reliable;
    }
    if (value == "best_effort")
    {
        return rclcpp::ReliabilityPolicy::BestEffort;
    }
    throw std::invalid_argument(
        parameter_name + " must be 'reliable' or 'best_effort'");
}

const char *reliabilityPolicyName(const rclcpp::ReliabilityPolicy policy)
{
    return policy == rclcpp::ReliabilityPolicy::BestEffort ? "best_effort" : "reliable";
}

void validateExtrinsicParameters(const std::vector<double> &translation,
                                 const std::vector<double> &rotation)
{
    if (translation.size() != 3)
    {
        throw std::invalid_argument("mapping.extrinsic_T must contain exactly three values");
    }
    if (rotation.size() != 9)
    {
        throw std::invalid_argument("mapping.extrinsic_R must contain exactly nine values");
    }
    const auto is_finite = [](const double value) { return std::isfinite(value); };
    if (!std::all_of(translation.begin(), translation.end(), is_finite) ||
        !std::all_of(rotation.begin(), rotation.end(), is_finite))
    {
        throw std::invalid_argument("mapping.extrinsic_T and mapping.extrinsic_R must be finite");
    }

    Eigen::Matrix3d lidar_rotation;
    lidar_rotation << rotation[0], rotation[1], rotation[2], rotation[3], rotation[4], rotation[5],
        rotation[6], rotation[7], rotation[8];

    const double orthogonality_error =
        (lidar_rotation.transpose() * lidar_rotation - Eigen::Matrix3d::Identity()).norm();
    const double determinant = lidar_rotation.determinant();
    if (orthogonality_error > kExtrinsicRotationTolerance || determinant <= 0.0 ||
        std::abs(determinant - 1.0) > kExtrinsicRotationTolerance)
    {
        throw std::invalid_argument(
            "mapping.extrinsic_R must be a proper rotation matrix");
    }
}
}  // namespace

SPARKFastLIO2::SPARKFastLIO2(const rclcpp::NodeOptions &options)
    : Node("spark_fast_lio_node", options),
      clock_(get_clock()),
      last_lidar_timestamp_(0, 0, RCL_ROS_TIME),
      last_imu_timestamp_(0, 0, RCL_ROS_TIME)
{
    base_gravity_                 = Zero3d;
    stationary_mean_acceleration_ = Zero3d;
    gravity_alignment_rotation_   = Eye3d;

    lidar_translation_in_base_ = Zero3d;
    lidar_rotation_in_base_    = Eye3d;

    path_enabled_ = declare_parameter<bool>("publish.path_enabled", true);
    path_history_duration_s_ =
        declare_parameter<double>("publish.path_history_duration_sec", 600.0);
    if (path_history_duration_s_ < 0.0)
    {
        throw std::invalid_argument("publish.path_history_duration_sec must be non-negative");
    }
    scan_publish_enabled_             = declare_parameter<bool>("publish.scan_publish_enabled", false);
    dense_publish_enabled_            = declare_parameter<bool>("publish.dense_publish_enabled", false);
    scan_lidar_frame_publish_enabled_ = declare_parameter<bool>("publish.scan_lidar_frame_publish_enabled", false);
    scan_body_frame_publish_enabled_  = declare_parameter<bool>("publish.scan_body_frame_publish_enabled", false);
    scan_base_frame_publish_enabled_  = declare_parameter<bool>("publish.scan_base_frame_publish_enabled", false);
    imu_predicted_odometry_enabled_ =
        declare_parameter<bool>("publish.imu_predicted_odometry_enabled", true);

    max_iterations_ = declare_parameter<int>("max_iteration", 4);

    map_frame_     = declare_parameter<std::string>("common.map_frame", "odom");
    lidar_frame_   = declare_parameter<std::string>("common.lidar_frame", "lidar");
    base_frame_    = declare_parameter<std::string>("common.base_frame", "");
    imu_frame_     = declare_parameter<std::string>("common.imu_frame", "imu");
    viz_frame_     = declare_parameter<std::string>("common.visualization_frame", "imu");
    if (viz_frame_ != "imu" && viz_frame_ != "lidar" && viz_frame_ != "base")
    {
        throw std::invalid_argument(
            "common.visualization_frame must be one of: imu, lidar, base");
    }
    time_sync_enabled_ = declare_parameter<bool>("common.time_sync_enabled", false);
    process_on_callback_ = declare_parameter<bool>("common.process_on_callback", false);
    imu_qos_depth_     = declare_parameter<int>("common.imu_qos_depth", 1000);
    imu_qos_depth_     = std::max(10, imu_qos_depth_);
    // Deep queues make replay transport lossless; the live default keeps
    // real-time drop behavior for on-robot use.
    lidar_qos_depth_ = declare_parameter<int>("common.lidar_qos_depth", 10);
    lidar_qos_depth_ = std::max(1, lidar_qos_depth_);
    lidar_qos_reliability_ = parseReliabilityPolicy(
        declare_parameter<std::string>("common.lidar_qos_reliability", "reliable"),
        "common.lidar_qos_reliability");
    imu_qos_reliability_ = parseReliabilityPolicy(
        declare_parameter<std::string>("common.imu_qos_reliability", "reliable"),
        "common.imu_qos_reliability");
    lidar_buffer_capacity_ = static_cast<std::size_t>(std::max<int64_t>(
        1, declare_parameter<int>("common.lidar_buffer_capacity", 20)));
    imu_buffer_capacity_ = static_cast<std::size_t>(std::max<int64_t>(
        1, declare_parameter<int>("common.imu_buffer_capacity", 1000)));
    // A bounded sliding window keeps the newest sensor data when processing
    // falls behind. Full buffers discard stale entries instead of resetting LIO.
    lidar_buffer_.set_capacity(lidar_buffer_capacity_);
    imu_buffer_.set_capacity(imu_buffer_capacity_);
    imu_prediction_buffer_.set_capacity(imu_buffer_capacity_);

    filter_size_map_min_ = declare_parameter<double>("filter_size_map", 0.5);
    if (!std::isfinite(filter_size_map_min_) || filter_size_map_min_ <= 0.0)
    {
        throw std::invalid_argument("filter_size_map must be positive and finite");
    }
    local_map_side_length_      = declare_parameter<double>("cube_side_length", 200.0);
    detection_range_             = declare_parameter<double>("mapping.det_range", 300.0);
    if (!std::isfinite(local_map_side_length_) || local_map_side_length_ <= 0.0)
    {
        throw std::invalid_argument("cube_side_length must be positive and finite");
    }
    if (!std::isfinite(detection_range_) || detection_range_ <= 0.0)
    {
        throw std::invalid_argument("mapping.det_range must be positive and finite");
    }
    fov_deg_                     = declare_parameter<double>("mapping.fov_degree", 360.0);
    gyroscope_covariance_        = declare_parameter<double>("mapping.gyr_cov", 0.1);
    accelerometer_covariance_    = declare_parameter<double>("mapping.acc_cov", 0.1);
    gyroscope_bias_covariance_   = declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    accelerometer_bias_covariance_ = declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    quality_gate_enabled_ = declare_parameter<bool>("mapping.quality_gate_enabled", false);
    max_linear_acceleration_ =
        declare_parameter<double>("mapping.max_linear_acceleration", 3.0);
    max_linear_speed_ = declare_parameter<double>("mapping.max_linear_speed", 12.4);
    max_lidar_position_adjustment_ =
        declare_parameter<double>("mapping.max_lidar_position_adjustment", 0.15);
    min_recovery_lidar_adjustment_ratio_ =
        declare_parameter<double>("mapping.min_recovery_lidar_adjustment_ratio", 0.2);
    min_matched_feature_ratio_ =
        declare_parameter<double>("mapping.min_matched_feature_ratio", 0.25);
    min_matched_features_ = declare_parameter<int>("mapping.min_matched_features", 100);
    reject_weak_lidar_ = declare_parameter<bool>("mapping.reject_weak_lidar", true);
    max_linear_acceleration_ = std::max(0.1, max_linear_acceleration_);
    max_linear_speed_ = std::max(0.1, max_linear_speed_);
    max_lidar_position_adjustment_ = std::max(0.01, max_lidar_position_adjustment_);
    min_recovery_lidar_adjustment_ratio_ =
        std::clamp(min_recovery_lidar_adjustment_ratio_, 0.0, 1.0);
    min_matched_feature_ratio_ = std::clamp(min_matched_feature_ratio_, 0.0, 1.0);
    min_matched_features_ = std::max(1, min_matched_features_);

    enable_gravity_alignment_ =
        declare_parameter<bool>("gravity_alignment.enable_gravity_alignment", true);
    acceleration_difference_threshold_ =
        declare_parameter<double>("gravity_alignment.acc_diff_thr", 0.2);
    moving_frame_threshold_ =
        declare_parameter<int>("gravity_alignment.num_moving_frames_thr", 20);
    gravity_measurement_threshold_ =
        declare_parameter<int>("gravity_alignment.num_gravity_measurements_thr", 20);

    verbose_     = declare_parameter<bool>("verbose", false);
    pcl_verbose_ = declare_parameter<bool>("pcl_verbose", true);
    if (!pcl_verbose_)
    {
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    extrinsic_est_enabled_ = declare_parameter<bool>("mapping.extrinsic_est_enabled", false);
    const bool replay_mode = declare_parameter<bool>("replay_mode", false);
    extrinsics_timeout_s_  = declare_parameter<double>("extrinsics_timeout_s", 10.0);
    pcd_save_enabled_      = declare_parameter<bool>("pcd_save.pcd_save_enabled", false);
    pcd_save_interval_     = declare_parameter<int>("pcd_save.interval", -1);

    point_filter_num_ = std::max(1, static_cast<int>(declare_parameter<int>("point_filter_num", 4)));

    // extrinsic_translation_ and extrinsic_rotation_ are sized 3 and 9 respectively
    extrinsic_translation_ =
        declare_parameter<std::vector<double>>("mapping.extrinsic_T", extrinsic_translation_);
    extrinsic_rotation_ =
        declare_parameter<std::vector<double>>("mapping.extrinsic_R", extrinsic_rotation_);
    validateExtrinsicParameters(extrinsic_translation_, extrinsic_rotation_);

    const auto gravity_vector =
        declare_parameter<std::vector<double>>("gravity_alignment.g_base", {0.0, 0.0, -1.0});
    if (gravity_vector.size() != 3 ||
        !std::all_of(gravity_vector.begin(),
                     gravity_vector.end(),
                     [](const double value) { return std::isfinite(value); }))
    {
        throw std::invalid_argument(
            "gravity_alignment.g_base must contain exactly three finite values");
    }
    base_gravity_ << gravity_vector[0], gravity_vector[1], gravity_vector[2];
    if (base_gravity_.norm() <= std::numeric_limits<double>::epsilon())
    {
        throw std::invalid_argument("gravity_alignment.g_base must be non-zero");
    }

    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    rclcpp::QoS predicted_odometry_qos{
        rclcpp::KeepLast(kPredictedOdometryQueueDepth)};
    predicted_odometry_qos.reliable();
    predicted_odometry_qos.durability_volatile();
    // The local bridge selects a 30 Hz sample from this IMU-rate state stream.
    // Preserve a short burst so its timestamp selector does not miss candidates.
    rclcpp::QoS px4_odometry_qos{rclcpp::KeepLast(kPx4OdometryQueueDepth)};
    px4_odometry_qos.reliable();
    px4_odometry_qos.durability_volatile();
    pub_cloud_full_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", qos);
    pub_cloud_lidar_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_lidar", qos);
    pub_cloud_body_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", qos);
    pub_cloud_base_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_base", qos);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
    pub_imu_predicted_odom_ =
        create_publisher<nav_msgs::msg::Odometry>("odometry_imu_predicted", predicted_odometry_qos);
    pub_px4_odometry_ =
        create_publisher<s_slam_interfaces::msg::Px4Odometry>("px4_odometry", px4_odometry_qos);
    pub_path_                 = create_publisher<nav_msgs::msg::Path>("path", qos);
    path_msg_.header.frame_id = map_frame_;

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Keep the established preprocess.* parameter keys compatible with existing YAML files.
    LidarProcessor::Config lidar_processor_config;
    lidar_processor_config.blind_distance = declare_parameter<double>("preprocess.blind", 0.01);
    lidar_processor_config.pilot_zone_blind_distance =
        declare_parameter<double>("preprocess.blind_for_human_pilots", 1.5);
    const int lidar_type_parameter = declare_parameter<int>(
        "preprocess.lidar_type", static_cast<int>(LidarType::kAvia));
    lidar_processor_config.lidar_type =
        LidarProcessor::GetLidarType(lidar_type_parameter);
    lidar_processor_config.scan_line_count = declare_parameter<int>("preprocess.scan_line", 16);
    const int timestamp_unit_parameter = declare_parameter<int>(
        "preprocess.timestamp_unit", static_cast<int>(TimestampUnit::kMicroseconds));
    lidar_processor_config.timestamp_unit =
        LidarProcessor::GetTimestampUnit(timestamp_unit_parameter);
    lidar_processor_config.scan_rate_hz = declare_parameter<int>("preprocess.scan_rate", 10);
    lidar_processor_config.point_filter_stride = std::max(
        1,
        static_cast<int>(declare_parameter<int>("point_filter_num_for_preprocessing", 1)));
    lidar_processor_ = std::make_shared<LidarProcessor>(lidar_processor_config);
    // One LiDAR scan period plus the established 20% timing tolerance is the
    // longest IMU gap that can still support reliable scan de-skewing.
    max_imu_gap_ =
        kMaximumImuGapScanPeriods / static_cast<double>(lidar_processor_->scanRateHz());

    imu_processor_ = std::make_shared<ImuProcessor>();
    if (extrinsic_translation_.size() == 3 && extrinsic_rotation_.size() == 9)
    {
        Eigen::Vector3d lidar_translation(
            extrinsic_translation_[0], extrinsic_translation_[1], extrinsic_translation_[2]);
        Eigen::Matrix3d lidar_rotation;
        lidar_rotation << extrinsic_rotation_[0], extrinsic_rotation_[1], extrinsic_rotation_[2],
            extrinsic_rotation_[3], extrinsic_rotation_[4], extrinsic_rotation_[5],
            extrinsic_rotation_[6], extrinsic_rotation_[7], extrinsic_rotation_[8];
        imu_processor_->setExtrinsic(lidar_translation, lidar_rotation);
    }

    imu_processor_->setGyroscopeCovariance(
        Eigen::Vector3d(gyroscope_covariance_, gyroscope_covariance_, gyroscope_covariance_));
    imu_processor_->setAccelerometerCovariance(Eigen::Vector3d(
        accelerometer_covariance_, accelerometer_covariance_, accelerometer_covariance_));
    imu_processor_->setGyroscopeBiasCovariance(Eigen::Vector3d(
        gyroscope_bias_covariance_, gyroscope_bias_covariance_, gyroscope_bias_covariance_));
    imu_processor_->setAccelerometerBiasCovariance(Eigen::Vector3d(
        accelerometer_bias_covariance_, accelerometer_bias_covariance_, accelerometer_bias_covariance_));
    imu_processor_->setReplayMode(replay_mode);

    down_size_filter_.setLeafSize(filter_size_map_min_, filter_size_map_min_, filter_size_map_min_);

    double epsi[23];
    for (int i = 0; i < 23; ++i)
    {
        epsi[i] = 0.001;
    }

    kf_.init_dyn_share(
        get_f,
        df_dx,
        df_dw,
        // we use a lambda so we can call a member function
        std::bind(&SPARKFastLIO2::computeMeasurementModel,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2),
        max_iterations_,
        epsi);

    full_points_.reset(new PointCloudXYZI());
    sampled_points_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    point_normals_.reset(new PointCloudXYZI());
    selected_points_.reset(new PointCloudXYZI());
    selected_normals_.reset(new PointCloudXYZI());
    cloud_to_be_saved_.reset(new PointCloudXYZI());

    if (base_frame_.empty())
    {
        activateSensorProcessing();
    }
    else
    {
        extrinsics_wait_started_ = std::chrono::steady_clock::now();
        retryBaseExtrinsics();
        if (!sensor_processing_active_)
        {
            extrinsics_retry_timer_ = create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&SPARKFastLIO2::retryBaseExtrinsics, this));
        }
    }

    if (lidar_processor_->pointFilterStride() != 1 && point_filter_num_ > 1)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Points may be too sparse. Set 'point_filter_num_for_preprocessing' to 1 and tune "
                    "'point_filter_num' instead.");
    }

    RCLCPP_INFO(this->get_logger(),
                "SPARKFastLIO2 constructed; lidar_qos=%s imu_qos=%s imu_qos_depth=%d "
                "process_on_callback=%d",
                reliabilityPolicyName(lidar_qos_reliability_),
                reliabilityPolicyName(imu_qos_reliability_),
                imu_qos_depth_,
                process_on_callback_ ? 1 : 0);
}

void SPARKFastLIO2::activateSensorProcessing()
{
    if (sensor_processing_active_)
    {
        return;
    }
    sensor_processing_active_ = true;

    rclcpp::QoS lidar_qos(rclcpp::KeepLast(static_cast<std::size_t>(lidar_qos_depth_)));
    lidar_qos.reliability(lidar_qos_reliability_);
    lidar_qos.durability_volatile();
    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar",
        lidar_qos,
        std::bind(&SPARKFastLIO2::standardLiDARCallback, this, std::placeholders::_1));

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    sub_lidar_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "lidar",
        lidar_qos,
        std::bind(&SPARKFastLIO2::livoxLiDARCallback, this, std::placeholders::_1));
#endif
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(imu_qos_depth_));
    imu_qos.reliability(imu_qos_reliability_);
    imu_qos.durability_volatile();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", imu_qos, std::bind(&SPARKFastLIO2::imuCallback, this, std::placeholders::_1));

    if (!process_on_callback_)
    {
        main_loop_timer_ = rclcpp::create_timer(
            this,
            get_clock(),
            rclcpp::Duration::from_nanoseconds(1000000),
            std::bind(&SPARKFastLIO2::main, this));
    }
}

void SPARKFastLIO2::retryBaseExtrinsics()
{
    if (sensor_processing_active_)
    {
        return;
    }

    std::string error;
    if (tryLookupBaseExtrinsics(lidar_translation_in_base_, lidar_rotation_in_base_, error))
    {
        if (extrinsics_retry_timer_)
        {
            extrinsics_retry_timer_->cancel();
        }
        activateSensorProcessing();
        RCLCPP_INFO(this->get_logger(), "Base extrinsics detected; sensor processing activated.");
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto waited = std::chrono::duration<double>(now - extrinsics_wait_started_).count();
    if (extrinsics_timeout_s_ > 0.0 && waited >= extrinsics_timeout_s_ &&
        !extrinsics_timeout_reported_)
    {
        extrinsics_timeout_reported_ = true;
        RCLCPP_ERROR(this->get_logger(),
                     "Base extrinsics unavailable after %.1f seconds; sensor processing remains disabled: %s",
                     waited,
                     error.c_str());
    }

    if (last_extrinsics_wait_log_.time_since_epoch().count() == 0 ||
        now - last_extrinsics_wait_log_ >= std::chrono::seconds(5))
    {
        last_extrinsics_wait_log_ = now;
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for transform from '%s' to '%s'; sensor processing is disabled: %s",
                    lidar_frame_.c_str(),
                    base_frame_.c_str(),
                    error.c_str());
    }
}

void SPARKFastLIO2::resetEstimatorState(const std::string &reason, const ResetMode mode)
{
    RCLCPP_WARN(this->get_logger(),
                "Resetting estimator state (%s): %s",
                mode == ResetMode::kWarmRecovery ? "warm recovery" : "cold",
                reason.c_str());

    // PX4 consumes the high-rate propagated odometry. Mark every front-end
    // reset so its EKF does not interpret the new local state as a measurement jump.
    ++odom_reset_counter_;

    // For kWarmRecovery, capture a re-init prior before wiping: a mid-run
    // reset can happen in flight, where the stationary initialization window
    // never comes. Gravity and velocity are converted to the IMU body frame so
    // they stay valid in the world frame the re-initialization will anchor at
    // the current body attitude. Whether the pre-reset state is trustworthy is
    // the CALLER's decision (via mode): timing glitches leave it valid, while
    // a corrupted-estimate reset must stay cold.
    bool have_warm_prior  = false;
    V3D warm_gravity_body = Zero3d;
    V3D warm_bg           = Zero3d;
    V3D warm_ba           = Zero3d;
    V3D warm_vel_body     = Zero3d;
    V3D warm_mean_acc     = Zero3d;
    // Gate on the IMU processor's own initialized flag, NOT the LiDAR warmup state:
    // the latter needs kInitializationTimeSec of post-reset data to flip true again, so a
    // burst of resets (e.g. LiDAR and IMU glitches back to back) inside that
    // window would be misjudged as a cold start and deadlock in flight.
    // Prefer the high-rate IMU propagation state when it is available. kf_
    // advances only with LiDAR processing, while kf_for_preintegration_ stays
    // current during rejected or missing LiDAR frames.
    if (mode == ResetMode::kWarmRecovery && imu_processor_ && imu_processor_->isInitialized())
    {
        const V3D mean_acceleration = imu_processor_->getSnapshot().mean_acceleration;
        state_ikfom prior_state = kf_.get_x();
        if (kf_for_preintegration_.has_value())
        {
            const state_ikfom propagated_state = kf_for_preintegration_->get_x();
            if (canSeedWarmRecovery(propagated_state, mean_acceleration))
            {
                prior_state = propagated_state;
            }
        }
        if (canSeedWarmRecovery(prior_state, mean_acceleration))
        {
            const V3D gravity_world(
                prior_state.grav[0], prior_state.grav[1], prior_state.grav[2]);
            warm_gravity_body = prior_state.rot.conjugate() * gravity_world;
            warm_bg           = prior_state.bg;
            warm_ba           = prior_state.ba;
            warm_vel_body     = prior_state.rot.conjugate() * prior_state.vel;
            warm_mean_acc     = mean_acceleration;
            have_warm_prior   = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Warm recovery state is invalid; falling back to cold initialization.");
        }
    }

    lidar_buffer_.clear();
    imu_buffer_.clear();
    imu_prediction_buffer_.clear();
    imu_integration_queue_.clear();
    imu_prediction_state_time_.reset();

    has_pending_lidar_frame_ = false;
    lidar_end_time_ = 0.0;
    mean_scan_duration_ = 0.0;
    scan_duration_sample_count_ = 0;
    imu_gap_lidar_skip_count_ = 0;
    first_lidar_time_ = 0.0;
    has_lidar_start_time_   = false;
    is_lio_warmup_complete_ = false;
    has_last_lidar_timestamp_ = false;
    has_last_imu_timestamp_ = false;
    last_consumed_imu_time_.reset();
    last_not_enough_imu_log_timestamp_ns_ = -1;

    full_points_->clear();
    sampled_points_->clear();
    feats_down_body_->clear();
    feats_down_world_->clear();
    cloud_to_be_saved_->clear();
    nearest_map_points_.clear();
    map_boxes_to_remove_.clear();

    PointVector empty_map;
    ikd_tree_.Build(empty_map);
    ikd_tree_.set_downsample_param(filter_size_map_min_);
    local_map_initialized_ = false;

    state_ikfom initial_state;
    kf_.change_x(initial_state);
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov initial_cov =
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov::Identity();
    kf_.change_P(initial_cov);
    kf_for_preintegration_.reset();
    imu_processor_->reset();
    if (have_warm_prior)
    {
        imu_processor_->setWarmStartPrior(
            warm_gravity_body, warm_bg, warm_ba, warm_vel_body, warm_mean_acc);
    }

    latest_state_ = initial_state;
    has_accepted_lio_update_   = false;
    last_accepted_lio_position_ = Zero3d;
    last_accepted_lio_time_ = 0.0;

    path_msg_.poses.clear();
    path_publish_counter_ = 0;
    consecutive_gate_reject_count_ = 0;
    total_residual_ = 0.0;
    mean_residual_ = 0.05;
    effective_feature_count_ = 0;
    downsampled_point_count_ = 0;
    lio_state_log_counter_ = 0;

    is_gravity_aligned_ = false;
    gravity_alignment_rotation_ = Eye3d;
    num_consecutive_moving_frames_ = 0;
    has_lidar_imu_time_offset_ = false;
    lidar_imu_time_offset_ = 0;
    global_gravity_directions_.clear();
    stationary_mean_acceleration_ = Zero3d;
}

// Outputs the rotation that aligns gravity_from to gravity_to.
M3D SPARKFastLIO2::computeRelativeRotation(const Eigen::Vector3d &gravity_from,
                                           const Eigen::Vector3d &gravity_to)
{
    Eigen::Vector3d normalized_gravity_from = gravity_from.normalized();
    Eigen::Vector3d normalized_gravity_to   = gravity_to.normalized();

    Eigen::Vector3d axis = normalized_gravity_from.cross(normalized_gravity_to);
    double cos_theta     = normalized_gravity_from.dot(normalized_gravity_to);

    if (std::fabs(1.0 - cos_theta) < 1e-3)
    {
        return Eigen::Matrix3d::Identity();
    }

    // Degenerate condition a = -b
    // Compute cross product with any arbitrary nonparallel vector,
    // i.e., Eigen::Vector3d(1, 2, 3)
    if (std::fabs(1.0 + cos_theta) < 1e-3)
    {
        Eigen::Vector3d perturbed = normalized_gravity_from + Eigen::Vector3d(1, 2, 3);
        axis = normalized_gravity_from.cross(perturbed);

        if (axis.norm() < 1e-6)
        {
            perturbed = normalized_gravity_from + Eigen::Vector3d(3, 2, 1);
            axis      = normalized_gravity_from.cross(perturbed);
        }

        axis.normalize();
        return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
    }
    else
    {
        axis.normalize();
        double theta = std::acos(cos_theta);

        Eigen::Quaterniond q(Eigen::AngleAxisd(theta, axis));

        return q.toRotationMatrix();
    }
}

bool SPARKFastLIO2::tryLookupBaseExtrinsics(V3D &lidar_translation_in_base,
                                            M3D &lidar_rotation_in_base,
                                            std::string &error)
{
    const auto lookup_time = rclcpp::Time(0);
    if (!tf_buffer_->canTransform(
            base_frame_, lidar_frame_, lookup_time, tf2::durationFromSec(0.0), &error))
    {
        return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, lookup_time);
    }
    catch (const tf2::TransformException &exception)
    {
        error = exception.what();
        return false;
    }

    lidar_translation_in_base(0)   = transform.transform.translation.x;
    lidar_translation_in_base(1)   = transform.transform.translation.y;
    lidar_translation_in_base(2)   = transform.transform.translation.z;

    Eigen::Quaterniond q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);

    lidar_rotation_in_base = q.toRotationMatrix();

    RCLCPP_INFO(this->get_logger(),
                "Translation: [%.3f, %.3f, %.3f]",
                lidar_translation_in_base(0),
                lidar_translation_in_base(1),
                lidar_translation_in_base(2));

    RCLCPP_INFO(this->get_logger(),
                "Rotation (Quaternion): [%.3f, %.3f, %.3f, %.3f]",
                q.x(),
                q.y(),
                q.z(),
                q.w());

    return true;
}

}  // namespace spark_fast_lio

RCLCPP_COMPONENTS_REGISTER_NODE(spark_fast_lio::SPARKFastLIO2)
