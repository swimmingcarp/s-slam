#include "spark_fast_lio.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace spark_fast_lio
{

#ifdef SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER
namespace
{
bool pointLessXYZICurvature(const PointType &lhs, const PointType &rhs)
{
    if (lhs.x != rhs.x)
    {
        return lhs.x < rhs.x;
    }
    if (lhs.y != rhs.y)
    {
        return lhs.y < rhs.y;
    }
    if (lhs.z != rhs.z)
    {
        return lhs.z < rhs.z;
    }
    if (lhs.intensity != rhs.intensity)
    {
        return lhs.intensity < rhs.intensity;
    }
    return lhs.curvature < rhs.curvature;
}
}  // namespace
#endif

struct SPARKFastLIO2::PropagationCheckpoint
{
    bool gravity_aligned = false;
    M3D gravity_rotation = Eye3d;
    std::deque<V3D> gravity_directions;
    V3D static_acceleration_mean = Zero3d;
    int moving_frame_count = 0;
    esekfom::esekf<state_ikfom, 12, input_ikfom> propagated_filter;
    ImuProcessor::Snapshot propagated_imu_snapshot;
};

struct SPARKFastLIO2::MotionQualityReport
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

SPARKFastLIO2::SPARKFastLIO2(const rclcpp::NodeOptions &options)
    : Node("spark_fast_lio_node", options),
      clock_(get_clock()),
      last_lidar_timestamp_(0, 0, RCL_ROS_TIME),
      last_imu_timestamp_(0, 0, RCL_ROS_TIME)
{
    xaxis_point_body_ << LIDAR_SP_LEN, 0.0, 0.0;
    xaxis_point_world_ << LIDAR_SP_LEN, 0.0, 0.0;
    base_gravity_                 = Zero3d;
    stationary_mean_acceleration_ = Zero3d;
    position_last_                = Zero3d;
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

    map_file_path_ = declare_parameter<std::string>("map_file_path", "");
    save_dir_      = declare_parameter<std::string>("common.save_dir", "");
    sequence_name_ = declare_parameter<std::string>("common.sequence_name", "");
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

    filter_size_map_min_ = declare_parameter<double>("filter_size_map", 0.5);
    local_map_side_length_      = declare_parameter<double>("cube_side_length", 200.0);
    detection_range_             = declare_parameter<double>("mapping.det_range", 300.0);
    fov_deg_                     = declare_parameter<double>("mapping.fov_degree", 360.0);
    gyroscope_covariance_        = declare_parameter<double>("mapping.gyr_cov", 0.1);
    accelerometer_covariance_    = declare_parameter<double>("mapping.acc_cov", 0.1);
    gyroscope_bias_covariance_   = declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    accelerometer_bias_covariance_ = declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    motion_quality_gate_enabled_ =
        declare_parameter<bool>("mapping.motion_quality_gate_enabled", false);
    motion_gate_max_pre_grav_residual_ =
        declare_parameter<double>("mapping.motion_gate_max_pre_grav_residual", 3.0);
    motion_gate_suspect_frame_step_ =
        declare_parameter<double>("mapping.motion_gate_suspect_frame_step", 0.5);
    motion_gate_max_update_step_ =
        declare_parameter<double>("mapping.motion_gate_max_update_step", 0.15);
    motion_gate_max_update_step_ratio_ =
        declare_parameter<double>("mapping.motion_gate_max_update_step_ratio", 0.2);
    motion_gate_min_effective_ratio_ =
        declare_parameter<double>("mapping.motion_gate_min_effective_ratio", 0.25);
    motion_gate_min_effective_features_ =
        declare_parameter<int>("mapping.motion_gate_min_effective_features", 100);
    motion_gate_reject_weak_lidar_ =
        declare_parameter<bool>("mapping.motion_gate_reject_weak_lidar", true);
    motion_gate_max_pre_grav_residual_ = std::max(0.1, motion_gate_max_pre_grav_residual_);
    motion_gate_suspect_frame_step_ = std::max(0.1, motion_gate_suspect_frame_step_);
    motion_gate_max_update_step_ = std::max(0.01, motion_gate_max_update_step_);
    motion_gate_max_update_step_ratio_ =
        std::clamp(motion_gate_max_update_step_ratio_, 0.0, 1.0);
    motion_gate_min_effective_ratio_ = std::clamp(motion_gate_min_effective_ratio_, 0.0, 1.0);
    motion_gate_min_effective_features_ = std::max(1, motion_gate_min_effective_features_);

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

    runtime_pos_log_       = declare_parameter<bool>("runtime_pos_log_enabled", false);
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

    rclcpp::QoS lidar_qos(rclcpp::KeepLast(static_cast<std::size_t>(lidar_qos_depth_)));
    lidar_qos.reliable();
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
    imu_qos.reliable();
    imu_qos.durability_volatile();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", imu_qos, std::bind(&SPARKFastLIO2::imuCallback, this, std::placeholders::_1));

    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    pub_cloud_full_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", qos);
    pub_cloud_lidar_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_lidar", qos);
    pub_cloud_body_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", qos);
    pub_cloud_base_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_base", qos);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
    pub_imu_predicted_odom_ =
        create_publisher<nav_msgs::msg::Odometry>("odometry_imu_predicted", qos);
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

    if (!base_frame_.empty())
    {
        if (!lookupBaseExtrinsics(lidar_translation_in_base_, lidar_rotation_in_base_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform.");
            return;
        }
    }

    if (!process_on_callback_)
    {
        main_loop_timer_ = rclcpp::create_timer(
            this,
            get_clock(),
            rclcpp::Duration::from_nanoseconds(1000000),
            std::bind(&SPARKFastLIO2::main, this));
    }

    if (lidar_processor_->pointFilterStride() != 1 && point_filter_num_ > 1)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Points may be too sparse. Set 'point_filter_num_for_preprocessing' to 1 and tune "
                    "'point_filter_num' instead.");
    }

    RCLCPP_INFO(this->get_logger(),
                "SPARKFastLIO2 constructed; imu_qos_depth=%d reliable=true process_on_callback=%d",
                imu_qos_depth_,
                process_on_callback_ ? 1 : 0);
}

void SPARKFastLIO2::resetEstimatorState(const std::string &reason, const ResetMode mode)
{
    RCLCPP_WARN(this->get_logger(),
                "Resetting estimator state (%s): %s",
                mode == ResetMode::kWarmRecovery ? "warm recovery" : "cold",
                reason.c_str());

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
    // Gate on the IMU processor's own initialized flag, NOT filter_initialized_:
    // the latter needs kInitializationTimeSec of post-reset data to flip true again, so a
    // burst of resets (e.g. LiDAR and IMU glitches back to back) inside that
    // window would be misjudged as a cold start and deadlock in flight. The
    // live filter state is used as the fallback because latest_state_ is only
    // refreshed by completed update cycles.
    if (mode == ResetMode::kWarmRecovery && imu_processor_ && imu_processor_->isInitialized())
    {
        const state_ikfom prior_state =
            have_last_good_state_ ? last_good_state_ : kf_.get_x();
        const V3D gravity_world(
            prior_state.grav[0], prior_state.grav[1], prior_state.grav[2]);
        if (gravity_world.norm() > 0.5 * G_m_s2)
        {
            warm_gravity_body = prior_state.rot.conjugate() * gravity_world;
            warm_bg           = prior_state.bg;
            warm_ba           = prior_state.ba;
            warm_vel_body     = prior_state.rot.conjugate() * prior_state.vel;
            warm_mean_acc     = imu_processor_->getSnapshot().mean_acceleration;
            have_warm_prior   = true;
        }
    }

    lidar_buffer_.clear();
    time_buffer_.clear();
    lidar_end_time_buffer_.clear();
    imu_buffer_.clear();
    imu_integration_queue_.clear();

    lidar_pushed_ = false;
    lidar_end_time_ = 0.0;
    mean_scan_duration_ = 0.0;
    scan_duration_sample_count_ = 0;
    imu_gap_lidar_skip_count_ = 0;
    first_lidar_time_ = 0.0;
    is_first_lidar_scan_ = true;
    filter_initialized_ = false;
    has_last_lidar_timestamp_ = false;
    has_last_imu_timestamp_ = false;
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
    removed_map_point_count_ = 0;
    inserted_point_count_ = 0;
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
    last_good_state_ = initial_state;
    have_last_good_state_ = false;
    last_good_imu_processor_snapshot_.reset();
    have_last_lio_debug_state_ = false;
    last_lio_debug_pos_ = Zero3d;
    last_lio_debug_time_ = 0.0;

    path_msg_.poses.clear();
    path_publish_counter_ = 0;
    motion_gate_consecutive_reject_count_ = 0;
    total_residual_ = 0.0;
    mean_residual_ = 0.05;
    effective_feature_count_ = 0;
    downsampled_point_count_ = 0;
    scan_count_ = 0;
    lio_state_log_counter_ = 0;

    is_gravity_aligned_ = false;
    gravity_alignment_rotation_ = Eye3d;
    num_consecutive_moving_frames_ = 0;
    time_offset_initialized_ = false;
    lidar_imu_time_offset_ = 0;
    global_gravity_directions_.clear();
    stationary_mean_acceleration_ = Zero3d;
    position_last_ = Zero3d;
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

bool SPARKFastLIO2::lookupBaseExtrinsics(V3D &lidar_translation_in_base, M3D &lidar_rotation_in_base)
{
    RCLCPP_INFO(this->get_logger(),
                "Looking up transform from %s -> %s",
                base_frame_.c_str(),
                lidar_frame_.c_str());

    const auto lookup_time = rclcpp::Time(0);
    bool has_transform     = false;
    std::string err_str;
    auto start_time          = this->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(extrinsics_timeout_s_);
    rclcpp::Rate rate(10.0);  // Just 10 Hz works

    while (rclcpp::ok())
    {
        if (tf_buffer_->canTransform(
                base_frame_, lidar_frame_, lookup_time, tf2::durationFromSec(0.0), &err_str))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32mExtrinsics detected.\033[1;0m");
            has_transform = true;
            break;
        }

        const auto time_since_start = now() - start_time;
        if (extrinsics_timeout_s_ > 0.0 && time_since_start > timeout)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Timeout after "
                                    << timeout.seconds() << " seconds waiting for transform from '"
                                    << lidar_frame_ << "' to '" << base_frame_ << "': " << err_str);
            break;
        }

        RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),
                                              *clock_,
                                              5000,
                                              "Waiting for transform from '"
                                                  << lidar_frame_ << "' to '" << base_frame_
                                                  << "': " << err_str);

        rate.sleep();
    }

    if (!has_transform)
    {
        return has_transform;
    }

    const auto &transform = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, lookup_time);
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

    return has_transform;
}

void SPARKFastLIO2::pointBodyToWorld(PointType const *const input_point,
                                     PointType *const output_point,
                                     const state_ikfom &state)
{
    *output_point = *input_point;
    V3D point_in_body(input_point->x, input_point->y, input_point->z);
    V3D point_in_world(
        state.rot * (state.offset_R_L_I * point_in_body + state.offset_T_L_I) + state.pos);

    output_point->x = point_in_world(0);
    output_point->y = point_in_world(1);
    output_point->z = point_in_world(2);
}

void SPARKFastLIO2::pclPointBodyToWorld(PointType const *const input_point,
                                        PointType *const output_point)
{
    *output_point = *input_point;
    V3D point_in_body(input_point->x, input_point->y, input_point->z);
    V3D point_in_world(latest_state_.rot *
                           (latest_state_.offset_R_L_I * point_in_body +
                            latest_state_.offset_T_L_I) +
                       latest_state_.pos);

    output_point->x = point_in_world(0);
    output_point->y = point_in_world(1);
    output_point->z = point_in_world(2);
}

void SPARKFastLIO2::pclPointBodyLidarToIMU(PointType const *const input_point,
                                           PointType *const output_point)
{
    *output_point = *input_point;
    V3D point_in_lidar(input_point->x, input_point->y, input_point->z);
    V3D point_in_imu(latest_state_.offset_R_L_I * point_in_lidar + latest_state_.offset_T_L_I);

    output_point->x = point_in_imu(0);
    output_point->y = point_in_imu(1);
    output_point->z = point_in_imu(2);
}

void SPARKFastLIO2::pclPointBodyLidarToBase(PointType const *const input_point,
                                            PointType *const output_point)
{
    *output_point = *input_point;
    V3D point_in_lidar(input_point->x, input_point->y, input_point->z);
    V3D point_in_base(lidar_rotation_in_base_ * point_in_lidar + lidar_translation_in_base_);

    output_point->x = point_in_base(0);
    output_point->y = point_in_base(1);
    output_point->z = point_in_base(2);
}

void SPARKFastLIO2::pclPointIMUToLiDAR(PointType const *const input_point,
                                       PointType *const output_point)
{
    *output_point = *input_point;
    V3D point_in_imu(input_point->x, input_point->y, input_point->z);
    V3D point_in_lidar(
        latest_state_.offset_R_L_I.inverse() * (point_in_imu - latest_state_.offset_T_L_I));

    output_point->x = point_in_lidar(0);
    output_point->y = point_in_lidar(1);
    output_point->z = point_in_lidar(2);
}

void SPARKFastLIO2::pclPointIMUToBase(PointType const *const input_point,
                                      PointType *const output_point)
{
    *output_point = *input_point;
    const Eigen::Matrix3d offset_R_B_I = latest_state_.offset_R_L_I * lidar_rotation_in_base_.inverse();
    const Eigen::Vector3d offset_T_B_I =
        -1 * offset_R_B_I * lidar_translation_in_base_ + latest_state_.offset_T_L_I;

    V3D point_in_imu(input_point->x, input_point->y, input_point->z);
    V3D point_in_base(offset_R_B_I.inverse() * (point_in_imu - offset_T_B_I));

    output_point->x = point_in_base(0);
    output_point->y = point_in_base(1);
    output_point->z = point_in_base(2);
}

void SPARKFastLIO2::collectRemovedPoints()
{
    PointVector points_history;
    ikd_tree_.acquire_removed_points(points_history);
}

void SPARKFastLIO2::standardLiDARCallback(const sensor_msgs::msg::PointCloud2 &msg)
{
    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);
        ++scan_count_;
        rclcpp::Time msg_time = msg.header.stamp;
        double msg_end_time   = 0.0;

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        if (!lidar_processor_->process(msg, ptr))
        {
            return;
        }

        if (lidar_processor_->hasScanTime())
        {
            msg_time     = rclcpp::Time(lidar_processor_->scanStartTime() * 1e9);
            msg_end_time = lidar_processor_->scanEndTime();
        }

        if (has_last_lidar_timestamp_ && msg_time < last_lidar_timestamp_)
        {
            resetEstimatorState("LiDAR timestamp moved backwards", ResetMode::kWarmRecovery);
        }
        last_lidar_timestamp_ = msg_time;
        has_last_lidar_timestamp_ = true;

        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(msg_time.seconds());
        lidar_end_time_buffer_.push_back(msg_end_time);
    }

    if (process_on_callback_)
    {
        processPendingMeasurements();
    }
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void SPARKFastLIO2::livoxLiDARCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);
        ++scan_count_;
        rclcpp::Time msg_time = msg->header.stamp;

        if (has_last_lidar_timestamp_ && msg_time < last_lidar_timestamp_)
        {
            resetEstimatorState("Livox timestamp moved backwards", ResetMode::kWarmRecovery);
        }
        last_lidar_timestamp_ = msg_time;
        has_last_lidar_timestamp_ = true;

        const auto diff_s = std::abs((last_imu_timestamp_ - last_lidar_timestamp_).seconds());
        if (!time_sync_enabled_ && diff_s > 10.0 && !imu_buffer_.empty() && !lidar_buffer_.empty())
        {
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "IMU and LiDAR not Synced, IMU time: "
                                   << last_imu_timestamp_.nanoseconds()
                                   << ", lidar header time: " << last_lidar_timestamp_.nanoseconds());
        }

        if (time_sync_enabled_ && !time_offset_initialized_ && diff_s > 1.0 && !imu_buffer_.empty())
        {
            time_offset_initialized_           = true;
            lidar_imu_time_offset_ =
                last_lidar_timestamp_.nanoseconds() + static_cast<int64_t>(1.0e8) -
                last_imu_timestamp_.nanoseconds();
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Self sync IMU and LiDAR, time diff is " << lidar_imu_time_offset_ << "[ns]");
        }

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        if (!lidar_processor_->process(*msg, ptr))
        {
            return;
        }

        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(msg_time.seconds());
        lidar_end_time_buffer_.push_back(0.0);
    }

    if (process_on_callback_)
    {
        processPendingMeasurements();
    }
}
#endif

void SPARKFastLIO2::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    rclcpp::Time stamp = msg->header.stamp;
    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);

        auto imu_input = std::make_shared<sensor_msgs::msg::Imu>(*msg);
        if (time_sync_enabled_ && std::abs(lidar_imu_time_offset_) > static_cast<int64_t>(1.0e8))
        {
            stamp += rclcpp::Duration::from_nanoseconds(lidar_imu_time_offset_);
            imu_input->header.stamp = stamp;
        }

        if (has_last_imu_timestamp_ && stamp < last_imu_timestamp_)
        {
            std::stringstream reason;
            reason << "IMU timestamp moved backwards (previous: "
                   << last_imu_timestamp_.nanoseconds()
                   << " vs. received: " << stamp.nanoseconds() << " [ns])";
            resetEstimatorState(reason.str(), ResetMode::kWarmRecovery);
        }
        last_imu_timestamp_ = stamp;
        has_last_imu_timestamp_ = true;

        if (imu_predicted_odometry_enabled_ && kf_for_preintegration_.has_value())
        {
            integrateIMU(*kf_for_preintegration_, *imu_input);
        }

        imu_buffer_.push_back(imu_input);
    }

    if (process_on_callback_)
    {
        processPendingMeasurements();
    }
}

void SPARKFastLIO2::integrateIMU(esekfom::esekf<state_ikfom, 12, input_ikfom> &state,
                                 const sensor_msgs::msg::Imu &msg)
{
    imu_integration_queue_.push_back(msg);

    if (imu_integration_queue_.size() < 2)
    {
        return;
    }

    // Assume that timestamps are sufficiently close and ascending order
    const double delta_time = rclcpp::Time(imu_integration_queue_[1].header.stamp).seconds() -
                              rclcpp::Time(imu_integration_queue_[0].header.stamp).seconds();

    if (delta_time <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "IMU timestamps must be in ascending order!");
        imu_integration_queue_.pop_front();
        return;
    }

    auto integrated_state = imu_processor_->integrateImu(imu_integration_queue_, state);
    const auto &stamp     = imu_integration_queue_[1].header.stamp;
    imu_integration_queue_.pop_front();

    integrated_state.pos = gravity_alignment_rotation_ * integrated_state.pos;
    integrated_state.rot = gravity_alignment_rotation_ * integrated_state.rot;

    publishOdometry(integrated_state, stamp, pub_imu_predicted_odom_, false);
}

void SPARKFastLIO2::computeMeasurementModel(
    state_ikfom &state,
    esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    selected_points_->clear();
    selected_normals_->clear();
    selected_points_->reserve(downsampled_point_count_);
    selected_normals_->reserve(downsampled_point_count_);
    if (static_cast<int>(point_residuals_.size()) < downsampled_point_count_)
    {
        point_residuals_.resize(downsampled_point_count_, 0.0f);
    }
    if (static_cast<int>(surface_point_selected_.size()) < downsampled_point_count_)
    {
        surface_point_selected_.resize(downsampled_point_count_, 0U);
    }
    total_residual_ = 0.0;
    int nearest_candidate_num = 0;
    int plane_candidate_num   = 0;

    /** closest surface search and residual computation **/
#ifdef MP_ENABLED
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for reduction(+ : nearest_candidate_num, plane_candidate_num)
#endif
    for (int i = 0; i < downsampled_point_count_; ++i)
    {
        PointType &point_body  = feats_down_body_->points[i];
        PointType &point_world = feats_down_world_->points[i];

        /* transform to world frame */
        V3D point_in_body(point_body.x, point_body.y, point_body.z);
        V3D point_in_world(
            state.rot * (state.offset_R_L_I * point_in_body + state.offset_T_L_I) + state.pos);
        point_world.x         = point_in_world(0);
        point_world.y         = point_in_world(1);
        point_world.z         = point_in_world(2);
        point_world.intensity = point_body.intensity;

        auto &points_near = nearest_map_points_[i];
        thread_local std::vector<float> point_search_sq_dis;

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikd_tree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, point_search_sq_dis);
            surface_point_selected_[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                      : point_search_sq_dis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                   : true;
        }

        if (!surface_point_selected_[i])
        {
            continue;
        }
        ++nearest_candidate_num;

        VF(4) plane_coefficients;
        surface_point_selected_[i] = false;
        if (esti_plane(plane_coefficients, points_near, 0.1f))
        {
            ++plane_candidate_num;
            const float point_to_plane_distance =
                plane_coefficients(0) * point_world.x +
                plane_coefficients(1) * point_world.y +
                plane_coefficients(2) * point_world.z + plane_coefficients(3);
            const float plane_score =
                1 - 0.9f * fabs(point_to_plane_distance) / sqrt(point_in_body.norm());

            if (plane_score > 0.9f)
            {
                surface_point_selected_[i]   = true;
                point_normals_->points[i].x  = plane_coefficients(0);
                point_normals_->points[i].y  = plane_coefficients(1);
                point_normals_->points[i].z  = plane_coefficients(2);
                point_normals_->points[i].intensity = point_to_plane_distance;
                point_residuals_[i] = abs(point_to_plane_distance);
            }
        }
    }

    effective_feature_count_ = 0;

    for (int i = 0; i < downsampled_point_count_; ++i)
    {
        if (surface_point_selected_[i])
        {
            selected_points_->push_back(feats_down_body_->points[i]);
            selected_normals_->push_back(point_normals_->points[i]);
            total_residual_ += point_residuals_[i];
            ++effective_feature_count_;
        }
    }

    if (effective_feature_count_ < 1)
    {
        ekfom_data.valid = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "No Effective Points! feats_down=%d nearest_candidates=%d "
                             "plane_candidates=%d tree=%d pos=[%.3f, %.3f, %.3f] "
                             "vel=[%.3f, %.3f, %.3f]",
                             downsampled_point_count_,
                             nearest_candidate_num,
                             plane_candidate_num,
                             ikd_tree_.size(),
                             state.pos[0],
                             state.pos[1],
                             state.pos[2],
                             state.vel[0],
                             state.vel[1],
                             state.vel[2]);
        return;
    }

    mean_residual_ = total_residual_ / effective_feature_count_;
    match_time_ += omp_get_wtime() - match_start;
    double solve_start = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effective_feature_count_, 12);  // 23
    ekfom_data.h.resize(effective_feature_count_);

    for (int i = 0; i < effective_feature_count_; ++i)
    {
        const PointType &selected_point = selected_points_->points[i];
        V3D point_in_lidar(selected_point.x, selected_point.y, selected_point.z);
        M3D lidar_point_cross;
        lidar_point_cross << SKEW_SYM_MATRX(point_in_lidar);
        V3D point_in_imu = state.offset_R_L_I * point_in_lidar + state.offset_T_L_I;
        M3D imu_point_cross;
        imu_point_cross << SKEW_SYM_MATRX(point_in_imu);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &normal_point = selected_normals_->points[i];
        V3D normal(normal_point.x, normal_point.y, normal_point.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D rotated_normal(state.rot.conjugate() * normal);
        V3D position_jacobian(imu_point_cross * rotated_normal);
        if (extrinsic_est_enabled_)
        {
            V3D extrinsic_rotation_jacobian(
                lidar_point_cross * state.offset_R_L_I.conjugate() * rotated_normal);
            ekfom_data.h_x.block<1, 12>(i, 0)
                << normal_point.x, normal_point.y, normal_point.z,
                VEC_FROM_ARRAY(position_jacobian), VEC_FROM_ARRAY(extrinsic_rotation_jacobian),
                VEC_FROM_ARRAY(rotated_normal);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0)
                << normal_point.x, normal_point.y, normal_point.z,
                VEC_FROM_ARRAY(position_jacobian), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -normal_point.intensity;
    }
    solve_time_ += omp_get_wtime() - solve_start;
}

void SPARKFastLIO2::updateLocalMapWindow()
{
    map_boxes_to_remove_.clear();
    removed_map_point_count_ = 0;
    map_removal_time_        = 0.0;
    pointBodyToWorld(xaxis_point_body_, xaxis_point_world_, latest_state_);

    const V3D lidar_position = kf_.get_lidar_position();
    if (!local_map_initialized_)
    {
        for (int i = 0; i < 3; ++i)
        {
            local_map_bounds_.vertex_min[i] = lidar_position(i) - local_map_side_length_ / 2.0;
            local_map_bounds_.vertex_max[i] = lidar_position(i) + local_map_side_length_ / 2.0;
        }
        local_map_initialized_ = true;
        return;
    }

    float distance_to_map_edge[3][2];
    bool should_move_window = false;
    for (int i = 0; i < 3; ++i)
    {
        distance_to_map_edge[i][0] = fabs(lidar_position(i) - local_map_bounds_.vertex_min[i]);
        distance_to_map_edge[i][1] = fabs(lidar_position(i) - local_map_bounds_.vertex_max[i]);
        if (distance_to_map_edge[i][0] <= kMapMoveThreshold * detection_range_ ||
            distance_to_map_edge[i][1] <= kMapMoveThreshold * detection_range_)
        {
            should_move_window = true;
        }
    }
    if (!should_move_window)
    {
        return;
    }
    BoxPointType new_map_bounds, removed_bounds;
    new_map_bounds = local_map_bounds_;
    const float move_distance =
        max((local_map_side_length_ - 2.0 * kMapMoveThreshold * detection_range_) * 0.5 * 0.9,
            static_cast<double>(detection_range_ * (kMapMoveThreshold - 1)));
    for (int i = 0; i < 3; ++i)
    {
        removed_bounds = local_map_bounds_;
        if (distance_to_map_edge[i][0] <= kMapMoveThreshold * detection_range_)
        {
            new_map_bounds.vertex_max[i] -= move_distance;
            new_map_bounds.vertex_min[i] -= move_distance;
            removed_bounds.vertex_min[i] = local_map_bounds_.vertex_max[i] - move_distance;
            map_boxes_to_remove_.push_back(removed_bounds);
        }
        else if (distance_to_map_edge[i][1] <= kMapMoveThreshold * detection_range_)
        {
            new_map_bounds.vertex_max[i] += move_distance;
            new_map_bounds.vertex_min[i] += move_distance;
            removed_bounds.vertex_max[i] = local_map_bounds_.vertex_min[i] + move_distance;
            map_boxes_to_remove_.push_back(removed_bounds);
        }
    }
    local_map_bounds_ = new_map_bounds;

    collectRemovedPoints();
    double delete_begin = omp_get_wtime();
    if (map_boxes_to_remove_.size() > 0)
    {
        removed_map_point_count_ = ikd_tree_.Delete_Point_Boxes(map_boxes_to_remove_);
    }
    map_removal_time_ = omp_get_wtime() - delete_begin;
}

void SPARKFastLIO2::insertScanIntoMap()
{
    PointVector points_to_insert;
    PointVector points_to_insert_without_downsampling;
    points_to_insert.reserve(downsampled_point_count_);
    points_to_insert_without_downsampling.reserve(downsampled_point_count_);

    for (int i = 0; i < downsampled_point_count_; ++i)
    {
        // transform to world frame
        pointBodyToWorld(
            &(feats_down_body_->points[i]), &(feats_down_world_->points[i]), latest_state_);

        // decide if we need to add to map
        if (!nearest_map_points_[i].empty() && filter_initialized_)
        {
            const PointVector &points_near = nearest_map_points_[i];
            bool should_insert             = true;

            PointType voxel_center{};
            voxel_center.x = std::floor(feats_down_world_->points[i].x / filter_size_map_min_) *
                                 filter_size_map_min_ +
                             0.5 * filter_size_map_min_;
            voxel_center.y = std::floor(feats_down_world_->points[i].y / filter_size_map_min_) *
                                 filter_size_map_min_ +
                             0.5 * filter_size_map_min_;
            voxel_center.z = std::floor(feats_down_world_->points[i].z / filter_size_map_min_) *
                                 filter_size_map_min_ +
                             0.5 * filter_size_map_min_;

            const float point_to_voxel_center_distance =
                calc_dist(feats_down_world_->points[i], voxel_center);
            if (std::fabs(points_near[0].x - voxel_center.x) > 0.5f * filter_size_map_min_ &&
                std::fabs(points_near[0].y - voxel_center.y) > 0.5f * filter_size_map_min_ &&
                std::fabs(points_near[0].z - voxel_center.z) > 0.5f * filter_size_map_min_)
            {
                points_to_insert_without_downsampling.push_back(feats_down_world_->points[i]);
                continue;
            }

            for (int neighbor_index = 0; neighbor_index < NUM_MATCH_POINTS; ++neighbor_index)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                {
                    break;
                }
                if (calc_dist(points_near[neighbor_index], voxel_center) <
                    point_to_voxel_center_distance)
                {
                    should_insert = false;
                    break;
                }
            }
            if (should_insert)
            {
                points_to_insert.push_back(feats_down_world_->points[i]);
            }
        }
        else
        {
            // No nearest map points, or the filter has not initialized yet.
            points_to_insert.push_back(feats_down_world_->points[i]);
        }
    }

#ifdef SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER
    std::sort(points_to_insert.begin(), points_to_insert.end(), pointLessXYZICurvature);
    std::sort(points_to_insert_without_downsampling.begin(),
              points_to_insert_without_downsampling.end(),
              pointLessXYZICurvature);
#endif

    const double insertion_start_time = omp_get_wtime();
    inserted_point_count_ = ikd_tree_.Add_Points(points_to_insert, true);
    ikd_tree_.Add_Points(points_to_insert_without_downsampling, false);

    inserted_point_count_ =
        points_to_insert.size() + points_to_insert_without_downsampling.size();
    map_insertion_time_ = omp_get_wtime() - insertion_start_time;
}

void SPARKFastLIO2::publishOdometry(const state_ikfom &state, const rclcpp::Time &stamp)
{
    publishOdometry(state, stamp, pub_odom_, true);
}

void SPARKFastLIO2::publishOdometry(
    const state_ikfom &state,
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

    // fill the covariance
    auto P = kf_.get_P();
    for (int i = 0; i < 6; ++i)
    {
        int k                                     = (i < 3) ? (i + 3) : (i - 3);
        odomAftMapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped_.pose.covariance[i * 6 + 5] = P(k, 2);
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
    publishOdometry(state, stamp);

    if (insert_into_map)
    {
        insertScanIntoMap();
    }

    if (append_path && path_enabled_)
    {
        publishPath(state);
    }

    if (!scan_publish_enabled_)
    {
        return;
    }

    publishMapScan(pub_cloud_full_);
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud)
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
        // allocate world frames
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        PointCloudXYZI::Ptr laserCloudTmp(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; ++i)
        {
            if (viz_frame_ == "imu")
            {
                pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
            }
            else if (viz_frame_ == "lidar")
            {
                pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudTmp->points[i]);
                pclPointIMUToLiDAR(&laserCloudTmp->points[i], &laserCloudWorld->points[i]);
            }
            else if (viz_frame_ == "base")
            {
                pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudTmp->points[i]);
                pclPointIMUToBase(&laserCloudTmp->points[i], &laserCloudWorld->points[i]);
            }
            else
            {
                throw std::invalid_argument("Invalid visualization frame has been given");
            }
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
            pclPointBodyToWorld(&full_points_->points[i], &laserCloudWorld2->points[i]);
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
    // offset_A_B: transformation matrix of A w.r.t. B
    Eigen::Vector3d lidar_position =
        state.offset_R_L_I.inverse() *
        (state.rot * state.offset_T_L_I + state.pos - state.offset_T_L_I);

    Eigen::Quaterniond lidar_orientation =
        state.offset_R_L_I.inverse() * state.rot * state.offset_R_L_I;

    PoseStruct output;
    output.position_    = lidar_position;
    output.orientation_ = lidar_orientation;
    return output;
}

void SPARKFastLIO2::main()
{
    processPendingMeasurements();
}

void SPARKFastLIO2::processPendingMeasurements()
{
    while (syncPackages(measures_, verbose_))
    {
        processLidarAndImu(measures_);
    }
}

PoseStruct SPARKFastLIO2::transformPoseToBaseFrame(const state_ikfom &state) const
{
    const Eigen::Matrix3d offset_R_B_I = state.offset_R_L_I * lidar_rotation_in_base_.inverse();
    const Eigen::Vector3d offset_T_B_I =
        -offset_R_B_I * lidar_translation_in_base_ + state.offset_T_L_I;

    Eigen::Vector3d base_position =
        offset_R_B_I.inverse() * (state.rot * offset_T_B_I + state.pos - offset_T_B_I);

    Eigen::Quaterniond base_orientation =
        Eigen::Quaterniond(offset_R_B_I.inverse() * state.rot * offset_R_B_I);

    PoseStruct output;
    output.position_    = base_position;
    output.orientation_ = base_orientation;
    return output;
}

bool SPARKFastLIO2::syncPackages(MeasureGroup &measurements, bool verbose)
{
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    if (verbose)
    {
        const std::size_t lidar_buffer_size = lidar_buffer_.size();
        const std::size_t imu_buffer_size   = imu_buffer_.size();

        // To only print out when changes occur
        if (verbose_lidar_buffer_size_ != lidar_buffer_size ||
            verbose_imu_buffer_size_ != imu_buffer_size)
        {
            RCLCPP_INFO(this->get_logger(), "%zu vs. %zu", lidar_buffer_size, imu_buffer_size);
            verbose_lidar_buffer_size_ = lidar_buffer_size;
            verbose_imu_buffer_size_   = imu_buffer_size;
        }
    }

    if (lidar_buffer_.empty() || imu_buffer_.empty())
    {
        return false;
    }

    if (!lidar_pushed_)
    {
        measurements.lidar          = lidar_buffer_.front();
        measurements.lidar_beg_time = time_buffer_.front();
        const double msg_end_time =
            lidar_end_time_buffer_.empty() ? 0.0 : lidar_end_time_buffer_.front();
        constexpr double kPointTimeOffsetScale = 1000.0;

        // RoboSense filtering can drop late-scan points; prefer the raw scan end timestamp.
        if (msg_end_time > measurements.lidar_beg_time)
        {
            lidar_end_time_ = msg_end_time;
            const double scan_duration = lidar_end_time_ - measurements.lidar_beg_time;
            const double expected_scan_time_ms =
                1000.0 / static_cast<double>(lidar_processor_->scanRateHz());
            const double scan_duration_ms = scan_duration * 1000.0;
            if (scan_duration_ms < 0.8 * expected_scan_time_ms ||
                scan_duration_ms > 1.2 * expected_scan_time_ms)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "LiDAR scan duration (%.2f ms) should be close to %.2f ms. Please check "
                    "the point timestamp field from your sensor.",
                    scan_duration_ms,
                    expected_scan_time_ms);
            }
            ++scan_duration_sample_count_;
            mean_scan_duration_ +=
                (scan_duration - mean_scan_duration_) / static_cast<double>(scan_duration_sample_count_);
        }
        else if (measurements.lidar->points.size() <= 1)
        {
            lidar_end_time_ = measurements.lidar_beg_time + mean_scan_duration_;
        }
        else if (measurements.lidar->points.back().curvature / kPointTimeOffsetScale <
                 0.5 * mean_scan_duration_)
        {
            lidar_end_time_ = measurements.lidar_beg_time + mean_scan_duration_;
        }
        else
        {
            ++scan_duration_sample_count_;
            if (measurements.lidar->points.back().curvature < 80 ||
                measurements.lidar->points.back().curvature > 120)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Point time offset (%.2f) should be close to 100. Please "
                    "check the `timestamp_unit` "
                    "or values of `time` (or `t`) field of the point cloud input from your sensor.",
                    measurements.lidar->points.back().curvature);
            }

            const double scan_duration = measurements.lidar->points.back().curvature / 1000.0;
            lidar_end_time_           = measurements.lidar_beg_time + scan_duration;
            mean_scan_duration_ +=
                (scan_duration - mean_scan_duration_) / static_cast<double>(scan_duration_sample_count_);
        }
        measurements.lidar_end_time = lidar_end_time_;
        lidar_pushed_              = true;
    }

    if (last_imu_timestamp_.seconds() < lidar_end_time_)
    {
        if (verbose)
        {
            // To only print out when changes occur
            if (last_not_enough_imu_log_timestamp_ns_ != last_imu_timestamp_.nanoseconds())
            {
                RCLCPP_INFO(this->get_logger(),
                            "Not enough IMU data (%.6f < %.6f)",
                            last_imu_timestamp_.seconds(),
                            lidar_end_time_);
                last_not_enough_imu_log_timestamp_ns_ = last_imu_timestamp_.nanoseconds();
            }
        }
        return false;
    }

    const auto consume_lidar = [&]()
    {
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        if (!lidar_end_time_buffer_.empty())
        {
            lidar_end_time_buffer_.pop_front();
        }
        lidar_pushed_ = false;
    };

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
    measurements.imu.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
    {
        imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
        if (imu_time > lidar_end_time_)
        {
            break;
        }
        measurements.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    if (measurements.imu.empty())
    {
        // The next available IMU is already after this scan. The scan cannot be
        // undistorted safely, so do not pass an empty IMU package to ESKF.
        ++imu_gap_lidar_skip_count_;
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "Skipping LiDAR scan due to IMU coverage gap: count=%d "
                             "lidar=[%.6f, %.6f] next_imu=%.6f",
                             imu_gap_lidar_skip_count_,
                             measurements.lidar_beg_time,
                             lidar_end_time_,
                             imu_time);
        consume_lidar();
        return false;
    }

    consume_lidar();

    return true;
}

bool SPARKFastLIO2::isMotionStopped(const V3D &acc_ref,
                                    const V3D &acc_curr,
                                    const double acc_diff_thr)
{
    return (acc_ref - acc_curr).norm() <= acc_diff_thr;
}

SPARKFastLIO2::PropagationCheckpoint SPARKFastLIO2::propagateLidarFrame(
    const MeasureGroup &measures)
{
    PropagationCheckpoint checkpoint;
    checkpoint.gravity_aligned          = is_gravity_aligned_;
    checkpoint.gravity_rotation         = gravity_alignment_rotation_;
    checkpoint.gravity_directions       = global_gravity_directions_;
    checkpoint.static_acceleration_mean = stationary_mean_acceleration_;
    checkpoint.moving_frame_count       = num_consecutive_moving_frames_;

    imu_processor_->process(measures, kf_, full_points_);

    // A rejected LiDAR correction must retain this frame's IMU propagation.
    // Rolling back to the last corrected state would discard elapsed motion for
    // every rejected frame.
    checkpoint.propagated_filter       = kf_;
    checkpoint.propagated_imu_snapshot = imu_processor_->getSnapshot();
    return checkpoint;
}

void SPARKFastLIO2::restorePropagatedFrame(const PropagationCheckpoint &checkpoint)
{
    is_gravity_aligned_            = checkpoint.gravity_aligned;
    gravity_alignment_rotation_             = checkpoint.gravity_rotation;
    global_gravity_directions_     = checkpoint.gravity_directions;
    stationary_mean_acceleration_              = checkpoint.static_acceleration_mean;
    num_consecutive_moving_frames_ = checkpoint.moving_frame_count;

    kf_ = checkpoint.propagated_filter;
    imu_processor_->restoreSnapshot(checkpoint.propagated_imu_snapshot);
    latest_state_     = kf_.get_x();
    latest_state_.pos = gravity_alignment_rotation_ * latest_state_.pos;
    latest_state_.rot = gravity_alignment_rotation_ * latest_state_.rot;
    kf_for_preintegration_ = kf_;
}

void SPARKFastLIO2::publishPropagatedFrame()
{
    const auto stamp = rclcpp::Time(lidar_end_time_ * 1e9);
    publishCurrentFrame(latest_state_, stamp, false, false);
}

PointCloudXYZI::ConstPtr SPARKFastLIO2::selectMatchingPoints()
{
    PointCloudXYZI::ConstPtr matching_points = full_points_;
    if (point_filter_num_ > 1)
    {
        sampled_points_->reserve(full_points_->size() / point_filter_num_);
        for (std::size_t i = 0; i < full_points_->points.size(); i += point_filter_num_)
        {
            sampled_points_->push_back(full_points_->points[i]);
        }
        matching_points = sampled_points_;
    }
    return matching_points;
}

void SPARKFastLIO2::updateGravityAlignmentBeforeLio(MeasureGroup &measures)
{
    if (!enable_gravity_alignment_ || is_gravity_aligned_ || base_frame_.empty())
    {
        return;
    }

    if (!filter_initialized_)
    {
        // Assume that it is stationary at the beginning.
        stationary_mean_acceleration_ = measures.getMeanAcc();
        return;
    }

    const auto &mean_acc = measures.getMeanAcc();
    if (isMotionStopped(stationary_mean_acceleration_, mean_acc, acceleration_difference_threshold_))
    {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Waiting for motion to perform gravity alignment...now a robot "
                           "has been stopped");
        num_consecutive_moving_frames_ = 0;
        return;
    }

    num_consecutive_moving_frames_ = min(num_consecutive_moving_frames_ + 1, 100000);
}

bool SPARKFastLIO2::initializeLocalMapIfNeeded()
{
    if (ikd_tree_.Root_Node != nullptr)
    {
        return true;
    }

    if (downsampled_point_count_ > 5)
    {
        ikd_tree_.set_downsample_param(filter_size_map_min_);
        feats_down_world_->resize(downsampled_point_count_);
        for (int i = 0; i < downsampled_point_count_; ++i)
        {
            pointBodyToWorld(
                &(feats_down_body_->points[i]), &(feats_down_world_->points[i]), latest_state_);
        }
#ifdef SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER
        std::sort(feats_down_world_->points.begin(),
                  feats_down_world_->points.end(),
                  pointLessXYZICurvature);
#endif
        ikd_tree_.Build(feats_down_world_->points);
    }
    return false;
}

bool SPARKFastLIO2::prepareLioUpdate(MeasureGroup &measures,
                                     const PointCloudXYZI::ConstPtr &matching_points,
                                     state_ikfom &propagated_state)
{
    latest_state_   = kf_.get_x();
    filter_initialized_ = (measures.lidar_beg_time - first_lidar_time_) >= kInitializationTimeSec;

    if (matching_points->empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "No point, skip this scan! full_points=%zu "
                             "matching_points=%zu raw_lidar=%zu imu=%zu scan_dt=%.3f ms "
                             "pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]",
                             full_points_->size(),
                             matching_points->size(),
                             measures.lidar ? measures.lidar->size() : 0,
                             measures.imu.size(),
                             (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0,
                             latest_state_.pos[0],
                             latest_state_.pos[1],
                             latest_state_.pos[2],
                             latest_state_.vel[0],
                             latest_state_.vel[1],
                             latest_state_.vel[2]);
        if (motion_quality_gate_enabled_ && have_last_good_state_)
        {
            publishPropagatedFrame();
        }
        return false;
    }

    latest_state_    = kf_.get_x();
    propagated_state = latest_state_;
    updateGravityAlignmentBeforeLio(measures);

    down_size_filter_.setInputCloud(matching_points);
    down_size_filter_.filter(*feats_down_body_);
#ifdef SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER
    std::sort(feats_down_body_->points.begin(),
              feats_down_body_->points.end(),
              pointLessXYZICurvature);
#endif
    downsampled_point_count_ = feats_down_body_->points.size();

    if (!initializeLocalMapIfNeeded())
    {
        return false;
    }
    if (downsampled_point_count_ < 5)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "No point, skip this scan! full_points=%zu "
                             "sampled_points=%zu feats_down=%d raw_lidar=%zu imu=%zu "
                             "scan_dt=%.3f ms pos=[%.3f, %.3f, %.3f] "
                             "vel=[%.3f, %.3f, %.3f]",
                             full_points_->size(),
                             sampled_points_->size(),
                             downsampled_point_count_,
                             measures.lidar ? measures.lidar->size() : 0,
                             measures.imu.size(),
                             (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0,
                             latest_state_.pos[0],
                             latest_state_.pos[1],
                             latest_state_.pos[2],
                             latest_state_.vel[0],
                             latest_state_.vel[1],
                             latest_state_.vel[2]);
        if (motion_quality_gate_enabled_ && have_last_good_state_)
        {
            publishPropagatedFrame();
        }
        return false;
    }

    point_normals_->resize(downsampled_point_count_);
    feats_down_world_->resize(downsampled_point_count_);
    nearest_map_points_.resize(downsampled_point_count_);
    point_residuals_.assign(downsampled_point_count_, 0.0f);
    surface_point_selected_.assign(downsampled_point_count_, 0U);
    return true;
}

void SPARKFastLIO2::updateGravityAlignmentAfterLio()
{
    if (!enable_gravity_alignment_ || is_gravity_aligned_ || base_frame_.empty() ||
        num_consecutive_moving_frames_ <= moving_frame_threshold_)
    {
        return;
    }

    const Eigen::Matrix3d offset_R_I_B =
        lidar_rotation_in_base_ * latest_state_.offset_R_L_I.inverse();
    const V3D gravity_direction = kf_.get_x().grav;
    if (global_gravity_directions_.size() <
        static_cast<std::size_t>(gravity_measurement_threshold_))
    {
        std::stringstream ss;
        ss << "Waiting for motion: " << global_gravity_directions_.size() << " / "
           << gravity_measurement_threshold_;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        global_gravity_directions_.push_back(offset_R_I_B * gravity_direction);
        return;
    }

    V3D average_global_gravity = Eigen::Vector3d::Zero();
    for (const auto &gravity : global_gravity_directions_)
    {
        average_global_gravity += gravity;
    }
    average_global_gravity /= global_gravity_directions_.size();
    gravity_alignment_rotation_ = computeRelativeRotation(average_global_gravity, base_gravity_);

    std::stringstream ss;
    ss << "Gravity alignment complete! `R_gravity_aligned`: " << gravity_alignment_rotation_;
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    is_gravity_aligned_ = true;
}

void SPARKFastLIO2::runLioUpdate()
{
    double update_start = omp_get_wtime();
    double solve_H_time = 0;
    kf_.update_iterated_dyn_share_modified(kLaserPointCovariance, solve_H_time);
    updateGravityAlignmentAfterLio();
    (void) update_start;
}

SPARKFastLIO2::MotionQualityReport SPARKFastLIO2::evaluateMotionQuality(
    MeasureGroup &measures,
    const state_ikfom &propagated_state)
{
    latest_state_     = kf_.get_x();
    latest_state_.pos = gravity_alignment_rotation_ * latest_state_.pos;
    latest_state_.rot = gravity_alignment_rotation_ * latest_state_.rot;

    MotionQualityReport quality;
    quality.lidar_time        = lidar_end_time_;
    quality.mean_acceleration = measures.getMeanAcc();
    const SO3 rotation_update = propagated_state.rot.conjugate() * latest_state_.rot;
    quality.rotation_correction_deg =
        Eigen::AngleAxisd(rotation_update.toRotationMatrix()).angle() * 180.0 / M_PI;
    quality.pre_gravity_residual =
        propagated_state.rot * quality.mean_acceleration +
        V3D(propagated_state.grav[0], propagated_state.grav[1], propagated_state.grav[2]);
    quality.post_gravity_residual =
        latest_state_.rot * quality.mean_acceleration +
        V3D(latest_state_.grav[0], latest_state_.grav[1], latest_state_.grav[2]);
    quality.correction_step = (latest_state_.pos - propagated_state.pos).norm();
    quality.velocity_norm   = latest_state_.vel.norm();

    // Frame-count gating instead of clock-based throttling: clock-driven
    // throttling formats this line on a timing-dependent subset of frames,
    // which perturbs the allocator and breaks bit-reproducible replay.
    if (++lio_state_log_counter_ % 10 == 1)
    {
        RCLCPP_INFO(this->get_logger(),
                    "LIO state: pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f] "
                    "ekf_update_step=%.3f m rot_update=%.3f deg res_mean=%.5f "
                    "bg=[%.5f, %.5f, %.5f] ba=[%.5f, %.5f, %.5f] "
                    "grav=[%.3f, %.3f, %.3f] mean_acc=[%.3f, %.3f, %.3f] "
                    "mean_acc_norm=%.3f pre_grav_res=%.3f post_grav_res=%.3f "
                    "feats_down=%d effect=%d tree=%d imu=%zu "
                    "scan_dt=%.3f ms",
                    latest_state_.pos[0],
                    latest_state_.pos[1],
                    latest_state_.pos[2],
                    latest_state_.vel[0],
                    latest_state_.vel[1],
                    latest_state_.vel[2],
                    quality.correction_step,
                    quality.rotation_correction_deg,
                    mean_residual_,
                    latest_state_.bg[0],
                    latest_state_.bg[1],
                    latest_state_.bg[2],
                    latest_state_.ba[0],
                    latest_state_.ba[1],
                    latest_state_.ba[2],
                    latest_state_.grav[0],
                    latest_state_.grav[1],
                    latest_state_.grav[2],
                    quality.mean_acceleration[0],
                    quality.mean_acceleration[1],
                    quality.mean_acceleration[2],
                    quality.mean_acceleration.norm(),
                    quality.pre_gravity_residual.norm(),
                    quality.post_gravity_residual.norm(),
                    downsampled_point_count_,
                    effective_feature_count_,
                    ikd_tree_.size(),
                    measures.imu.size(),
                    (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0);
    }

    if (have_last_lio_debug_state_)
    {
        quality.delta_time = quality.lidar_time - last_lio_debug_time_;
        quality.state_step  = (latest_state_.pos - last_lio_debug_pos_).norm();
        quality.state_speed =
            quality.delta_time > 1.0e-6 ? quality.state_step / quality.delta_time
                : 0.0;
        quality.correction_step_ratio =
            quality.state_step > 1.0e-6 ? quality.correction_step / quality.state_step : 1.0;
    }
    quality.effective_feature_ratio =
        downsampled_point_count_ > 0
            ? static_cast<double>(effective_feature_count_) / static_cast<double>(downsampled_point_count_)
            : 0.0;
    quality.finite_state =
        std::isfinite(latest_state_.pos[0]) && std::isfinite(latest_state_.pos[1]) &&
        std::isfinite(latest_state_.pos[2]) && std::isfinite(latest_state_.vel[0]) &&
        std::isfinite(latest_state_.vel[1]) && std::isfinite(latest_state_.vel[2]);
    quality.high_pre_gravity_residual =
        quality.pre_gravity_residual.norm() > motion_gate_max_pre_grav_residual_;
    quality.high_post_gravity_residual =
        quality.post_gravity_residual.norm() > motion_gate_max_pre_grav_residual_;
    quality.suspicious_large_correction =
        have_last_lio_debug_state_ && quality.delta_time > 0.0 &&
        quality.state_step > motion_gate_suspect_frame_step_ &&
        quality.correction_step > motion_gate_max_update_step_;
    quality.weak_lidar_update =
        quality.correction_step_ratio < motion_gate_max_update_step_ratio_;
    quality.weak_lidar_constraints =
        effective_feature_count_ < motion_gate_min_effective_features_ ||
        quality.effective_feature_ratio < motion_gate_min_effective_ratio_;
    quality.unsupported_recovery_step =
        have_last_lio_debug_state_ && quality.delta_time > 0.0 &&
        motion_gate_consecutive_reject_count_ > 0 &&
        quality.state_step > motion_gate_suspect_frame_step_ &&
        quality.correction_step <= motion_gate_max_update_step_ &&
        (quality.weak_lidar_update ||
         (motion_gate_reject_weak_lidar_ && quality.weak_lidar_constraints) ||
         quality.high_pre_gravity_residual ||
         quality.high_post_gravity_residual);
    quality.reject =
        motion_quality_gate_enabled_ && filter_initialized_ &&
        (!quality.finite_state ||
         (motion_gate_reject_weak_lidar_ && quality.weak_lidar_constraints) ||
         quality.high_pre_gravity_residual || quality.high_post_gravity_residual ||
         quality.suspicious_large_correction || quality.unsupported_recovery_step);
    return quality;
}

void SPARKFastLIO2::rejectMotionFrame(const MeasureGroup &measures,
                                      const PropagationCheckpoint &checkpoint,
                                      const MotionQualityReport &quality)
{
    ++motion_gate_reject_count_;
    ++motion_gate_consecutive_reject_count_;
    restorePropagatedFrame(checkpoint);
    RCLCPP_WARN(this->get_logger(),
                "Motion quality gate rejected scan #%d: consecutive=%d dt=%.3f s step=%.3f m "
                "speed=%.3f m/s ekf_update_step=%.3f m update_step_ratio=%.3f "
                "state_speed=%.3f m/s pre_grav_res=%.3f post_grav_res=%.3f "
                "feats_down=%d effect=%d effective_ratio=%.3f imu=%zu "
                "scan_dt=%.3f ms finite_state=%d weak_lidar=%d high_pre_grav=%d "
                "high_post_grav=%d weak_update=%d large_correction=%d "
                "unsupported_recovery=%d restored_propagation=1 published_propagation=1",
                motion_gate_reject_count_,
                motion_gate_consecutive_reject_count_,
                quality.delta_time,
                quality.state_step,
                quality.state_speed,
                quality.correction_step,
                quality.correction_step_ratio,
                quality.velocity_norm,
                quality.pre_gravity_residual.norm(),
                quality.post_gravity_residual.norm(),
                downsampled_point_count_,
                effective_feature_count_,
                quality.effective_feature_ratio,
                measures.imu.size(),
                (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0,
                quality.finite_state ? 1 : 0,
                quality.weak_lidar_constraints ? 1 : 0,
                quality.high_pre_gravity_residual ? 1 : 0,
                quality.high_post_gravity_residual ? 1 : 0,
                quality.weak_lidar_update ? 1 : 0,
                quality.suspicious_large_correction ? 1 : 0,
                quality.unsupported_recovery_step ? 1 : 0);
    publishPropagatedFrame();
}

void SPARKFastLIO2::logLargeStateJump(const MeasureGroup &measures,
                                      const state_ikfom &propagated_state,
                                      const MotionQualityReport &quality)
{
    if (!have_last_lio_debug_state_ ||
        ((quality.delta_time <= 0.0 || quality.state_step <= 0.5) &&
         latest_state_.pos.norm() <= 50.0))
    {
        return;
    }

    const double imu_first_time =
        measures.imu.empty() ? 0.0 : rclcpp::Time(measures.imu.front()->header.stamp).seconds();
    const double imu_last_time =
        measures.imu.empty() ? 0.0 : rclcpp::Time(measures.imu.back()->header.stamp).seconds();
    const double imu_span_ms =
        measures.imu.empty() ? 0.0 : (imu_last_time - imu_first_time) * 1000.0;
    const double imu_first_minus_lidar_begin_ms =
        measures.imu.empty() ? 0.0 : (imu_first_time - measures.lidar_beg_time) * 1000.0;
    const double lidar_end_minus_imu_last_ms =
        measures.imu.empty() ? 0.0 : (measures.lidar_end_time - imu_last_time) * 1000.0;
    RCLCPP_WARN_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Large LIO state jump: dt=%.3f s step=%.3f m speed=%.3f m/s "
                         "pre_pos=[%.3f, %.3f, %.3f] pre_vel=[%.3f, %.3f, %.3f] "
                         "post_pos=[%.3f, %.3f, %.3f] post_vel=[%.3f, %.3f, %.3f] "
                         "ekf_update_step=%.3f m rot_update=%.3f deg res_mean=%.5f "
                         "bg=[%.5f, %.5f, %.5f] ba=[%.5f, %.5f, %.5f] "
                         "grav=[%.3f, %.3f, %.3f] "
                         "pre_grav_res=%.3f post_grav_res=%.3f "
                         "feats_down=%d effect=%d tree=%d imu=%zu "
                         "scan_dt=%.3f ms imu_span=%.3f ms "
                         "imu_first_minus_lidar_begin=%.3f ms "
                         "lidar_end_minus_imu_last=%.3f ms",
                         quality.delta_time,
                         quality.state_step,
                         quality.state_speed,
                         propagated_state.pos[0],
                         propagated_state.pos[1],
                         propagated_state.pos[2],
                         propagated_state.vel[0],
                         propagated_state.vel[1],
                         propagated_state.vel[2],
                         latest_state_.pos[0],
                         latest_state_.pos[1],
                         latest_state_.pos[2],
                         latest_state_.vel[0],
                         latest_state_.vel[1],
                         latest_state_.vel[2],
                         quality.correction_step,
                         quality.rotation_correction_deg,
                         mean_residual_,
                         latest_state_.bg[0],
                         latest_state_.bg[1],
                         latest_state_.bg[2],
                         latest_state_.ba[0],
                         latest_state_.ba[1],
                         latest_state_.ba[2],
                         latest_state_.grav[0],
                         latest_state_.grav[1],
                         latest_state_.grav[2],
                         quality.pre_gravity_residual.norm(),
                         quality.post_gravity_residual.norm(),
                         downsampled_point_count_,
                         effective_feature_count_,
                         ikd_tree_.size(),
                         measures.imu.size(),
                         (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0,
                         imu_span_ms,
                         imu_first_minus_lidar_begin_ms,
                         lidar_end_minus_imu_last_ms);
}

void SPARKFastLIO2::commitOdometryUpdate(const MeasureGroup &measures,
                                         const state_ikfom &propagated_state,
                                         const MotionQualityReport &quality)
{
    updateLocalMapWindow();
    motion_gate_consecutive_reject_count_ = 0;
    kf_for_preintegration_                = kf_;
    last_good_kf_                         = kf_;
    last_good_imu_processor_snapshot_     = imu_processor_->getSnapshot();
    last_good_state_                      = latest_state_;
    have_last_good_state_                 = true;
    logLargeStateJump(measures, propagated_state, quality);

    have_last_lio_debug_state_ = true;
    last_lio_debug_pos_        = latest_state_.pos;
    last_lio_debug_time_       = quality.lidar_time;

    if (enable_gravity_alignment_ && !is_gravity_aligned_ && !base_frame_.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Gravity alignment is enabled but not yet completed. Waiting for alignment...");
        return;
    }

    const auto stamp = rclcpp::Time(lidar_end_time_ * 1e9);
    publishCurrentFrame(latest_state_, stamp, true, true);
}

void SPARKFastLIO2::processLidarAndImu(MeasureGroup &measures)
{
    if (is_first_lidar_scan_)
    {
        first_lidar_time_    = measures.lidar_beg_time;
        is_first_lidar_scan_ = false;
        return;
    }

    // Keep the IMU-processed full cloud for dense publishing/PCD saving, then sample
    // it here so scan matching uses the same corrected points at a lower rate.
    full_points_->clear();
    sampled_points_->clear();

    const auto propagation_checkpoint = propagateLidarFrame(measures);
    const auto matching_points        = selectMatchingPoints();

    state_ikfom propagated_state;
    if (!prepareLioUpdate(measures, matching_points, propagated_state))
    {
        return;
    }

    runLioUpdate();
    const auto motion_quality = evaluateMotionQuality(measures, propagated_state);
    if (motion_quality.reject)
    {
        rejectMotionFrame(measures, propagation_checkpoint, motion_quality);
        return;
    }

    commitOdometryUpdate(measures, propagated_state, motion_quality);
}
}  // namespace spark_fast_lio

RCLCPP_COMPONENTS_REGISTER_NODE(spark_fast_lio::SPARKFastLIO2)
