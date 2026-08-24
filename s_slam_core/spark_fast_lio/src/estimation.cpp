#include "spark_fast_lio.h"

#include "common/gravity_alignment.hpp"
#include "point_ordering.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <omp.h>

namespace spark_fast_lio
{
namespace
{
bool hasFiniteImuMeasurement(const sensor_msgs::msg::Imu &measurement)
{
    const auto &acceleration     = measurement.linear_acceleration;
    const auto &angular_velocity = measurement.angular_velocity;
    return std::isfinite(acceleration.x) && std::isfinite(acceleration.y) &&
           std::isfinite(acceleration.z) && std::isfinite(angular_velocity.x) &&
           std::isfinite(angular_velocity.y) && std::isfinite(angular_velocity.z);
}

bool hasFiniteImuMeasurements(const MeasureGroup &measures)
{
    return std::all_of(measures.imu.begin(), measures.imu.end(), [](const auto &measurement)
                       { return measurement && hasFiniteImuMeasurement(*measurement); });
}

bool hasFiniteState(const state_ikfom &state)
{
    const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
    return state.pos.allFinite() && state.vel.allFinite() && state.bg.allFinite() &&
           state.ba.allFinite() && gravity.allFinite() &&
           state.rot.toRotationMatrix().allFinite() &&
           state.offset_R_L_I.toRotationMatrix().allFinite() && state.offset_T_L_I.allFinite();
}
}  // namespace

struct SPARKFastLIO2::PropagationCheckpoint
{
    bool gravity_aligned = false;
    M3D gravity_rotation = Eye3d;
    std::deque<V3D> gravity_directions;
    V3D static_acceleration_mean = Zero3d;
    int moving_frame_count = 0;
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter_before_propagation;
    ImuProcessor::Snapshot imu_snapshot_before_propagation;
    bool state_before_propagation_is_finite = false;
    esekfom::esekf<state_ikfom, 12, input_ikfom> propagated_filter;
    ImuProcessor::Snapshot propagated_imu_snapshot;
    bool propagated_state_is_finite = false;
};

bool SPARKFastLIO2::isMotionStopped(const V3D &acc_ref,
                                    const V3D &acc_curr,
                                    const double acc_diff_thr)
{
    return (acc_ref - acc_curr).norm() <= acc_diff_thr;
}

SPARKFastLIO2::PropagationCheckpoint SPARKFastLIO2::propagateLidarFrame(
    MeasureGroup &measures)
{
    PropagationCheckpoint checkpoint;
    checkpoint.gravity_aligned          = is_gravity_aligned_;
    checkpoint.gravity_rotation         = gravity_alignment_rotation_;
    checkpoint.gravity_directions       = global_gravity_directions_;
    checkpoint.static_acceleration_mean = stationary_mean_acceleration_;
    checkpoint.moving_frame_count       = num_consecutive_moving_frames_;
    checkpoint.filter_before_propagation          = kf_;
    checkpoint.imu_snapshot_before_propagation    = imu_processor_->getSnapshot();
    checkpoint.state_before_propagation_is_finite = hasFiniteState(kf_.get_x());

    imu_processor_->process(measures, kf_, full_points_);

    // A rejected LiDAR correction must retain this frame's IMU propagation.
    // Rolling back to the last corrected state would discard elapsed motion for
    // every rejected frame.
    checkpoint.propagated_filter          = kf_;
    checkpoint.propagated_imu_snapshot    = imu_processor_->getSnapshot();
    checkpoint.propagated_state_is_finite = hasFiniteState(kf_.get_x());
    return checkpoint;
}

void SPARKFastLIO2::restorePropagatedFrame(const PropagationCheckpoint &checkpoint)
{
    is_gravity_aligned_            = checkpoint.gravity_aligned;
    gravity_alignment_rotation_    = checkpoint.gravity_rotation;
    global_gravity_directions_     = checkpoint.gravity_directions;
    stationary_mean_acceleration_  = checkpoint.static_acceleration_mean;
    num_consecutive_moving_frames_ = checkpoint.moving_frame_count;

    kf_ = checkpoint.propagated_filter;
    imu_processor_->restoreSnapshot(checkpoint.propagated_imu_snapshot);
    latest_state_ = kf_.get_x();
    if (!is_gravity_aligned_)
    {
        latest_state_.pos = gravity_alignment_rotation_ * latest_state_.pos;
        latest_state_.rot = gravity_alignment_rotation_ * latest_state_.rot;
    }
    kf_for_preintegration_ = kf_;
}

void SPARKFastLIO2::restorePrePropagationFrame(const PropagationCheckpoint &checkpoint)
{
    is_gravity_aligned_            = checkpoint.gravity_aligned;
    gravity_alignment_rotation_    = checkpoint.gravity_rotation;
    global_gravity_directions_     = checkpoint.gravity_directions;
    stationary_mean_acceleration_  = checkpoint.static_acceleration_mean;
    num_consecutive_moving_frames_ = checkpoint.moving_frame_count;

    kf_ = checkpoint.filter_before_propagation;
    imu_processor_->restoreSnapshot(checkpoint.imu_snapshot_before_propagation);
    latest_state_ = kf_.get_x();
    if (!is_gravity_aligned_)
    {
        latest_state_.pos = gravity_alignment_rotation_ * latest_state_.pos;
        latest_state_.rot = gravity_alignment_rotation_ * latest_state_.rot;
    }
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
                  point_ordering::lessXYZICurvature);
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
                             measures.lidar_point_count,
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
              point_ordering::lessXYZICurvature);
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
                             measures.lidar_point_count,
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
    latest_state_ = kf_.get_x();
    if (!is_gravity_aligned_)
    {
        // Preserve the existing pre-alignment trajectory bit pattern. The
        // rotation is identity here; after alignment, latest_state_ remains raw.
        latest_state_.pos = gravity_alignment_rotation_ * latest_state_.pos;
        latest_state_.rot = gravity_alignment_rotation_ * latest_state_.rot;
    }

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
    quality.finite_state = hasFiniteState(latest_state_);
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
    if (!quality.finite_state)
    {
        if (checkpoint.propagated_state_is_finite)
        {
            restorePropagatedFrame(checkpoint);
            RCLCPP_ERROR(this->get_logger(),
                         "Dropping non-finite LiDAR update; publishing valid IMU propagation.");
            publishPropagatedFrame();
            return;
        }

        if (checkpoint.state_before_propagation_is_finite)
        {
            restorePrePropagationFrame(checkpoint);
            RCLCPP_ERROR(this->get_logger(),
                         "Dropping LiDAR frame because IMU propagation produced a non-finite state; "
                         "restored the state before propagation.");
            return;
        }

        RCLCPP_ERROR(this->get_logger(),
                     "IMU propagation started from a non-finite state; resetting the estimator.");
        resetEstimatorState("non-finite IMU propagation state", ResetMode::kCold);
        return;
    }

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
        // Do not return before adding this raw scan: the local window may have evicted
        // old points, and skipping an accepted scan would leave the matching map stale.
        insertScanIntoMap(latest_state_);
        RCLCPP_WARN(this->get_logger(),
                    "Gravity alignment is enabled but not yet completed. Waiting for alignment...");
        return;
    }

    const auto stamp = rclcpp::Time(lidar_end_time_ * 1e9);
    publishCurrentFrame(latest_state_, stamp, true, true);
}

void SPARKFastLIO2::processLidarAndImu(MeasureGroup &measures)
{
    if (!hasFiniteImuMeasurements(measures))
    {
        RCLCPP_ERROR(this->get_logger(), "Dropping LiDAR frame with non-finite IMU data.");
        return;
    }

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
    if (!propagation_checkpoint.propagated_state_is_finite)
    {
        rejectMotionFrame(measures, propagation_checkpoint, MotionQualityReport{});
        return;
    }

    const auto matching_points = selectMatchingPoints();

    state_ikfom propagated_state;
    if (!prepareLioUpdate(measures, matching_points, propagated_state))
    {
        return;
    }

    runLioUpdate();
    const auto motion_quality = evaluateMotionQuality(measures, propagated_state);
    if (!motion_quality.finite_state || motion_quality.reject)
    {
        rejectMotionFrame(measures, propagation_checkpoint, motion_quality);
        return;
    }

    commitOdometryUpdate(measures, propagated_state, motion_quality);
}
}  // namespace spark_fast_lio
