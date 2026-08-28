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
bool hasFiniteImuMeasurements(const MeasureGroup &measures)
{
    return std::all_of(measures.imu.begin(), measures.imu.end(), [](const auto &measurement)
                       { return measurement && ImuProcessor::hasFiniteMeasurement(*measurement); });
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

bool SPARKFastLIO2::isMotionStopped(const V3D &reference_acceleration,
                                    const V3D &current_acceleration,
                                    const double max_acceleration_difference)
{
    return (reference_acceleration - current_acceleration).norm() <=
           max_acceleration_difference;
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

    if (!is_lio_warmup_complete_)
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
    is_lio_warmup_complete_ =
        (measures.lidar_beg_time - first_lidar_time_) >= kInitializationTimeSec;

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
        if (quality_gate_enabled_ && has_accepted_lio_update_)
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
        if (quality_gate_enabled_ && has_accepted_lio_update_)
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

void SPARKFastLIO2::updateFilterWithLidar()
{
    double solver_time = 0;
    kf_.update_iterated_dyn_share_modified(kLaserPointCovariance, solver_time);
    updateGravityAlignmentAfterLio();
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
    quality.predicted_linear_acceleration =
        propagated_state.rot * quality.mean_acceleration +
        V3D(propagated_state.grav[0], propagated_state.grav[1], propagated_state.grav[2]);
    quality.corrected_linear_acceleration =
        latest_state_.rot * quality.mean_acceleration +
        V3D(latest_state_.grav[0], latest_state_.grav[1], latest_state_.grav[2]);
    quality.lidar_position_adjustment = (latest_state_.pos - propagated_state.pos).norm();
    quality.velocity_norm   = latest_state_.vel.norm();

    // Frame-count gating instead of clock-based throttling: clock-driven
    // throttling formats this line on a timing-dependent subset of frames,
    // which perturbs the allocator and breaks bit-reproducible replay.
    if (++lio_state_log_counter_ % 10 == 1)
    {
        RCLCPP_INFO(this->get_logger(),
                    "LIO state: pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f] "
                    "lidar_position_adjustment=%.3f m rot_update=%.3f deg res_mean=%.5f "
                    "bg=[%.5f, %.5f, %.5f] ba=[%.5f, %.5f, %.5f] "
                    "grav=[%.3f, %.3f, %.3f] mean_acc=[%.3f, %.3f, %.3f] "
                    "mean_acc_norm=%.3f predicted_linear_acc=%.3f corrected_linear_acc=%.3f "
                    "feats_down=%d effect=%d tree=%d imu=%zu "
                    "scan_dt=%.3f ms",
                    latest_state_.pos[0],
                    latest_state_.pos[1],
                    latest_state_.pos[2],
                    latest_state_.vel[0],
                    latest_state_.vel[1],
                    latest_state_.vel[2],
                    quality.lidar_position_adjustment,
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
                    quality.predicted_linear_acceleration.norm(),
                    quality.corrected_linear_acceleration.norm(),
                    downsampled_point_count_,
                    effective_feature_count_,
                    ikd_tree_.size(),
                    measures.imu.size(),
                    (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0);
    }

    if (has_accepted_lio_update_)
    {
        quality.delta_time = quality.lidar_time - last_accepted_lio_time_;
        quality.position_step = (latest_state_.pos - last_accepted_lio_position_).norm();
        quality.position_speed =
            quality.delta_time > 1.0e-6 ? quality.position_step / quality.delta_time
                : 0.0;
        quality.lidar_adjustment_ratio =
            quality.position_step > 1.0e-6
                ? quality.lidar_position_adjustment / quality.position_step
                : 1.0;
    }
    quality.matched_feature_ratio =
        downsampled_point_count_ > 0
            ? static_cast<double>(effective_feature_count_) / static_cast<double>(downsampled_point_count_)
            : 0.0;
    quality.has_finite_state = hasFiniteState(latest_state_);
    quality.has_high_predicted_linear_acceleration =
        quality.predicted_linear_acceleration.norm() > max_linear_acceleration_;
    quality.has_high_corrected_linear_acceleration =
        quality.corrected_linear_acceleration.norm() > max_linear_acceleration_;
    quality.has_large_frame_jump_and_lidar_adjustment =
        has_accepted_lio_update_ && quality.delta_time > 0.0 &&
        quality.position_step > max_jump_between_two_frames_ &&
        quality.lidar_position_adjustment > max_lidar_position_adjustment_;
    quality.has_small_lidar_adjustment =
        quality.lidar_adjustment_ratio < min_recovery_lidar_adjustment_ratio_;
    quality.has_insufficient_matches =
        effective_feature_count_ < min_matched_features_ ||
        quality.matched_feature_ratio < min_matched_feature_ratio_;
    quality.has_unsupported_recovery_step =
        has_accepted_lio_update_ && quality.delta_time > 0.0 &&
        consecutive_gate_reject_count_ > 0 &&
        quality.position_step > max_jump_between_two_frames_ &&
        quality.lidar_position_adjustment <= max_lidar_position_adjustment_ &&
        (quality.has_small_lidar_adjustment ||
         (reject_weak_lidar_ && quality.has_insufficient_matches) ||
         quality.has_high_predicted_linear_acceleration ||
         quality.has_high_corrected_linear_acceleration);
    quality.should_reject =
        quality_gate_enabled_ && is_lio_warmup_complete_ &&
        (!quality.has_finite_state ||
         (reject_weak_lidar_ && quality.has_insufficient_matches) ||
         quality.has_high_predicted_linear_acceleration ||
         quality.has_high_corrected_linear_acceleration ||
         quality.has_large_frame_jump_and_lidar_adjustment ||
         quality.has_unsupported_recovery_step);
    return quality;
}

void SPARKFastLIO2::rejectMotionFrame(const MeasureGroup &measures,
                                      const PropagationCheckpoint &checkpoint,
                                      const MotionQualityReport &quality)
{
    if (!quality.has_finite_state)
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

    ++gate_reject_count_;
    ++consecutive_gate_reject_count_;
    restorePropagatedFrame(checkpoint);
    RCLCPP_WARN(this->get_logger(),
                "Motion quality gate rejected scan #%d: consecutive=%d dt=%.3f s step=%.3f m "
                "speed=%.3f m/s lidar_position_adjustment=%.3f m lidar_adjustment_ratio=%.3f "
                "position_speed=%.3f m/s predicted_linear_acc=%.3f corrected_linear_acc=%.3f "
                "feats_down=%d matched=%d matched_ratio=%.3f imu=%zu "
                "scan_dt=%.3f ms finite_state=%d insufficient_matches=%d "
                "high_predicted_linear_acc=%d high_corrected_linear_acc=%d "
                "small_lidar_adjustment=%d large_frame_jump_and_lidar_adjustment=%d "
                "unsupported_recovery=%d restored_propagation=1 published_propagation=1",
                gate_reject_count_,
                consecutive_gate_reject_count_,
                quality.delta_time,
                quality.position_step,
                quality.position_speed,
                quality.lidar_position_adjustment,
                quality.lidar_adjustment_ratio,
                quality.velocity_norm,
                quality.predicted_linear_acceleration.norm(),
                quality.corrected_linear_acceleration.norm(),
                downsampled_point_count_,
                effective_feature_count_,
                quality.matched_feature_ratio,
                measures.imu.size(),
                (measures.lidar_end_time - measures.lidar_beg_time) * 1000.0,
                quality.has_finite_state ? 1 : 0,
                quality.has_insufficient_matches ? 1 : 0,
                quality.has_high_predicted_linear_acceleration ? 1 : 0,
                quality.has_high_corrected_linear_acceleration ? 1 : 0,
                quality.has_small_lidar_adjustment ? 1 : 0,
                quality.has_large_frame_jump_and_lidar_adjustment ? 1 : 0,
                quality.has_unsupported_recovery_step ? 1 : 0);
    publishPropagatedFrame();
}

void SPARKFastLIO2::logLargeStateJump(const MeasureGroup &measures,
                                      const state_ikfom &propagated_state,
                                      const MotionQualityReport &quality)
{
    if (!has_accepted_lio_update_ || quality.delta_time <= 0.0 ||
        quality.position_step <= 0.5)
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
                         "lidar_position_adjustment=%.3f m rot_update=%.3f deg res_mean=%.5f "
                         "bg=[%.5f, %.5f, %.5f] ba=[%.5f, %.5f, %.5f] "
                         "grav=[%.3f, %.3f, %.3f] "
                         "predicted_linear_acc=%.3f corrected_linear_acc=%.3f "
                         "feats_down=%d effect=%d tree=%d imu=%zu "
                         "scan_dt=%.3f ms imu_span=%.3f ms "
                         "imu_first_minus_lidar_begin=%.3f ms "
                         "lidar_end_minus_imu_last=%.3f ms",
                         quality.delta_time,
                         quality.position_step,
                         quality.position_speed,
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
                         quality.lidar_position_adjustment,
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
                         quality.predicted_linear_acceleration.norm(),
                         quality.corrected_linear_acceleration.norm(),
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
    consecutive_gate_reject_count_ = 0;
    kf_for_preintegration_                = kf_;
    has_accepted_lio_update_              = true;
    logLargeStateJump(measures, propagated_state, quality);

    last_accepted_lio_position_ = latest_state_.pos;
    last_accepted_lio_time_     = quality.lidar_time;

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

    if (!has_lidar_start_time_)
    {
        first_lidar_time_      = measures.lidar_beg_time;
        has_lidar_start_time_  = true;
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

    updateFilterWithLidar();
    const auto motion_quality = evaluateMotionQuality(measures, propagated_state);
    if (!motion_quality.has_finite_state || motion_quality.should_reject)
    {
        rejectMotionFrame(measures, propagation_checkpoint, motion_quality);
        return;
    }

    commitOdometryUpdate(measures, propagated_state, motion_quality);
}
}  // namespace spark_fast_lio
