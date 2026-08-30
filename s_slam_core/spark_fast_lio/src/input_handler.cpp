#include "spark_fast_lio.h"

#include "common/gravity_alignment.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace spark_fast_lio
{
void SPARKFastLIO2::standardLiDARCallback(const sensor_msgs::msg::PointCloud2 &msg)
{
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

    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);
        if (has_last_lidar_timestamp_ && msg_time < last_lidar_timestamp_)
        {
            resetEstimatorState("LiDAR timestamp moved backwards", ResetMode::kWarmRecovery);
        }
        last_lidar_timestamp_ = msg_time;
        has_last_lidar_timestamp_ = true;

        lidar_buffer_.push_back({ptr, msg_time.seconds(), msg_end_time});
    }

    if (process_on_callback_)
    {
        processPendingMeasurements();
    }
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void SPARKFastLIO2::livoxLiDARCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
    const rclcpp::Time msg_time = msg->header.stamp;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    if (!lidar_processor_->process(*msg, ptr))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);
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

        if (time_sync_enabled_ && !has_lidar_imu_time_offset_ && diff_s > 1.0 && !imu_buffer_.empty())
        {
            has_lidar_imu_time_offset_         = true;
            lidar_imu_time_offset_ =
                last_lidar_timestamp_.nanoseconds() + static_cast<int64_t>(1.0e8) -
                last_imu_timestamp_.nanoseconds();
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Self sync IMU and LiDAR, time diff is " << lidar_imu_time_offset_ << "[ns]");
        }

        lidar_buffer_.push_back({ptr, msg_time.seconds(), 0.0});
    }

    if (process_on_callback_)
    {
        processPendingMeasurements();
    }
}
#endif

void SPARKFastLIO2::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    if (!ImuProcessor::hasFiniteMeasurement(*msg))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "Dropping non-finite IMU measurement.");
        return;
    }

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

        if (imu_predicted_odometry_enabled_ && process_on_callback_)
        {
            // In callback-driven replay, defer prediction until the matching
            // LiDAR interval is complete so DDS callback order cannot choose
            // whether this IMU sees the preceding LiDAR correction.
            imu_prediction_buffer_.push_back(*imu_input);
        }
        else if (imu_predicted_odometry_enabled_ && kf_for_preintegration_.has_value())
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

    const double head_time =
        rclcpp::Time(imu_integration_queue_[0].header.stamp).seconds();
    const double tail_time =
        rclcpp::Time(imu_integration_queue_[1].header.stamp).seconds();
    const double integration_start_time =
        imu_prediction_state_time_.has_value() ? *imu_prediction_state_time_ : head_time;
    const double delta_time = tail_time - integration_start_time;

    if (delta_time <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "IMU timestamps must be in ascending order!");
        imu_integration_queue_.pop_front();
        return;
    }
    if (delta_time > max_imu_gap_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "Skipping predicted odometry across an IMU gap: "
                             "gap=%.6f max=%.6f",
                             delta_time,
                             max_imu_gap_);
        imu_integration_queue_.pop_front();
        return;
    }

    const auto &head = imu_integration_queue_[0];
    const auto &tail = imu_integration_queue_[1];
    const V3D measured_angular_velocity(
        0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
        0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
        0.5 * (head.angular_velocity.z + tail.angular_velocity.z));
    const state_ikfom integrated_state = imu_processor_->integrateImu(
        imu_integration_queue_, delta_time, state);
    const StateCovariance state_covariance = state.get_P();
    const V3D angular_velocity = measured_angular_velocity - integrated_state.bg;
    const auto &stamp = tail.header.stamp;
    imu_integration_queue_.pop_front();
    if (imu_prediction_state_time_.has_value())
    {
        imu_prediction_state_time_ = tail_time;
    }
    const M3D world_rotation =
        is_gravity_aligned_ ? gravity_alignment_rotation_ : M3D::Identity();
    const PoseCovariance pose_covariance =
        poseCovariance(integrated_state, state_covariance, world_rotation, viz_frame_);

    if (is_gravity_aligned_)
    {
        publishOdometry(
            gravityAlignedState(integrated_state, gravity_alignment_rotation_),
            pose_covariance,
            stamp,
            pub_imu_predicted_odom_,
            false);
    }
    else
    {
        publishOdometry(integrated_state, pose_covariance, stamp, pub_imu_predicted_odom_, false);
    }
    publishPx4Odometry(
        integrated_state, state_covariance, angular_velocity, world_rotation, stamp);
}

void SPARKFastLIO2::publishImuPredictionsUpTo(const double end_time)
{
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    while (!imu_prediction_buffer_.empty())
    {
        const double imu_time =
            rclcpp::Time(imu_prediction_buffer_.front().header.stamp).seconds();
        if (imu_time > end_time)
        {
            break;
        }
        sensor_msgs::msg::Imu imu = std::move(imu_prediction_buffer_.front());
        imu_prediction_buffer_.pop_front();

        if (!kf_for_preintegration_.has_value() ||
            (imu_prediction_state_time_.has_value() &&
             imu_time <= *imu_prediction_state_time_))
        {
            continue;
        }
        integrateIMU(*kf_for_preintegration_, imu);
    }
}

void SPARKFastLIO2::resetImuPrediction(const sensor_msgs::msg::Imu &reference_imu,
                                       const double state_time)
{
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    imu_integration_queue_.clear();
    imu_integration_queue_.push_back(reference_imu);
    imu_prediction_state_time_ = state_time;

    while (!imu_prediction_buffer_.empty() &&
           rclcpp::Time(imu_prediction_buffer_.front().header.stamp).seconds() <= state_time)
    {
        imu_prediction_buffer_.pop_front();
    }
}

void SPARKFastLIO2::main()
{
    processPendingMeasurements();
}

void SPARKFastLIO2::processPendingMeasurements()
{
    while (syncPackages(measures_, verbose_))
    {
        if (imu_predicted_odometry_enabled_ && process_on_callback_)
        {
            // Publish this interval from the previous LiDAR-corrected state
            // before the current LiDAR frame changes that state.
            publishImuPredictionsUpTo(measures_.lidar_end_time);
        }
        processLidarAndImu(measures_);
    }
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

    if ((!has_pending_lidar_frame_ && lidar_buffer_.empty()) || imu_buffer_.empty())
    {
        return false;
    }

    if (!has_pending_lidar_frame_)
    {
        const BufferedLidarFrame &buffered_lidar = lidar_buffer_.front();
        measurements.lidar                       = buffered_lidar.cloud;
        measurements.lidar_beg_time              = buffered_lidar.begin_time;
        measurements.lidar_point_count           = buffered_lidar.cloud->size();
        const double msg_end_time                 = buffered_lidar.end_time;
        lidar_buffer_.pop_front();
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
        has_pending_lidar_frame_   = true;
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

    /*** push imu data, and pop from imu buffer ***/
    double next_imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
    measurements.imu.clear();
    while (!imu_buffer_.empty())
    {
        const double imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
        if (imu_time > lidar_end_time_)
        {
            next_imu_time = imu_time;
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
                             next_imu_time);
        has_pending_lidar_frame_ = false;
        return false;
    }

    double largest_imu_gap = 0.0;
    std::optional<double> previous_imu_time = last_consumed_imu_time_;
    for (const auto &imu : measurements.imu)
    {
        const double imu_time = rclcpp::Time(imu->header.stamp).seconds();
        if (previous_imu_time.has_value())
        {
            largest_imu_gap = std::max(largest_imu_gap, imu_time - *previous_imu_time);
        }
        previous_imu_time = imu_time;
    }
    largest_imu_gap = std::max(largest_imu_gap, lidar_end_time_ - *previous_imu_time);
    last_consumed_imu_time_ = previous_imu_time;

    if (largest_imu_gap > max_imu_gap_)
    {
        ++imu_gap_lidar_skip_count_;
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "Skipping LiDAR scan due to IMU coverage gap: count=%d "
                             "lidar=[%.6f, %.6f] largest_imu_gap=%.6f max_imu_gap=%.6f",
                             imu_gap_lidar_skip_count_,
                             measurements.lidar_beg_time,
                             lidar_end_time_,
                             largest_imu_gap,
                             max_imu_gap_);
        imu_processor_->skipLidarFrame(measurements);
        has_pending_lidar_frame_ = false;
        return false;
    }

    has_pending_lidar_frame_ = false;

    return true;
}
}  // namespace spark_fast_lio
