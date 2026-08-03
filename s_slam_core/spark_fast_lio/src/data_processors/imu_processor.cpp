#include "data_processors/imu_processor.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace
{

bool isPointTimeOrdered(const PointType &lhs, const PointType &rhs)
{
    return lhs.curvature < rhs.curvature;
}

}  // namespace

ImuProcessor::ImuProcessor()
    : start_timestamp_(-1),
      last_lidar_end_time_(-1),
      init_begin_time_(-1),
      init_end_time_(-1),
      is_first_frame_(true),
      needs_initialization_(true)
{
    init_sample_count_               = 1;
    process_noise_covariance_        = process_noise_cov();
    accelerometer_covariance_        = V3D(0.1, 0.1, 0.1);
    gyroscope_covariance_            = V3D(0.1, 0.1, 0.1);
    accelerometer_covariance_scale_  = accelerometer_covariance_;
    gyroscope_covariance_scale_      = gyroscope_covariance_;
    gyroscope_bias_covariance_       = V3D(0.0001, 0.0001, 0.0001);
    accelerometer_bias_covariance_   = V3D(0.0001, 0.0001, 0.0001);
    mean_acceleration_               = V3D(0, 0, -1.0);
    mean_angular_velocity_           = V3D(0, 0, 0);
    last_angular_velocity_           = Zero3d;
    last_acceleration_world_         = Zero3d;
    lidar_translation_wrt_imu_       = Zero3d;
    lidar_rotation_wrt_imu_          = Eye3d;
    last_imu_.reset(new sensor_msgs::msg::Imu());
}

void ImuProcessor::setWarmStartPrior(
    const V3D &gravity_body,
    const V3D &bg,
    const V3D &ba,
    const V3D &vel_body)
{
    warm_start_gravity_body_ = gravity_body;
    warm_start_bg_           = bg;
    warm_start_ba_           = ba;
    warm_start_vel_body_     = vel_body;
    has_warm_start_prior_    = true;
}

ImuProcessor::Snapshot ImuProcessor::getSnapshot() const
{
    Snapshot snapshot;
    snapshot.process_noise_covariance       = process_noise_covariance_;
    snapshot.accelerometer_covariance       = accelerometer_covariance_;
    snapshot.gyroscope_covariance           = gyroscope_covariance_;
    snapshot.accelerometer_covariance_scale = accelerometer_covariance_scale_;
    snapshot.gyroscope_covariance_scale     = gyroscope_covariance_scale_;
    snapshot.gyroscope_bias_covariance      = gyroscope_bias_covariance_;
    snapshot.accelerometer_bias_covariance  = accelerometer_bias_covariance_;
    snapshot.last_imu                       = last_imu_;
    snapshot.imu_poses                      = imu_poses_;
    snapshot.lidar_rotation_wrt_imu         = lidar_rotation_wrt_imu_;
    snapshot.lidar_translation_wrt_imu      = lidar_translation_wrt_imu_;
    snapshot.mean_acceleration              = mean_acceleration_;
    snapshot.mean_angular_velocity          = mean_angular_velocity_;
    snapshot.last_angular_velocity          = last_angular_velocity_;
    snapshot.last_acceleration_world        = last_acceleration_world_;
    snapshot.start_timestamp                = start_timestamp_;
    snapshot.last_lidar_end_time            = last_lidar_end_time_;
    snapshot.init_begin_time                = init_begin_time_;
    snapshot.init_end_time                  = init_end_time_;
    snapshot.init_sample_count              = init_sample_count_;
    snapshot.is_first_frame                 = is_first_frame_;
    snapshot.needs_initialization           = needs_initialization_;
    return snapshot;
}

void ImuProcessor::restoreSnapshot(const Snapshot &snapshot)
{
    process_noise_covariance_       = snapshot.process_noise_covariance;
    accelerometer_covariance_       = snapshot.accelerometer_covariance;
    gyroscope_covariance_           = snapshot.gyroscope_covariance;
    accelerometer_covariance_scale_ = snapshot.accelerometer_covariance_scale;
    gyroscope_covariance_scale_     = snapshot.gyroscope_covariance_scale;
    gyroscope_bias_covariance_      = snapshot.gyroscope_bias_covariance;
    accelerometer_bias_covariance_  = snapshot.accelerometer_bias_covariance;
    last_imu_                       = snapshot.last_imu;
    imu_poses_                      = snapshot.imu_poses;
    lidar_rotation_wrt_imu_         = snapshot.lidar_rotation_wrt_imu;
    lidar_translation_wrt_imu_      = snapshot.lidar_translation_wrt_imu;
    mean_acceleration_              = snapshot.mean_acceleration;
    mean_angular_velocity_          = snapshot.mean_angular_velocity;
    last_angular_velocity_          = snapshot.last_angular_velocity;
    last_acceleration_world_        = snapshot.last_acceleration_world;
    start_timestamp_                = snapshot.start_timestamp;
    last_lidar_end_time_            = snapshot.last_lidar_end_time;
    init_begin_time_                = snapshot.init_begin_time;
    init_end_time_                  = snapshot.init_end_time;
    init_sample_count_              = snapshot.init_sample_count;
    is_first_frame_                 = snapshot.is_first_frame;
    needs_initialization_           = snapshot.needs_initialization;
}

void ImuProcessor::reset()
{
    mean_acceleration_        = V3D(0, 0, -1.0);
    mean_angular_velocity_    = V3D(0, 0, 0);
    last_angular_velocity_    = Zero3d;
    last_acceleration_world_  = Zero3d;
    accelerometer_covariance_ = V3D(0.1, 0.1, 0.1);
    gyroscope_covariance_     = V3D(0.1, 0.1, 0.1);
    needs_initialization_     = true;
    is_first_frame_           = true;
    start_timestamp_          = -1;
    last_lidar_end_time_      = -1;
    init_begin_time_          = -1;
    init_end_time_            = -1;
    init_sample_count_        = 1;
    imu_poses_.clear();
    last_imu_.reset(new sensor_msgs::msg::Imu());
}

void ImuProcessor::setExtrinsic(const MD(4, 4) &transform)
{
    lidar_translation_wrt_imu_ = transform.block<3, 1>(0, 3);
    lidar_rotation_wrt_imu_    = transform.block<3, 3>(0, 0);
}

void ImuProcessor::setExtrinsic(const V3D &translation)
{
    lidar_translation_wrt_imu_ = translation;
    lidar_rotation_wrt_imu_.setIdentity();
}

void ImuProcessor::setExtrinsic(const V3D &translation, const M3D &rotation)
{
    lidar_translation_wrt_imu_ = translation;
    lidar_rotation_wrt_imu_    = rotation;
}

void ImuProcessor::setGyroscopeCovariance(const V3D &covariance)
{
    gyroscope_covariance_scale_ = covariance;
}

void ImuProcessor::setAccelerometerCovariance(const V3D &covariance)
{
    accelerometer_covariance_scale_ = covariance;
}

void ImuProcessor::setGyroscopeBiasCovariance(const V3D &covariance)
{
    gyroscope_bias_covariance_ = covariance;
}

void ImuProcessor::setAccelerometerBiasCovariance(const V3D &covariance)
{
    accelerometer_bias_covariance_ = covariance;
}

void ImuProcessor::setReplayMode(const bool replay_mode)
{
    replay_mode_ = replay_mode;
}

void ImuProcessor::initializeImu(
    const MeasureGroup &measures,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &filter,
    int &init_sample_count)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/

    V3D cur_acc, cur_gyr;

    if (is_first_frame_)
    {
        reset();
        init_sample_count   = 1;
        is_first_frame_     = false;
        const auto &imu_acc = measures.imu.front()->linear_acceleration;
        const auto &gyr_acc = measures.imu.front()->angular_velocity;
        const auto first_imu_time = rclcpp::Time(measures.imu.front()->header.stamp).seconds();
        mean_acceleration_ << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_angular_velocity_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        init_begin_time_ = first_imu_time;
        init_end_time_   = first_imu_time;
    }

    for (const auto &imu : measures.imu)
    {
        init_end_time_ = rclcpp::Time(imu->header.stamp).seconds();

        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acceleration_ += (cur_acc - mean_acceleration_) / init_sample_count;
        mean_angular_velocity_ += (cur_gyr - mean_angular_velocity_) / init_sample_count;

        accelerometer_covariance_ =
            accelerometer_covariance_ * (init_sample_count - 1.0) / init_sample_count +
            (cur_acc - mean_acceleration_).cwiseProduct(cur_acc - mean_acceleration_) *
                (init_sample_count - 1.0) / (init_sample_count * init_sample_count);
        gyroscope_covariance_ =
            gyroscope_covariance_ * (init_sample_count - 1.0) / init_sample_count +
            (cur_gyr - mean_angular_velocity_).cwiseProduct(cur_gyr - mean_angular_velocity_) *
                (init_sample_count - 1.0) / (init_sample_count * init_sample_count);

        ++init_sample_count;
    }
    state_ikfom init_state = filter.get_x();
    init_state.grav         = S2(-mean_acceleration_ / mean_acceleration_.norm() * G_m_s2);

    init_state.bg           = mean_angular_velocity_;
    init_state.offset_T_L_I = lidar_translation_wrt_imu_;
    init_state.offset_R_L_I = lidar_rotation_wrt_imu_;
    filter.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = filter.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    filter.change_P(init_P);
    last_imu_ = measures.imu.back();
}

state_ikfom ImuProcessor::integrateImu(
    const std::deque<sensor_msgs::msg::Imu> &imu_queue,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &filter)
{
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    input_ikfom in;
    const auto &head = imu_queue[0];
    const auto &tail = imu_queue[1];

    angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
        0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
        0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
    acc_avr << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
        0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
        0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

    acc_avr = acc_avr * G_m_s2 / mean_acceleration_.norm();  // - state_inout.ba;

    double dt =
        rclcpp::Time(tail.header.stamp).seconds() - rclcpp::Time(head.header.stamp).seconds();

    in.acc                         = acc_avr;
    in.gyro                        = angvel_avr;
    process_noise_covariance_.block<3, 3>(0, 0).diagonal() = gyroscope_covariance_;
    process_noise_covariance_.block<3, 3>(3, 3).diagonal() = accelerometer_covariance_;
    process_noise_covariance_.block<3, 3>(6, 6).diagonal() = gyroscope_bias_covariance_;
    process_noise_covariance_.block<3, 3>(9, 9).diagonal() = accelerometer_bias_covariance_;
    filter.predict(dt, process_noise_covariance_, in);

    return filter.get_x();
}

void ImuProcessor::undistortPointCloud(
    const MeasureGroup &measures,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &filter,
    PointCloudXYZI &point_cloud)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = measures.imu;
    v_imu.push_front(last_imu_);
    const double imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
    const double pcl_beg_time = measures.lidar_beg_time;
    const double pcl_end_time = measures.lidar_end_time;

    /*** sort point clouds by offset time ***/
    point_cloud = *(measures.lidar);
    std::sort(point_cloud.points.begin(), point_cloud.points.end(), isPointTimeOrdered);
    /*** Initialize IMU pose ***/
    state_ikfom imu_state = filter.get_x();
    imu_poses_.clear();
    imu_poses_.push_back(set_pose6d(0.0,
                                    last_acceleration_world_,
                                    last_angular_velocity_,
                                    imu_state.vel,
                                    imu_state.pos,
                                    imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    double dt = 0;

    input_ikfom in;
    bool propagated = false;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); ++it_imu)
    {
        const auto &head = *it_imu;
        const auto &tail = *(it_imu + 1);

        if (rclcpp::Time(tail->header.stamp).seconds() < last_lidar_end_time_)
        {
            continue;
        }

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        acc_avr = acc_avr * G_m_s2 / mean_acceleration_.norm();  // - state_inout.ba;

        if (rclcpp::Time(head->header.stamp).seconds() < last_lidar_end_time_)
        {
            dt = rclcpp::Time(tail->header.stamp).seconds() - last_lidar_end_time_;
        }
        else
        {
            dt = rclcpp::Time(tail->header.stamp).seconds() -
                 rclcpp::Time(head->header.stamp).seconds();
        }

        in.acc                         = acc_avr;
        in.gyro                        = angvel_avr;
        process_noise_covariance_.block<3, 3>(0, 0).diagonal() = gyroscope_covariance_;
        process_noise_covariance_.block<3, 3>(3, 3).diagonal() = accelerometer_covariance_;
        process_noise_covariance_.block<3, 3>(6, 6).diagonal() = gyroscope_bias_covariance_;
        process_noise_covariance_.block<3, 3>(9, 9).diagonal() = accelerometer_bias_covariance_;
        filter.predict(dt, process_noise_covariance_, in);
        propagated = true;

        /* save the poses at each IMU measurements */
        imu_state                = filter.get_x();
        last_angular_velocity_   = angvel_avr - imu_state.bg;
        last_acceleration_world_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; ++i)
        {
            last_acceleration_world_[i] += imu_state.grav[i];
        }
        const double offs_t = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
        imu_poses_.push_back(set_pose6d(offs_t,
                                        last_acceleration_world_,
                                        last_angular_velocity_,
                                        imu_state.vel,
                                        imu_state.pos,
                                        imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    if (propagated)
    {
        double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
        dt          = note * (pcl_end_time - imu_end_time);
        filter.predict(dt, process_noise_covariance_, in);
    }

    imu_state            = filter.get_x();
    last_imu_            = measures.imu.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (point_cloud.points.begin() == point_cloud.points.end())
    {
        return;
    }
    auto it_pcl = point_cloud.points.end() - 1;
    for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); --it_kp)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        acc_imu << VEC_FROM_ARRAY(tail->acc);
        angvel_avr << VEC_FROM_ARRAY(tail->gyr);

        for (; it_pcl->curvature / static_cast<double>(1000) > head->offset_time; --it_pcl)
        {
            dt = it_pcl->curvature / static_cast<double>(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global
             * frame
             */
            M3D R_i(R_imu * Exp(angvel_avr, dt));

            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            V3D P_compensate =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() *
                     (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                 imu_state.offset_T_L_I);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == point_cloud.points.begin())
            {
                break;
            }
        }
    }
}

void ImuProcessor::process(
    const MeasureGroup &measures,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &filter,
    PointCloudXYZI::Ptr undistorted_cloud)
{
    if (measures.imu.empty())
    {
        return;
    }
    assert(measures.lidar != nullptr);

    if (needs_initialization_)
    {
        /// The very first lidar frame
        state_ikfom state_before_init = filter.get_x();
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov cov_before_init = filter.get_P();
        initializeImu(measures, filter, init_sample_count_);

        needs_initialization_ = true;

        last_imu_ = measures.imu.back();

        if (has_warm_start_prior_)
        {
            // Warm re-initialization after a mid-run reset. The platform may be
            // moving (in flight), where the stationary window below never
            // comes. Anchor the new world frame at the current body attitude
            // (rot = identity) and seed gravity/biases/velocity from the
            // pre-reset state; the gravity covariance is left large so LiDAR
            // updates can absorb the attitude drift accumulated since the
            // prior was captured.
            state_ikfom init_state = filter.get_x();
            init_state.grav         = S2(warm_start_gravity_body_);
            init_state.bg           = warm_start_bg_;
            init_state.ba           = warm_start_ba_;
            init_state.vel          = warm_start_vel_body_;
            init_state.offset_T_L_I = lidar_translation_wrt_imu_;
            init_state.offset_R_L_I = lidar_rotation_wrt_imu_;
            filter.change_x(init_state);

            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = filter.get_P();
            init_P.setIdentity();
            init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
            init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
            init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
            init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
            // Wider than the cold-init 1e-5: the prior gravity direction is
            // stale by the reset-to-reinit gap.
            init_P(21, 21) = init_P(22, 22) = 0.1;
            filter.change_P(init_P);

            needs_initialization_         = false;
            last_lidar_end_time_          = measures.lidar_end_time;
            accelerometer_covariance_     = accelerometer_covariance_scale_;
            gyroscope_covariance_         = gyroscope_covariance_scale_;
            has_warm_start_prior_         = false;
            RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"),
                        "IMU warm re-initialization from pre-reset state (no stationary "
                        "window required): grav_body=[%.4f, %.4f, %.4f] "
                        "bg=[%.5f, %.5f, %.5f] vel_body=[%.3f, %.3f, %.3f]",
                        warm_start_gravity_body_[0],
                        warm_start_gravity_body_[1],
                        warm_start_gravity_body_[2],
                        warm_start_bg_[0],
                        warm_start_bg_[1],
                        warm_start_bg_[2],
                        warm_start_vel_body_[0],
                        warm_start_vel_body_[1],
                        warm_start_vel_body_[2]);
            return;
        }

        state_ikfom imu_state = filter.get_x();
        const int init_samples     = std::max(0, init_sample_count_ - 1);
        const double init_duration = init_end_time_ - init_begin_time_;
        const double mean_acc_norm = mean_acceleration_.norm();
        const double mean_gyr_norm = mean_angular_velocity_.norm();
        // A moving or vibrating window can still have a mean acceleration norm
        // near gravity. Reject it so linear acceleration is not baked into the
        // initial gravity direction.
        const double acc_std_norm =
            accelerometer_covariance_.cwiseMax(V3D::Zero()).cwiseSqrt().norm();
        const double gyr_std_norm =
            gyroscope_covariance_.cwiseMax(V3D::Zero()).cwiseSqrt().norm();
        const bool has_enough_imu = init_samples >= kMinImuInitSamples &&
                                    init_duration >= kMinImuInitDuration;
        const bool is_stationary  = mean_acc_norm >= kMinInitAccNorm &&
                                   mean_acc_norm <= kMaxInitAccNorm &&
                                   mean_gyr_norm <= kMaxInitMeanGyroNorm &&
                                   acc_std_norm <= kMaxInitAccStdNorm &&
                                   gyr_std_norm <= kMaxInitGyrStdNorm;
        if (has_enough_imu && !replay_mode_ && !is_stationary)
        {
            RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"),
                        "IMU initialization rejected: keep the sensor still "
                        "(samples=%d duration=%.3f s mean_acc_norm=%.6f mean_gyr_norm=%.6f "
                        "acc_std_norm=%.6f gyr_std_norm=%.6f)",
                        init_samples,
                        init_duration,
                        mean_acc_norm,
                        mean_gyr_norm,
                        acc_std_norm,
                        gyr_std_norm);
            reset();
            filter.change_x(state_before_init);
            filter.change_P(cov_before_init);
            return;
        }
        if (has_enough_imu && replay_mode_ && !is_stationary)
        {
            RCLCPP_WARN(rclcpp::get_logger("ImuProcessor"),
                        "IMU initialization accepted without a stationary window "
                        "(dataset/replay mode): samples=%d duration=%.3f s "
                        "mean_acc_norm=%.6f mean_gyr_norm=%.6f acc_std_norm=%.6f "
                        "gyr_std_norm=%.6f",
                        init_samples,
                        init_duration,
                        mean_acc_norm,
                        mean_gyr_norm,
                        acc_std_norm,
                        gyr_std_norm);
        }

        if (!has_enough_imu)
        {
            // Do not commit provisional gravity/bias estimates until a full
            // stationary window has passed the initialization checks.
            filter.change_x(state_before_init);
            filter.change_P(cov_before_init);
            return;
        }

        if (has_enough_imu)
        {
            accelerometer_covariance_ *= std::pow(G_m_s2 / mean_acceleration_.norm(), 2);
            needs_initialization_     = false;
            last_lidar_end_time_      = measures.lidar_end_time;

            accelerometer_covariance_ = accelerometer_covariance_scale_;
            gyroscope_covariance_     = gyroscope_covariance_scale_;
            RCLCPP_INFO(rclcpp::get_logger("ImuProcessor"),
                        "IMU Initial Done: samples=%d duration=%.3f s "
                        "mean_acc=[%.6f, %.6f, %.6f] norm=%.6f "
                        "mean_gyr=[%.6f, %.6f, %.6f] "
                        "grav=[%.6f, %.6f, %.6f] bg=[%.6f, %.6f, %.6f] "
                        "cov_acc=[%.6g, %.6g, %.6g] cov_gyr=[%.6g, %.6g, %.6g]",
                        init_samples,
                        init_duration,
                        mean_acceleration_[0],
                        mean_acceleration_[1],
                        mean_acceleration_[2],
                        mean_acceleration_.norm(),
                        mean_angular_velocity_[0],
                        mean_angular_velocity_[1],
                        mean_angular_velocity_[2],
                        imu_state.grav[0],
                        imu_state.grav[1],
                        imu_state.grav[2],
                        imu_state.bg[0],
                        imu_state.bg[1],
                        imu_state.bg[2],
                        accelerometer_covariance_[0],
                        accelerometer_covariance_[1],
                        accelerometer_covariance_[2],
                        gyroscope_covariance_[0],
                        gyroscope_covariance_[1],
                        gyroscope_covariance_[2]);
        }

        return;
    }

    undistortPointCloud(measures, filter, *undistorted_cloud);
}
