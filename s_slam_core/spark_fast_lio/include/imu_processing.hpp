#include <math.h>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/common_lib.h"
#include "common/so3_math.h"
#include "common/use-ikfom.hpp"

/// *************Preconfiguration

static constexpr int kMinImuInitSamples      = 200;
static constexpr double kMinImuInitDuration  = 2.0;
static constexpr double kMaxInitMeanGyroNorm = 0.05;
// Initialization estimates gravity from average acceleration, so the window
// must be quiet, not just close to 1g on average.
static constexpr double kMaxInitAccStdNorm   = 0.5;
static constexpr double kMaxInitGyrStdNorm   = 0.03;
static constexpr double kMinInitAccNorm      = 8.0;
static constexpr double kMaxInitAccNorm      = 11.5;

bool time_list(PointType &x, PointType &y)
{
    return (x.curvature < y.curvature);
}

/// *************IMU Process and undistortion
class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Snapshot
    {
        Eigen::Matrix<double, 12, 12> Q;
        V3D cov_acc;
        V3D cov_gyr;
        V3D cov_acc_scale;
        V3D cov_gyr_scale;
        V3D cov_bias_gyr;
        V3D cov_bias_acc;
        double first_lidar_time = 0.0;
        std::shared_ptr<const sensor_msgs::msg::Imu> last_imu;
        std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> v_imu;
        std::vector<Pose6D> imu_pose;
        std::vector<M3D> v_rot_pcl;
        M3D lidar_R_wrt_imu;
        V3D lidar_T_wrt_imu;
        V3D mean_acc;
        V3D mean_gyr;
        V3D angvel_last;
        V3D acc_s_last;
        double start_timestamp = -1.0;
        double last_lidar_end_time = -1.0;
        double init_begin_time = -1.0;
        double init_end_time = -1.0;
        int init_iter_num = 1;
        bool b_first_frame = true;
        bool imu_need_init = true;
    };

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void Reset(double start_timestamp, const std::shared_ptr<const sensor_msgs::msg::Imu> &lastimu);
    Snapshot GetSnapshot() const;
    void RestoreSnapshot(const Snapshot &snapshot);
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4, 4) & T);
    void set_gyr_cov(const V3D &scaler);
    void set_acc_cov(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    Eigen::Matrix<double, 12, 12> Q;
    void Process(const MeasureGroup &meas,
                 esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                 PointCloudXYZI::Ptr pcl_un_);
    state_ikfom IntegrateIMU(const std::deque<sensor_msgs::msg::Imu> imu_queue,
                             esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
    std::ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;

private:
    void IMU_init(const MeasureGroup &meas,
                  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                  int &init_sample_count);
    void UndistortPcl(const MeasureGroup &meas,
                      esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                      PointCloudXYZI &pcl_in_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    std::shared_ptr<const sensor_msgs::msg::Imu> last_imu_;
    std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> v_imu_;
    std::vector<Pose6D> IMUpose;
    std::vector<M3D> v_rot_pcl_;
    M3D Lidar_R_wrt_IMU;
    V3D Lidar_T_wrt_IMU;
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double start_timestamp_;
    double last_lidar_end_time_;
    double init_begin_time_;
    double init_end_time_;
    int init_iter_num   = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : start_timestamp_(-1),
      last_lidar_end_time_(-1),
      init_begin_time_(-1),
      init_end_time_(-1),
      b_first_frame_(true),
      imu_need_init_(true)
{
    init_iter_num   = 1;
    Q               = process_noise_cov();
    cov_acc         = V3D(0.1, 0.1, 0.1);
    cov_gyr         = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr    = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc    = V3D(0.0001, 0.0001, 0.0001);
    mean_acc        = V3D(0, 0, -1.0);
    mean_gyr        = V3D(0, 0, 0);
    angvel_last     = Zero3d;
    Lidar_T_wrt_IMU = Zero3d;
    Lidar_R_wrt_IMU = Eye3d;
    last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess()
{
}

ImuProcess::Snapshot ImuProcess::GetSnapshot() const
{
    Snapshot snapshot;
    snapshot.Q                   = Q;
    snapshot.cov_acc             = cov_acc;
    snapshot.cov_gyr             = cov_gyr;
    snapshot.cov_acc_scale       = cov_acc_scale;
    snapshot.cov_gyr_scale       = cov_gyr_scale;
    snapshot.cov_bias_gyr        = cov_bias_gyr;
    snapshot.cov_bias_acc        = cov_bias_acc;
    snapshot.first_lidar_time    = first_lidar_time;
    snapshot.last_imu            = last_imu_;
    snapshot.v_imu               = v_imu_;
    snapshot.imu_pose            = IMUpose;
    snapshot.v_rot_pcl           = v_rot_pcl_;
    snapshot.lidar_R_wrt_imu     = Lidar_R_wrt_IMU;
    snapshot.lidar_T_wrt_imu     = Lidar_T_wrt_IMU;
    snapshot.mean_acc            = mean_acc;
    snapshot.mean_gyr            = mean_gyr;
    snapshot.angvel_last         = angvel_last;
    snapshot.acc_s_last          = acc_s_last;
    snapshot.start_timestamp     = start_timestamp_;
    snapshot.last_lidar_end_time = last_lidar_end_time_;
    snapshot.init_begin_time     = init_begin_time_;
    snapshot.init_end_time       = init_end_time_;
    snapshot.init_iter_num       = init_iter_num;
    snapshot.b_first_frame       = b_first_frame_;
    snapshot.imu_need_init       = imu_need_init_;
    return snapshot;
}

void ImuProcess::RestoreSnapshot(const Snapshot &snapshot)
{
    Q                    = snapshot.Q;
    cov_acc              = snapshot.cov_acc;
    cov_gyr              = snapshot.cov_gyr;
    cov_acc_scale        = snapshot.cov_acc_scale;
    cov_gyr_scale        = snapshot.cov_gyr_scale;
    cov_bias_gyr         = snapshot.cov_bias_gyr;
    cov_bias_acc         = snapshot.cov_bias_acc;
    first_lidar_time     = snapshot.first_lidar_time;
    last_imu_            = snapshot.last_imu;
    v_imu_               = snapshot.v_imu;
    IMUpose              = snapshot.imu_pose;
    v_rot_pcl_           = snapshot.v_rot_pcl;
    Lidar_R_wrt_IMU      = snapshot.lidar_R_wrt_imu;
    Lidar_T_wrt_IMU      = snapshot.lidar_T_wrt_imu;
    mean_acc             = snapshot.mean_acc;
    mean_gyr             = snapshot.mean_gyr;
    angvel_last          = snapshot.angvel_last;
    acc_s_last           = snapshot.acc_s_last;
    start_timestamp_     = snapshot.start_timestamp;
    last_lidar_end_time_ = snapshot.last_lidar_end_time;
    init_begin_time_     = snapshot.init_begin_time;
    init_end_time_       = snapshot.init_end_time;
    init_iter_num        = snapshot.init_iter_num;
    b_first_frame_       = snapshot.b_first_frame;
    imu_need_init_       = snapshot.imu_need_init;
}

void ImuProcess::Reset()
{
    // ROS_WARN("Reset ImuProcess");
    mean_acc         = V3D(0, 0, -1.0);
    mean_gyr         = V3D(0, 0, 0);
    angvel_last      = Zero3d;
    imu_need_init_   = true;
    b_first_frame_   = true;
    start_timestamp_ = -1;
    last_lidar_end_time_ = -1;
    init_begin_time_     = -1;
    init_end_time_       = -1;
    init_iter_num    = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new sensor_msgs::msg::Imu());
    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
    Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
    Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
    cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
    cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
    cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
    cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas,
                          esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                          int &init_sample_count)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/

    V3D cur_acc, cur_gyr;

    if (b_first_frame_)
    {
        Reset();
        init_sample_count   = 1;
        b_first_frame_      = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        const auto first_imu_time = rclcpp::Time(meas.imu.front()->header.stamp).seconds();
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        first_lidar_time = meas.lidar_beg_time;
        init_begin_time_ = first_imu_time;
        init_end_time_   = first_imu_time;
    }

    for (const auto &imu : meas.imu)
    {
        init_end_time_ = rclcpp::Time(imu->header.stamp).seconds();

        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += (cur_acc - mean_acc) / init_sample_count;
        mean_gyr += (cur_gyr - mean_gyr) / init_sample_count;

        cov_acc = cov_acc * (init_sample_count - 1.0) / init_sample_count +
                  (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) *
                      (init_sample_count - 1.0) / (init_sample_count * init_sample_count);
        cov_gyr = cov_gyr * (init_sample_count - 1.0) / init_sample_count +
                  (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) *
                      (init_sample_count - 1.0) / (init_sample_count * init_sample_count);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        ++init_sample_count;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav        = S2(-mean_acc / mean_acc.norm() * G_m_s2);

    // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    init_state.bg           = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
    last_imu_ = meas.imu.back();
}

state_ikfom ImuProcess::IntegrateIMU(const std::deque<sensor_msgs::msg::Imu> imu_queue,
                                     esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
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

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm();  // - state_inout.ba;

    double dt =
        rclcpp::Time(tail.header.stamp).seconds() - rclcpp::Time(head.header.stamp).seconds();

    in.acc                         = acc_avr;
    in.gyro                        = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    return kf_state.get_x();
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas,
                              esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudXYZI &pcl_out)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
    const double &pcl_beg_time = meas.lidar_beg_time;
    const double &pcl_end_time = meas.lidar_end_time;

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", "
    //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0,
                                 acc_s_last,
                                 angvel_last,
                                 imu_state.vel,
                                 imu_state.pos,
                                 imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    double dt = 0;

    input_ikfom in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); ++it_imu)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

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

        // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " <<
        // angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

        acc_avr = acc_avr * G_m_s2 / mean_acc.norm();  // - state_inout.ba;

        if (rclcpp::Time(head->header.stamp).seconds() < last_lidar_end_time_)
        {
            dt = rclcpp::Time(tail->header.stamp).seconds() - last_lidar_end_time_;
            // dt = tail->header.stamp.toSec() - pcl_beg_time;
        }
        else
        {
            dt = rclcpp::Time(tail->header.stamp).seconds() -
                 rclcpp::Time(head->header.stamp).seconds();
        }

        in.acc                         = acc_avr;
        in.gyro                        = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
        kf_state.predict(dt, Q, in);

        /* save the poses at each IMU measurements */
        imu_state   = kf_state.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; ++i)
        {
            acc_s_last[i] += imu_state.grav[i];
        }
        double &&offs_t = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t,
                                     acc_s_last,
                                     angvel_last,
                                     imu_state.vel,
                                     imu_state.pos,
                                     imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt          = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q, in);

    imu_state            = kf_state.get_x();
    last_imu_            = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.begin() == pcl_out.points.end())
    {
        return;
    }
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); --it_kp)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
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

            if (it_pcl == pcl_out.points.begin())
            {
                break;
            }
        }
    }
}

void ImuProcess::Process(const MeasureGroup &meas,
                         esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudXYZI::Ptr cur_pcl_un_)
{
    if (meas.imu.empty())
    {
        return;
    }
    assert(meas.lidar != nullptr);

    if (imu_need_init_)
    {
        /// The very first lidar frame
        IMU_init(meas, kf_state, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        const int init_samples      = std::max(0, init_iter_num - 1);
        const double init_duration  = init_end_time_ - init_begin_time_;
        const double mean_acc_norm  = mean_acc.norm();
        const double mean_gyr_norm  = mean_gyr.norm();
        // A moving or vibrating window can still have a mean acceleration norm
        // near gravity. Reject it so linear acceleration is not baked into the
        // initial gravity direction.
        const double acc_std_norm   = cov_acc.cwiseMax(V3D::Zero()).cwiseSqrt().norm();
        const double gyr_std_norm   = cov_gyr.cwiseMax(V3D::Zero()).cwiseSqrt().norm();
        const bool has_enough_imu   = init_samples >= kMinImuInitSamples &&
                                    init_duration >= kMinImuInitDuration;
        const bool is_stationary    = mean_acc_norm >= kMinInitAccNorm &&
                                   mean_acc_norm <= kMaxInitAccNorm &&
                                   mean_gyr_norm <= kMaxInitMeanGyroNorm &&
                                   acc_std_norm <= kMaxInitAccStdNorm &&
                                   gyr_std_norm <= kMaxInitGyrStdNorm;
        if (has_enough_imu && !is_stationary)
        {
            RCLCPP_WARN(rclcpp::get_logger("imu_processing"),
                        "IMU initialization rejected: keep the sensor still "
                        "(samples=%d duration=%.3f s mean_acc_norm=%.6f mean_gyr_norm=%.6f "
                        "acc_std_norm=%.6f gyr_std_norm=%.6f)",
                        init_samples,
                        init_duration,
                        mean_acc_norm,
                        mean_gyr_norm,
                        acc_std_norm,
                        gyr_std_norm);
            Reset();
            return;
        }

        if (has_enough_imu)
        {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;
            last_lidar_end_time_ = meas.lidar_end_time;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            RCLCPP_INFO(rclcpp::get_logger("imu_processing"),
                        "IMU Initial Done: samples=%d duration=%.3f s "
                        "mean_acc=[%.6f, %.6f, %.6f] norm=%.6f "
                        "mean_gyr=[%.6f, %.6f, %.6f] "
                        "grav=[%.6f, %.6f, %.6f] bg=[%.6f, %.6f, %.6f] "
                        "cov_acc=[%.6g, %.6g, %.6g] cov_gyr=[%.6g, %.6g, %.6g]",
                        init_samples,
                        init_duration,
                        mean_acc[0],
                        mean_acc[1],
                        mean_acc[2],
                        mean_acc.norm(),
                        mean_gyr[0],
                        mean_gyr[1],
                        mean_gyr[2],
                        imu_state.grav[0],
                        imu_state.grav[1],
                        imu_state.grav[2],
                        imu_state.bg[0],
                        imu_state.bg[1],
                        imu_state.bg[2],
                        cov_acc[0],
                        cov_acc[1],
                        cov_acc[2],
                        cov_gyr[0],
                        cov_gyr[1],
                        cov_gyr[2]);
            // RCLCPP_INFO(rclcpp::get_logger("imu_processing"),
            // "IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc
            // covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",
            //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(),
            //          cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1],
            //          cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
            fout_imu.open(DEBUG_FILE_DIR("imu.txt"), std::ios::out);
        }

        return;
    }

    UndistortPcl(meas, kf_state, *cur_pcl_un_);
}
