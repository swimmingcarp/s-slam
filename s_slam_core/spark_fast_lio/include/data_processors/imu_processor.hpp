#pragma once

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

// IMU initialization limits.

static constexpr int kMinImuInitSamples      = 200;
static constexpr double kMinImuInitDuration  = 2.0;
static constexpr double kMaxInitMeanGyroNorm = 0.05;
// Initialization estimates gravity from average acceleration, so the window
// must be quiet, not just close to 1g on average.
static constexpr double kMaxInitAccStdNorm   = 0.5;
static constexpr double kMaxInitGyrStdNorm   = 0.03;
static constexpr double kMinInitAccNorm      = 8.0;
static constexpr double kMaxInitAccNorm      = 11.5;


// IMU state propagation and point-cloud undistortion.
class ImuProcessor
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

    ImuProcessor();
    ~ImuProcessor();

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
    void set_replay_mode(bool replay_mode);
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

    // Arm a warm re-initialization after a mid-run estimator reset: seeds
    // gravity/biases/velocity (all expressed in the IMU body frame at re-init
    // time) from the pre-reset state so recovery does not require the
    // stationary window, which never comes while the platform is in flight.
    void setWarmStartPrior(const V3D &gravity_body,
                           const V3D &bg,
                           const V3D &ba,
                           const V3D &vel_body);

    // True once initialization (cold or warm) has committed. This — not the
    // caller's post-reset processing-progress flags — is the correct gate for
    // capturing a warm re-init prior: it flips true the instant a warm
    // re-initialization commits, closing the burst-reset window where a second
    // reset would otherwise be misjudged as a cold start.
    bool IsInitialized() const
    {
        return !imu_need_init_;
    }

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
    bool replay_mode_ = false;
    // Warm-start prior; deliberately NOT cleared by Reset() so it survives the
    // internal Reset() that IMU_init() performs on the first post-reset frame.
    bool has_warm_start_prior_   = false;
    V3D warm_start_gravity_body_ = V3D(0, 0, 0);
    V3D warm_start_bg_           = V3D(0, 0, 0);
    V3D warm_start_ba_           = V3D(0, 0, 0);
    V3D warm_start_vel_body_     = V3D(0, 0, 0);
};
