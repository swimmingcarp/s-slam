#pragma once

#include <deque>
#include <memory>
#include <vector>

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
        Eigen::Matrix<double, 12, 12> process_noise_covariance;
        V3D accelerometer_covariance;
        V3D gyroscope_covariance;
        V3D accelerometer_covariance_scale;
        V3D gyroscope_covariance_scale;
        V3D gyroscope_bias_covariance;
        V3D accelerometer_bias_covariance;
        std::shared_ptr<const sensor_msgs::msg::Imu> last_imu;
        std::vector<Pose6D> imu_poses;
        M3D lidar_rotation_wrt_imu;
        V3D lidar_translation_wrt_imu;
        V3D mean_acceleration;
        V3D mean_angular_velocity;
        V3D last_angular_velocity;
        V3D last_acceleration_world;
        double start_timestamp = -1.0;
        double last_lidar_end_time = -1.0;
        double init_begin_time = -1.0;
        double init_end_time = -1.0;
        int init_sample_count = 1;
        bool is_first_frame = true;
        bool needs_initialization = true;
    };

    ImuProcessor();
    ~ImuProcessor() = default;

    void reset();
    Snapshot getSnapshot() const;
    void restoreSnapshot(const Snapshot &snapshot);
    void setExtrinsic(const V3D &translation, const M3D &rotation);
    void setExtrinsic(const V3D &translation);
    void setExtrinsic(const MD(4, 4) &transform);
    void setGyroscopeCovariance(const V3D &covariance);
    void setAccelerometerCovariance(const V3D &covariance);
    void setGyroscopeBiasCovariance(const V3D &covariance);
    void setAccelerometerBiasCovariance(const V3D &covariance);
    void setReplayMode(bool replay_mode);
    void process(const MeasureGroup &measures,
                 esekfom::esekf<state_ikfom, 12, input_ikfom> &filter,
                 PointCloudXYZI::Ptr undistorted_cloud);
    state_ikfom integrateImu(const std::deque<sensor_msgs::msg::Imu> &imu_queue,
                             esekfom::esekf<state_ikfom, 12, input_ikfom> &filter);

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
    bool isInitialized() const
    {
        return !needs_initialization_;
    }

private:
    void initializeImu(const MeasureGroup &measures,
                       esekfom::esekf<state_ikfom, 12, input_ikfom> &filter,
                       int &init_sample_count);
    void undistortPointCloud(const MeasureGroup &measures,
                             esekfom::esekf<state_ikfom, 12, input_ikfom> &filter,
                             PointCloudXYZI &point_cloud);

    std::shared_ptr<const sensor_msgs::msg::Imu> last_imu_;
    Eigen::Matrix<double, 12, 12> process_noise_covariance_;
    V3D accelerometer_covariance_;
    V3D gyroscope_covariance_;
    V3D accelerometer_covariance_scale_;
    V3D gyroscope_covariance_scale_;
    V3D gyroscope_bias_covariance_;
    V3D accelerometer_bias_covariance_;

    std::vector<Pose6D> imu_poses_;
    M3D lidar_rotation_wrt_imu_;
    V3D lidar_translation_wrt_imu_;
    V3D mean_acceleration_;
    V3D mean_angular_velocity_;
    V3D last_angular_velocity_;
    V3D last_acceleration_world_;
    double start_timestamp_;
    double last_lidar_end_time_;
    double init_begin_time_;
    double init_end_time_;
    int init_sample_count_       = 1;
    bool is_first_frame_         = true;
    bool needs_initialization_   = true;
    bool replay_mode_            = false;
    // Warm-start prior; deliberately NOT cleared by reset() so it survives the
    // internal reset() that initializeImu() performs on the first post-reset frame.
    bool has_warm_start_prior_   = false;
    V3D warm_start_gravity_body_ = V3D(0, 0, 0);
    V3D warm_start_bg_           = V3D(0, 0, 0);
    V3D warm_start_ba_           = V3D(0, 0, 0);
    V3D warm_start_vel_body_     = V3D(0, 0, 0);
};
