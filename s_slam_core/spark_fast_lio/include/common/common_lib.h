#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <cmath>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/so3_math.h"

#define G_m_s2 (9.81)    // Gravaty const in GuangDong/China
#define NUM_MATCH_POINTS (5)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

using PointType      = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector    = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using V3D            = Eigen::Vector3d;
using M3D            = Eigen::Matrix3d;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

inline M3D Eye3d(M3D::Identity());
inline V3D Zero3d(0, 0, 0);

// Lidar data and imu dates for the curent process
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        lidar_end_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    }
    double lidar_beg_time;
    double lidar_end_time;
    std::size_t lidar_point_count = 0;
    PointCloudXYZI::Ptr lidar;
    std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> imu;

    inline V3D getMeanAcc()
    {
        V3D mean_acc(Zero3d);
        if (imu.empty())
        {
            return mean_acc;
        }
        for (const auto &meas : imu)
        {
            const auto &imu_acc = meas->linear_acceleration;
            V3D cur_acc;
            cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
            mean_acc += cur_acc;
        }
        return mean_acc /= static_cast<double>(imu.size());
    }
};

struct Pose6D
{
    double offset_time;
    double acc[3], gyr[3], vel[3], pos[3], rot[9];
};

template <typename T>
auto set_pose6d(const double t,
                const Eigen::Matrix<T, 3, 1> &a,
                const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v,
                const Eigen::Matrix<T, 3, 1> &p,
                const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; ++i)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; ++j)
        {
            rot_kp.rot[i * 3 + j] = R(i, j);
        }
    }
    return rot_kp;
}

inline float calc_dist(PointType p1, PointType p2)
{
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template <typename T>
bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    if (point.size() < NUM_MATCH_POINTS)
    {
        return false;
    }

    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; ++j)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    const T n = normvec.norm();
    if (!normvec.allFinite() || !std::isfinite(n) || n <= std::numeric_limits<T>::epsilon())
    {
        return false;
    }
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; ++j)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y +
                 pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

#endif
