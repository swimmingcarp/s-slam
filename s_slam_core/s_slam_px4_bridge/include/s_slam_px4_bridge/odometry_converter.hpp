#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <s_slam_interfaces/msg/px4_odometry.hpp>

namespace s_slam_px4_bridge
{

struct ConvertedOdometry
{
    std::int64_t sample_time_ns = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position_variance = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation_variance = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_velocity_variance = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

// Converts the FAST-LIO state into PX4's configured local FRD reference frame.
class OdometryConverter
{
public:
    OdometryConverter(const Eigen::Matrix3d &px4_world_from_source_world,
                      const Eigen::Matrix3d &source_body_from_px4_body);

    std::optional<ConvertedOdometry> convert(
        const s_slam_interfaces::msg::Px4Odometry &odometry) const;

    static bool isProperRotation(const Eigen::Matrix3d &rotation);

private:
    Eigen::Matrix3d px4_world_from_source_world_;
    Eigen::Matrix3d source_body_from_px4_body_;
};

// Advances a state to `interval_ns` after the sample it was measured at, using
// the state's own linear and angular velocity, and moves its sample time with
// it. This is what keeps the pose valid at the instant it is published rather
// than at the instant the source measured it.
ConvertedOdometry propagateOdometry(const ConvertedOdometry &odometry, std::int64_t interval_ns);

px4_msgs::msg::VehicleOdometry toVehicleOdometry(const ConvertedOdometry &odometry,
                                                 std::uint64_t publication_time_us,
                                                 std::uint8_t reset_counter);

}  // namespace s_slam_px4_bridge
