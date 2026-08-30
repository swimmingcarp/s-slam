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

px4_msgs::msg::VehicleOdometry toVehicleOdometry(const ConvertedOdometry &odometry,
                                                 std::uint64_t publication_time_us,
                                                 std::uint8_t reset_counter);

// Chooses the source sample nearest each target period. Holding one source
// sample avoids a systematic rate loss when the source period does not divide
// the target period, while remaining driven by measurement timestamps.
class SampleRateSelector
{
public:
    enum class Selection
    {
        kNone,
        kCurrent,
        kPrevious,
    };

    Selection select(std::int64_t sample_time_ns, std::int64_t target_interval_ns);
    void reset();

private:
    std::optional<std::int64_t> previous_sample_time_ns_;
    std::optional<std::int64_t> next_target_time_ns_;
    std::optional<std::int64_t> last_selected_sample_time_ns_;
};

}  // namespace s_slam_px4_bridge
