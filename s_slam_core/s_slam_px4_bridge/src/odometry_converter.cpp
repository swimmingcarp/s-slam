#include "s_slam_px4_bridge/odometry_converter.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace s_slam_px4_bridge
{
namespace
{
constexpr double kRotationTolerance = 1e-6;
constexpr double kCovarianceTolerance = 1e-9;

Eigen::Matrix<double, 6, 6> poseCovariance(
    const s_slam_interfaces::msg::Px4Odometry &odometry)
{
    Eigen::Matrix<double, 6, 6> covariance;
    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            covariance(row, column) = odometry.pose_covariance[row * 6 + column];
        }
    }
    return covariance;
}

Eigen::Matrix3d linearVelocityCovariance(const s_slam_interfaces::msg::Px4Odometry &odometry)
{
    Eigen::Matrix3d covariance;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            covariance(row, column) = odometry.linear_velocity_covariance[row * 3 + column];
        }
    }
    return covariance;
}

bool isValidCovariance(const Eigen::Matrix3d &covariance)
{
    if (!covariance.allFinite() ||
        (covariance - covariance.transpose()).norm() > kCovarianceTolerance)
    {
        return false;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    return eigen_solver.info() == Eigen::Success &&
           eigen_solver.eigenvalues().minCoeff() >= -kCovarianceTolerance;
}
}  // namespace

OdometryConverter::OdometryConverter(const Eigen::Matrix3d &px4_world_from_source_world,
                                     const Eigen::Matrix3d &source_body_from_px4_body)
  : px4_world_from_source_world_(px4_world_from_source_world),
    source_body_from_px4_body_(source_body_from_px4_body)
{
    if (!isProperRotation(px4_world_from_source_world_) ||
        !isProperRotation(source_body_from_px4_body_))
    {
        throw std::invalid_argument("PX4 frame conversion matrices must be proper rotations");
    }
}

std::optional<ConvertedOdometry> OdometryConverter::convert(
    const s_slam_interfaces::msg::Px4Odometry &odometry) const
{
    const std::int64_t sample_time_ns =
        static_cast<std::int64_t>(odometry.header.stamp.sec) * 1000000000LL +
        odometry.header.stamp.nanosec;
    if (sample_time_ns <= 0)
    {
        return std::nullopt;
    }

    const Eigen::Vector3d source_position(
        odometry.pose.position.x, odometry.pose.position.y, odometry.pose.position.z);
    const Eigen::Quaterniond source_orientation(
        odometry.pose.orientation.w,
        odometry.pose.orientation.x,
        odometry.pose.orientation.y,
        odometry.pose.orientation.z);
    const Eigen::Vector3d source_linear_velocity(
        odometry.linear_velocity.x, odometry.linear_velocity.y, odometry.linear_velocity.z);
    const Eigen::Matrix<double, 6, 6> source_covariance = poseCovariance(odometry);
    const Eigen::Matrix3d source_linear_velocity_covariance = linearVelocityCovariance(odometry);

    if (!source_position.allFinite() || !source_orientation.coeffs().allFinite() ||
        source_orientation.squaredNorm() <= std::numeric_limits<double>::epsilon() ||
        !source_linear_velocity.allFinite() || !source_covariance.allFinite() ||
        !source_linear_velocity_covariance.allFinite())
    {
        return std::nullopt;
    }

    const Eigen::Matrix3d source_position_covariance = source_covariance.block<3, 3>(0, 0);
    const Eigen::Matrix3d source_orientation_covariance = source_covariance.block<3, 3>(3, 3);
    if (!isValidCovariance(source_position_covariance) ||
        !isValidCovariance(source_orientation_covariance) ||
        !isValidCovariance(source_linear_velocity_covariance))
    {
        return std::nullopt;
    }

    ConvertedOdometry converted;
    converted.sample_time_ns = sample_time_ns;
    converted.position = px4_world_from_source_world_ * source_position;
    converted.orientation = Eigen::Quaterniond(
        px4_world_from_source_world_ * source_orientation.normalized().toRotationMatrix() *
        source_body_from_px4_body_);
    converted.orientation.normalize();
    converted.position_variance =
        (px4_world_from_source_world_ * source_position_covariance *
         px4_world_from_source_world_.transpose())
            .diagonal()
            .cwiseMax(0.0);
    // FAST-LIO publishes its pose orientation covariance in map-frame axes,
    // while PX4 VehicleOdometry expects attitude variance in body-FRD axes.
    const Eigen::Matrix3d px4_orientation_covariance =
        converted.orientation.toRotationMatrix().transpose() * px4_world_from_source_world_ *
        source_orientation_covariance * px4_world_from_source_world_.transpose() *
        converted.orientation.toRotationMatrix();
    converted.orientation_variance = px4_orientation_covariance.diagonal().cwiseMax(0.0);
    converted.linear_velocity = px4_world_from_source_world_ * source_linear_velocity;
    converted.linear_velocity_variance =
        (px4_world_from_source_world_ * source_linear_velocity_covariance *
         px4_world_from_source_world_.transpose())
            .diagonal()
            .cwiseMax(0.0);

    if (!converted.position.allFinite() || !converted.orientation.coeffs().allFinite() ||
        !converted.position_variance.allFinite() || !converted.orientation_variance.allFinite() ||
        !converted.linear_velocity.allFinite() || !converted.linear_velocity_variance.allFinite())
    {
        return std::nullopt;
    }
    return converted;
}

bool OdometryConverter::isProperRotation(const Eigen::Matrix3d &rotation)
{
    return rotation.allFinite() &&
           (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <=
               kRotationTolerance &&
           std::abs(rotation.determinant() - 1.0) <= kRotationTolerance;
}

px4_msgs::msg::VehicleOdometry toVehicleOdometry(const ConvertedOdometry &odometry,
                                                 std::uint64_t publication_time_us,
                                                 std::uint8_t reset_counter)
{
    px4_msgs::msg::VehicleOdometry px4_odometry{};
    px4_odometry.timestamp = publication_time_us;
    px4_odometry.timestamp_sample = static_cast<std::uint64_t>(odometry.sample_time_ns / 1000);
    px4_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    px4_odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
    px4_odometry.position = {static_cast<float>(odometry.position.x()),
                             static_cast<float>(odometry.position.y()),
                             static_cast<float>(odometry.position.z())};
    px4_odometry.q = {static_cast<float>(odometry.orientation.w()),
                      static_cast<float>(odometry.orientation.x()),
                      static_cast<float>(odometry.orientation.y()),
                      static_cast<float>(odometry.orientation.z())};
    px4_odometry.velocity = {static_cast<float>(odometry.linear_velocity.x()),
                             static_cast<float>(odometry.linear_velocity.y()),
                             static_cast<float>(odometry.linear_velocity.z())};
    px4_odometry.position_variance = {static_cast<float>(odometry.position_variance.x()),
                                      static_cast<float>(odometry.position_variance.y()),
                                      static_cast<float>(odometry.position_variance.z())};
    px4_odometry.orientation_variance = {static_cast<float>(odometry.orientation_variance.x()),
                                         static_cast<float>(odometry.orientation_variance.y()),
                                         static_cast<float>(odometry.orientation_variance.z())};
    px4_odometry.velocity_variance = {
        static_cast<float>(odometry.linear_velocity_variance.x()),
        static_cast<float>(odometry.linear_velocity_variance.y()),
        static_cast<float>(odometry.linear_velocity_variance.z())};
    px4_odometry.angular_velocity.fill(std::numeric_limits<float>::quiet_NaN());
    px4_odometry.reset_counter = reset_counter;
    px4_odometry.quality = 0;
    return px4_odometry;
}

SampleRateSelector::Selection SampleRateSelector::select(std::int64_t sample_time_ns,
                                                          std::int64_t target_interval_ns)
{
    if (sample_time_ns <= 0 || target_interval_ns <= 0)
    {
        return Selection::kNone;
    }
    if (!previous_sample_time_ns_.has_value())
    {
        previous_sample_time_ns_ = sample_time_ns;
        next_target_time_ns_ = sample_time_ns + target_interval_ns;
        last_selected_sample_time_ns_ = sample_time_ns;
        return Selection::kCurrent;
    }
    if (sample_time_ns <= *previous_sample_time_ns_)
    {
        return Selection::kNone;
    }

    const std::int64_t previous_sample_time_ns = *previous_sample_time_ns_;
    previous_sample_time_ns_ = sample_time_ns;
    if (sample_time_ns < *next_target_time_ns_)
    {
        return Selection::kNone;
    }
    if (sample_time_ns - *next_target_time_ns_ >= target_interval_ns)
    {
        *next_target_time_ns_ = sample_time_ns + target_interval_ns;
        if (last_selected_sample_time_ns_.has_value() &&
            sample_time_ns <= *last_selected_sample_time_ns_)
        {
            return Selection::kNone;
        }
        last_selected_sample_time_ns_ = sample_time_ns;
        return Selection::kCurrent;
    }

    const std::int64_t previous_distance = *next_target_time_ns_ - previous_sample_time_ns;
    const std::int64_t current_distance = sample_time_ns - *next_target_time_ns_;
    const Selection selection = previous_distance < current_distance ? Selection::kPrevious
                                                                       : Selection::kCurrent;
    const std::int64_t selected_sample_time_ns =
        selection == Selection::kPrevious ? previous_sample_time_ns : sample_time_ns;

    // A gap may cross several target periods. Publish the newest candidate once
    // and resume from the next target after that source measurement.
    *next_target_time_ns_ += target_interval_ns;
    if (*next_target_time_ns_ <= sample_time_ns)
    {
        *next_target_time_ns_ = sample_time_ns + target_interval_ns;
    }

    if (last_selected_sample_time_ns_.has_value() &&
        selected_sample_time_ns <= *last_selected_sample_time_ns_)
    {
        return Selection::kNone;
    }
    last_selected_sample_time_ns_ = selected_sample_time_ns;
    return selection;
}

void SampleRateSelector::reset()
{
    previous_sample_time_ns_.reset();
    next_target_time_ns_.reset();
    last_selected_sample_time_ns_.reset();
}

}  // namespace s_slam_px4_bridge
