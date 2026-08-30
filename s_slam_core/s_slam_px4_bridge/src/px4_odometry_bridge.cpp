#include "s_slam_px4_bridge/odometry_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <s_slam_interfaces/msg/px4_odometry.hpp>

namespace s_slam_px4_bridge
{
namespace
{
constexpr std::size_t kSourceOdometryQueueDepth = 10;

Eigen::Matrix3d rotationFromParameter(const std::vector<double> &values, const std::string &name)
{
    if (values.size() != 9 ||
        !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); }))
    {
        throw std::invalid_argument(name + " must contain exactly nine finite row-major values");
    }

    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            rotation(row, column) = values[row * 3 + column];
        }
    }
    return rotation;
}

std::int64_t rateIntervalNs(double publish_rate_hz)
{
    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 || publish_rate_hz > 200.0)
    {
        throw std::invalid_argument("publish_rate_hz must be in (0, 200]");
    }
    return static_cast<std::int64_t>(std::llround(1.0e9 / publish_rate_hz));
}
}  // namespace

class Px4OdometryBridge final : public rclcpp::Node
{
public:
    Px4OdometryBridge()
      : Node("px4_odometry_bridge"),
        enabled_(declare_parameter<bool>("enabled", false)),
        publish_interval_ns_(rateIntervalNs(declare_parameter<double>("publish_rate_hz", 30.0))),
        source_world_frame_(declare_parameter<std::string>("source_world_frame", "odom")),
        source_child_frame_(declare_parameter<std::string>("source_child_frame", "base_link")),
        converter_(rotationFromParameter(
                       declare_parameter<std::vector<double>>(
                           "px4_world_from_source_world", std::vector<double>{1.0, 0.0, 0.0,
                                                                                0.0, 1.0, 0.0,
                                                                                0.0, 0.0, 1.0}),
                       "px4_world_from_source_world"),
                   rotationFromParameter(
                       declare_parameter<std::vector<double>>(
                           "source_body_from_px4_body", std::vector<double>{1.0, 0.0, 0.0,
                                                                                0.0, 1.0, 0.0,
                                                                                0.0, 0.0, 1.0}),
                       "source_body_from_px4_body"))
    {
        const std::string source_topic = declare_parameter<std::string>("source_topic", "px4_odometry");
        const std::string output_topic =
            declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");

        output_publisher_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            output_topic, rclcpp::QoS(1).best_effort());
        rclcpp::QoS source_qos{rclcpp::KeepLast(kSourceOdometryQueueDepth)};
        // This is an on-companion state handoff, not the PX4 transport. Keep a
        // short reliable history so the timestamp selector sees each candidate.
        source_qos.reliable();
        source_qos.durability_volatile();
        px4_odometry_subscription_ = create_subscription<s_slam_interfaces::msg::Px4Odometry>(
            source_topic,
            source_qos,
            std::bind(&Px4OdometryBridge::handleOdometry, this, std::placeholders::_1));

        if (enabled_)
        {
            RCLCPP_INFO(get_logger(),
                        "Publishing PX4 visual odometry at up to %.2f Hz from '%s' to '%s'.",
                        1.0e9 / static_cast<double>(publish_interval_ns_),
                        source_topic.c_str(),
                        output_topic.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "PX4 odometry bridge is disabled. Calibrate both frame rotations and set "
                        "enabled:=true before flight use.");
        }
    }

private:
    void handleOdometry(const s_slam_interfaces::msg::Px4Odometry::SharedPtr message)
    {
        if (!enabled_)
        {
            return;
        }
        if (message->header.frame_id != source_world_frame_ ||
            message->body_frame != source_child_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Dropping odometry with frames '%s' -> '%s'; expected '%s' -> '%s'.",
                message->header.frame_id.c_str(),
                message->body_frame.c_str(),
                source_world_frame_.c_str(),
                source_child_frame_.c_str());
            return;
        }

        const std::optional<ConvertedOdometry> converted = converter_.convert(*message);
        if (!converted.has_value())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000, "Dropping invalid FAST-LIO odometry sample.");
            return;
        }

        if (message->reset_counter != reset_counter_)
        {
            reset_counter_ = message->reset_counter;
            rate_selector_.reset();
            previous_odometry_.reset();
            rate_selector_.select(converted->sample_time_ns, publish_interval_ns_);
            previous_odometry_ = *converted;
            RCLCPP_WARN(get_logger(), "FAST-LIO reset counter advanced to %u.", reset_counter_);
            publishConvertedOdometry(*converted);
            return;
        }

        const SampleRateSelector::Selection selection =
            rate_selector_.select(converted->sample_time_ns, publish_interval_ns_);
        if (selection == SampleRateSelector::Selection::kPrevious && previous_odometry_.has_value())
        {
            publishConvertedOdometry(*previous_odometry_);
        }
        else if (selection == SampleRateSelector::Selection::kCurrent)
        {
            publishConvertedOdometry(*converted);
        }
        previous_odometry_ = *converted;
    }

    void publishConvertedOdometry(const ConvertedOdometry &converted)
    {
        const px4_msgs::msg::VehicleOdometry px4_odometry = toVehicleOdometry(
            converted,
            static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000),
            reset_counter_);
        output_publisher_->publish(px4_odometry);
    }

    bool enabled_;
    std::int64_t publish_interval_ns_;
    std::string source_world_frame_;
    std::string source_child_frame_;
    OdometryConverter converter_;
    SampleRateSelector rate_selector_;
    std::optional<ConvertedOdometry> previous_odometry_;
    std::uint8_t reset_counter_ = 0;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr output_publisher_;
    rclcpp::Subscription<s_slam_interfaces::msg::Px4Odometry>::SharedPtr px4_odometry_subscription_;
};

}  // namespace s_slam_px4_bridge

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<s_slam_px4_bridge::Px4OdometryBridge>());
    rclcpp::shutdown();
    return 0;
}
