#include "s_slam_px4_bridge/odometry_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
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

// How far a state may be carried forward on its own velocity and body rate.
// The limit is an error budget for holding both constant: over 30 ms that
// costs about 4 mm at the 8 m/s^2 the frontend declares as its acceleration
// limit, and about a degree of attitude at the angular acceleration a
// multirotor reaches. A hundred milliseconds would cost 4 cm and several
// degrees, which is no longer a correction. The source normally runs far
// inside this: its newest state is a few milliseconds old whenever the
// frontend keeps up. An older state is still sent, stamped at the last time
// its propagation is trustworthy, so PX4 fuses it as the delayed measurement
// it is rather than as an extrapolation nobody can defend.
constexpr std::int64_t kMaximumPropagationNs = 30000000;

// A sample stamped in the companion time base arrives within the frontend's own
// pipeline latency, which is milliseconds. A whole second can only be a
// different time base, so this reports that rather than gating on it: the
// offset is a deployment fault for a human to fix, not a sample to drop.
constexpr std::int64_t kMaximumTimeBaseOffsetNs = 1000000000;

// VehicleOdometry carries microseconds, so keeping the published sample time
// increasing means clearing a whole microsecond, not a nanosecond.
constexpr std::int64_t kPublishedTimeResolutionNs = 1000;

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
        // short reliable history so a source burst is not lost before the
        // publication timer reads the newest state from it.
        source_qos.reliable();
        source_qos.durability_volatile();
        px4_odometry_subscription_ = create_subscription<s_slam_interfaces::msg::Px4Odometry>(
            source_topic,
            source_qos,
            std::bind(&Px4OdometryBridge::handleOdometry, this, std::placeholders::_1));

        if (enabled_)
        {
            publish_timer_ = rclcpp::create_timer(
                this,
                get_clock(),
                rclcpp::Duration::from_nanoseconds(publish_interval_ns_),
                std::bind(&Px4OdometryBridge::publishLatestOdometry, this));
            RCLCPP_INFO(get_logger(),
                        "Publishing PX4 visual odometry at %.2f Hz from '%s' to '%s'.",
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
            // A reset is the one place the source time base may step backwards,
            // so the published state anchors on this sample rather than on the
            // one before it.
            last_valid_time_ns_.reset();
            last_published_sample_time_ns_.reset();
            RCLCPP_WARN(get_logger(), "FAST-LIO reset counter advanced to %u.", reset_counter_);
        }
        else if (latest_odometry_.has_value() &&
                 converted->sample_time_ns <= latest_odometry_->sample_time_ns)
        {
            // Keep the published sample time monotonic: a repeated or reordered
            // source sample carries no state PX4 has not already been given.
            return;
        }

        latest_odometry_ = *converted;
        // The hold interval is measured between this arrival and the slot that
        // sends the state, both read from the companion clock. Subtracting the
        // source stamp instead would measure the offset between the sensor and
        // companion time bases, which is not an age and does not shrink when
        // the frontend keeps up.
        latest_arrival_time_ns_ = get_clock()->now().nanoseconds();
        warnOnUnsharedTimeBase(*latest_arrival_time_ns_, converted->sample_time_ns);
    }

    // The published sample time is only meaningful to PX4 once the sensor driver
    // and the companion share a time base. Nothing downstream can detect that
    // from the state itself, and the hold interval no longer reveals it, so
    // check it here against the arrival the state was actually seen at.
    void warnOnUnsharedTimeBase(const std::int64_t arrival_time_ns,
                                const std::int64_t sample_time_ns)
    {
        const std::int64_t offset_ns = arrival_time_ns - sample_time_ns;
        if (std::abs(offset_ns) <= kMaximumTimeBaseOffsetNs)
        {
            return;
        }
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "FAST-LIO sample time is %.3f s from the companion clock it arrived "
                             "on. PX4 reads `timestamp_sample` in the companion time base, so it "
                             "cannot place this measurement. A RoboSense driver running "
                             "`use_lidar_clock` publishes the LiDAR's own free-running counter; "
                             "establish a shared time base before enabling external-vision fusion.",
                             static_cast<double>(offset_ns) * 1.0e-9);
    }

    // PX4 needs a pose for the instant it is sent, not for the instant the
    // source measured it. Each slot therefore carries the newest state advanced
    // over its own age, which also puts the published sample time on the
    // publication cadence instead of on the source's arrival pattern.
    void publishLatestOdometry()
    {
        if (!latest_odometry_.has_value() || !latest_arrival_time_ns_.has_value())
        {
            return;
        }

        const std::int64_t publication_time_ns = get_clock()->now().nanoseconds();
        const std::int64_t state_age_ns = publication_time_ns - *latest_arrival_time_ns_;
        const std::int64_t propagation_ns =
            std::clamp(state_age_ns, static_cast<std::int64_t>(0), kMaximumPropagationNs);
        if (state_age_ns > kMaximumPropagationNs)
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 1000,
                                 "Newest FAST-LIO state has been held %.3f s, beyond the %.0f ms "
                                 "it can be propagated; PX4 is receiving a delayed pose. The "
                                 "frontend is not keeping up with the publication rate.",
                                 static_cast<double>(state_age_ns) * 1.0e-9,
                                 static_cast<double>(kMaximumPropagationNs) * 1.0e-6);
        }

        // Whether the source has moved on is a property of the source alone. The
        // hold interval grows while the frontend is stalled, so a sum that
        // includes it keeps advancing over one unchanged state and would send
        // that state again; PX4 would fuse one measurement as several and its
        // own external-vision timeout would never see the stall.
        if (last_published_sample_time_ns_.has_value() &&
            latest_odometry_->sample_time_ns <= *last_published_sample_time_ns_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 1000,
                                 "No FAST-LIO state newer than the last one sent; leaving this "
                                 "PX4 slot empty.");
            return;
        }

        // A newer state can still carry a smaller published sample time than the
        // last slot, because the hold interval it is stamped with may be shorter.
        // PX4 places this in a delayed-measurement buffer, so carry it forward far
        // enough to keep the published time increasing.
        const std::int64_t minimum_propagation_ns =
            last_valid_time_ns_.has_value()
                ? std::max<std::int64_t>(
                      0,
                      *last_valid_time_ns_ + kPublishedTimeResolutionNs -
                          latest_odometry_->sample_time_ns)
                : 0;
        const std::int64_t published_propagation_ns =
            std::max(propagation_ns, minimum_propagation_ns);
        if (published_propagation_ns > kMaximumPropagationNs)
        {
            // Keeping the published time increasing would carry this state further
            // than the error budget allows. Leave the slot empty rather than claim
            // an extrapolation nobody can defend.
            return;
        }

        last_valid_time_ns_ = latest_odometry_->sample_time_ns + published_propagation_ns;
        last_published_sample_time_ns_ = latest_odometry_->sample_time_ns;

        publishConvertedOdometry(propagateOdometry(*latest_odometry_, published_propagation_ns),
                                 publication_time_ns);
    }

    void publishConvertedOdometry(const ConvertedOdometry &converted,
                                  const std::int64_t publication_time_ns)
    {
        const px4_msgs::msg::VehicleOdometry px4_odometry = toVehicleOdometry(
            converted,
            static_cast<std::uint64_t>(publication_time_ns / 1000),
            reset_counter_);
        output_publisher_->publish(px4_odometry);
    }

    bool enabled_;
    std::int64_t publish_interval_ns_;
    std::string source_world_frame_;
    std::string source_child_frame_;
    OdometryConverter converter_;
    std::optional<ConvertedOdometry> latest_odometry_;
    std::optional<std::int64_t> latest_arrival_time_ns_;
    std::optional<std::int64_t> last_valid_time_ns_;
    std::optional<std::int64_t> last_published_sample_time_ns_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
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
