#pragma once

#ifndef KISS_MATCHER_INPUT_CONFIG_HPP
#define KISS_MATCHER_INPUT_CONFIG_HPP

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/qos.hpp>

namespace kiss_matcher
{
inline rclcpp::ReliabilityPolicy parseInputReliability(const std::string &value,
                                                       const std::string &parameter_name)
{
    if (value == "reliable")
    {
        return rclcpp::ReliabilityPolicy::Reliable;
    }
    if (value == "best_effort")
    {
        return rclcpp::ReliabilityPolicy::BestEffort;
    }
    throw std::invalid_argument(parameter_name + " must be 'reliable' or 'best_effort'");
}

inline const char *inputReliabilityName(const rclcpp::ReliabilityPolicy policy)
{
    return policy == rclcpp::ReliabilityPolicy::BestEffort ? "best_effort" : "reliable";
}

inline rclcpp::QoS makeInputQos(const int depth, const rclcpp::ReliabilityPolicy reliability,
                                const std::string &parameter_name)
{
    if (depth <= 0)
    {
        throw std::invalid_argument(parameter_name + " must be positive");
    }

    rclcpp::QoS qos(static_cast<std::size_t>(depth));
    qos.reliability(reliability);
    qos.durability_volatile();
    return qos;
}

inline uint32_t inputQueueSize(const int value, const std::string &parameter_name)
{
    if (value <= 0)
    {
        throw std::invalid_argument(parameter_name + " must be positive");
    }
    return static_cast<uint32_t>(value);
}
}  // namespace kiss_matcher

#endif  // KISS_MATCHER_INPUT_CONFIG_HPP
