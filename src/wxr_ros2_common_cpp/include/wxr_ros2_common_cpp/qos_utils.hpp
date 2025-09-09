#pragma once

#include <string>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

namespace wxr_ros2 {

inline std::string qosToBriefString(const rclcpp::QoS & qos)
{
  const auto p = qos.get_rmw_qos_profile();
  auto rel = (p.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) ? "RELIABLE" :
             (p.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) ? "BEST_EFFORT" : "SYSTEM_DEFAULT";
  auto dur = (p.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) ? "TRANSIENT_LOCAL" :
             (p.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE) ? "VOLATILE" : "SYSTEM_DEFAULT";
  auto hist = (p.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) ? "KEEP_ALL" :
              (p.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST) ? "KEEP_LAST" : "SYSTEM_DEFAULT";
  std::string liv;
  switch (p.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC: liv = "AUTOMATIC"; break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC: liv = "MANUAL_BY_TOPIC"; break;
    default: liv = "SYSTEM_DEFAULT"; break;
  }
  char depth_buf[32];
  std::snprintf(depth_buf, sizeof(depth_buf), "%zu", static_cast<size_t>(p.depth));
  return std::string("\n    history: ") + hist +
         "\n    depth: " + depth_buf +
         "\n    reliability: " + rel +
         "\n    durability: " + dur +
         "\n    liveliness: " + liv;
}

} // namespace wxr_ros2

