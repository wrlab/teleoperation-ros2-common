#pragma once

#include <string>
#include <ostream>
#include <rclcpp/rclcpp.hpp>

namespace wxr_ros2 {

inline void printParam(rclcpp::Node & node, const std::string & key, const std::string & value)
{
  RCLCPP_INFO(node.get_logger(), "[Param]: %s=%s", key.c_str(), value.c_str());
}

inline void logEndpoint(rclcpp::Node & node, const char * dir, const std::string & topic, const char * type, const std::string & qos_multiline)
{
  RCLCPP_INFO(node.get_logger(), "%s: %s [%s] qos=%s", dir, topic.c_str(), type, qos_multiline.c_str());
}

inline void logLatency(rclcpp::Node & node, const std::string & label, double start_sec, double end_sec, std::ostream * file = nullptr)
{
  const double latency = end_sec - start_sec;
  RCLCPP_INFO(node.get_logger(), "[Latency] %s: %.6f sec", label.c_str(), latency);
  if (file) {
    (*file) << "[Latency] " << label << ": " << latency << " sec\n";
    file->flush();
  }
}

} // namespace wxr_ros2


