#pragma once

#include <set>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace wxr_ros2 {

class ActivityTracker {
public:
  ActivityTracker(rclcpp::Node & node, std::chrono::milliseconds interval = std::chrono::milliseconds(1000))
  : node_(node)
  {
    timer_ = node_.create_wall_timer(interval, std::bind(&ActivityTracker::tick, this));
  }

  void markSub(const std::string & topic) { sub_.insert(topic); }
  void markPub(const std::string & topic) { pub_.insert(topic); }

private:
  void tick()
  {
    if (!sub_.empty()) {
      std::string joined;
      for (auto it = sub_.begin(); it != sub_.end(); ++it) {
        if (it != sub_.begin()) joined += ", ";
        joined += *it;
      }
      RCLCPP_INFO(node_.get_logger(), "[Activity] sub: %s", joined.c_str());
    }
    if (!pub_.empty()) {
      std::string joined;
      for (auto it = pub_.begin(); it != pub_.end(); ++it) {
        if (it != pub_.begin()) joined += ", ";
        joined += *it;
      }
      RCLCPP_INFO(node_.get_logger(), "[Activity] pub: %s", joined.c_str());
    }
    sub_.clear();
    pub_.clear();
  }

  rclcpp::Node & node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::set<std::string> sub_;
  std::set<std::string> pub_;
};

} // namespace wxr_ros2


