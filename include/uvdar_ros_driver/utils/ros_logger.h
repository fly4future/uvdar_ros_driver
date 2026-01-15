#pragma once

#include <ros/ros.h>
#include <uvdar_ros_driver/utils/i_logger.h>

class RosLogger : public ILogger {
 public:
  explicit RosLogger(const std::string& node_name) : node_name_(node_name) {
  }

  void log(const LogLevel level, const std::string& msg) override {
    // clang-format off
    if (level == LogLevel::Info) {
      ROS_INFO("[%s] %s", node_name_.c_str(), msg.c_str());
    } 
    else if (level == LogLevel::Warn) {
      ROS_WARN("[%s] %s", node_name_.c_str(), msg.c_str());
    } 
    else if (level == LogLevel::Debug) {
      ROS_DEBUG("[%s] %s", node_name_.c_str(), msg.c_str());
    } 
    else if (level == LogLevel::Error) {
      ROS_ERROR("[%s] %s", node_name_.c_str(), msg.c_str());
    } 
    else {
      ROS_ERROR("Given LogLevel is not supported!");
    }
    // clang-format on
  }

 private:
  const std::string node_name_;
};