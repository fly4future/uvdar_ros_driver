#pragma once

#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mrs_lib/param_loader.h>

#include <uvdar_ros_driver/uwb_kalman/uwb_kalman_core.h>

namespace uvdar_ros_driver {

class UwbKalmanFilterNodelet : public nodelet::Nodelet {
 public:
  virtual void onInit();

 private:
  void loadParams_();
  void startMainLoop_();

  void spinOnce_([[maybe_unused]] const ros::TimerEvent& e);

 private:
  ros::NodeHandle nh_;
  std::unique_ptr<mrs_lib::ParamLoader> param_loader_;

  KalmanFilterCfg _filter_cfg_;
  std::unique_ptr<UwbKalmanFilter> filter_;

  bool initialized_{false};

  ros::Timer main_timer_;
};

} // namespace uvdar_ros_driver