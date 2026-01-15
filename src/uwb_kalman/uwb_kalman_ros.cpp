#include <uvdar_ros_driver/uwb_kalman/uwb_kalman_ros.h>

namespace uvdar_ros_driver {

/* onInit //{ */
void UwbKalmanFilterNodelet::onInit() {
  nh_     = nodelet::Nodelet::getMTPrivateNodeHandle();
  logger_ = std::make_shared<RosLogger>("UwbKalmanFilterNodelet");

  ros::Time::waitForValid();
  loadParams_();

  filter_ = std::make_unique<UwbKalmanFilter>(*logger_, _filter_cfg_);

  startMainLoop_();

  ROS_INFO("[UwbKalmanFilterNodelet]: Initialized.");
  initialized_ = true;
}
//}

/* startMainLoop_ //{ */
void UwbKalmanFilterNodelet::startMainLoop_() {
  main_timer_ = nh_.createTimer(ros::Duration(_filter_cfg_.update_period_s), &UwbKalmanFilterNodelet::spinOnce_, this);
}
//}

/* loadParams_ //{ */
void UwbKalmanFilterNodelet::loadParams_() {
  nh_.getParam("update_period_s", _filter_cfg_.update_period_s);
  nh_.getParam("dt_max_s", _filter_cfg_.dt_max);
  nh_.getParam("sigma_range", _filter_cfg_.sigma_range);
  nh_.getParam("sigma_acc", _filter_cfg_.sigma_acc);

  // param_loader_ = std::make_unique<mrs_lib::ParamLoader>(nh_);

  // TODO: add later on
  // std::string uav_name_;
  // param_loader_->loadParam("uav_name", uav_name_);

  // | ------- check if all parameters loaded successfully ------ |
  // if (!param_loader_->loadedSuccessfully()) {
  //   ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", ros::this_node::getName().c_str());
  //   ros::shutdown();
  // }
}
//}

/* spinOnce_ //{ */
void UwbKalmanFilterNodelet::spinOnce_([[maybe_unused]] const ros::TimerEvent& e) {
}
//}

} // namespace uvdar_ros_driver

PLUGINLIB_EXPORT_CLASS(uvdar_ros_driver::UwbKalmanFilterNodelet, nodelet::Nodelet)
