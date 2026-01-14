#include <uvdar_ros_driver/uwb_kalman/uwb_kalman_core.h>

namespace uvdar_ros_driver {

/* UwbKalmanFilter constructor //{ */
UwbKalmanFilter::UwbKalmanFilter(const KalmanFilterCfg& cfg) {
  cfg_ = cfg;

  generateA_();
  generateB_();
  generateH_();
  generateQ_();
  generate_u_();
  generateR_();

  // | --------------- Kalman filter intialization -------------- |
  const x_t x0 = x_t::Zero();
  const P_t P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  filter_model_.state_cov = sc0;
  lkf_                    = std::make_shared<lkf_t>(filter_model_.A, filter_model_.B, filter_model_.H);
}
//}

/* generateA_ //{ */
void UwbKalmanFilter::generateA_() {
  // clang-format off
  filter_model_.A <<
    1,  cfg_.dt,
    0,  1;
  // clang-format on
}
//}

/* generateA_ //{ */
void UwbKalmanFilter::generateB_() {
  filter_model_.B = B_t::Zero(uwb_generic::n_states, uwb_generic::n_inputs);
}
//}

/* generateH_ //{ */
void UwbKalmanFilter::generateH_() {
  // clang-format off
  filter_model_.H <<
   1, 0;
  // clang-format on
}
//}

/* generateQ_ //{ */
void UwbKalmanFilter::generateQ_() {
  const double dt  = cfg_.dt;
  const double sa2 = cfg_.sigma_acc * cfg_.sigma_acc;

  // clang-format off
  filter_model_.Q << 
    sa2*std::pow(dt, 4)/4.0,   sa2*std::pow(dt, 3)/2.0,
    sa2*std::pow(dt, 3)/2.0,   sa2*std::pow(dt, 2);
  // clang-format on
}
//}

/* generate_u_ //{ */
void UwbKalmanFilter::generate_u_() {
  filter_model_.u = u_t::Zero(uwb_generic::n_inputs);
}
//}

/* generateR_ //{ */
void UwbKalmanFilter::generateR_() {
  filter_model_.R(0, 0) = cfg_.sigma_range * cfg_.sigma_range;
}
//}

/*//{ setDt */
void UwbKalmanFilter::setDt_(const double dt) {
  std::scoped_lock lock(filter_model_.mtx);

  cfg_.dt = dt;
  generateA_();
  generateB_();
  generateQ_();
  lkf_->A = filter_model_.A;
  lkf_->B = filter_model_.B;
}
/*//}*/

/* reset //{ */
void UwbKalmanFilter::reset(const double range, const double curr_time) {
  std::scoped_lock lock(filter_model_.mtx);

  filter_model_.state_cov.x.setZero();
  filter_model_.state_cov.x(0) = range;
  filter_model_.state_cov.x(1) = 0.0;
  filter_model_.state_cov.P    = 1e3 * P_t::Identity();

  last_time_ = curr_time;
}
/*//}*/

/* filter //{ */
std::optional<statecov_t> UwbKalmanFilter::filter(const double range, const double curr_time) {
  if (!is_initialized_) {
    reset(range, curr_time);
    is_initialized_ = true;

    return filter_model_.state_cov;
  }

  const auto dt = curr_time - last_time_;
  if (dt <= 0.0) {
    return std::nullopt;
  }

  if (dt >= cfg_.dt_max) {
    reset(range, curr_time);
    return filter_model_.state_cov;
  }

  setDt_(dt);
  last_time_ = curr_time;

  try {
    std::scoped_lock lock(filter_model_.mtx);
    filter_model_.state_cov = lkf_->predict(filter_model_.state_cov, filter_model_.u, filter_model_.Q, cfg_.dt);

    z_t z;
    z(0) = range;

    filter_model_.state_cov = lkf_->correct(filter_model_.state_cov, z, filter_model_.R);

  } catch (const std::exception& e) {
    // In case of error, alert the user
    ROS_ERROR("[UwbKalmanFilter]: LKF prediction failed: %s", e.what());
  }
  return filter_model_.state_cov;
}
//}

} // namespace uvdar_ros_driver