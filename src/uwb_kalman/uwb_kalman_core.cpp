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

  lkf_ = std::make_shared<lkf_t>(filter_model_.A, filter_model_.B, filter_model_.H);
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
  // clang-format off
  filter_model_.Q << 
    std::pow(cfg_.dt, 4)/4.0,   std::pow(cfg_.dt, 3)/2.0,
    std::pow(cfg_.dt, 3)/2.0,   std::pow(cfg_.dt, 2);
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
  // filter_model_.R =; //TODO
}
//}

/*//{ setDt */
void UwbKalmanFilter::setDt(const double dt) {
  cfg_.dt = dt;
  generateA_();
  generateB_();
  std::scoped_lock lock(filter_model_.mtx);
  lkf_->A = filter_model_.A;
  lkf_->B = filter_model_.B;
}
/*//}*/

/* filter //{ */
void UwbKalmanFilter::filter(const double range) {

  // 1. predict
  // sc -> statecov_t
  // sc = lkf_->predict(sc, u, Q, dt_);

  // 2. compute kalman gain
  // sc = lkf_->correct(sc, z, R_t::Ones() * R);

  // 3. compute the estimate
  // 4. compute the covariance error
}
//}

} // namespace uvdar_ros_driver