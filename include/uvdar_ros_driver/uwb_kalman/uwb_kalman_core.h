#pragma once

#include <mutex>

#include <mrs_lib/lkf.h>

namespace uvdar_ros_driver {

struct KalmanFilterCfg {
  double update_period_s;
  double dt;
};

namespace uwb_generic {

constexpr int n_states       = 2;
constexpr int n_inputs       = 0;
constexpr int n_measurements = 1;

} // namespace uwb_generic

using lkf_t      = mrs_lib::LKF<uwb_generic::n_states, uwb_generic::n_inputs, uwb_generic::n_measurements>;
using A_t        = lkf_t::A_t;
using B_t        = lkf_t::B_t;
using H_t        = lkf_t::H_t;
using Q_t        = lkf_t::Q_t;
using x_t        = lkf_t::x_t;
using P_t        = lkf_t::P_t;
using u_t        = lkf_t::u_t;
using z_t        = lkf_t::z_t;
using R_t        = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

struct UwbKalmanModelCfg {
  A_t A; // system transition
  B_t B; // input matrix
  H_t H; // state to measurement
  Q_t Q; // process noise (model noise)
  R_t R; // measurement noise
  u_t u; // input
  statecov_t state_cov;

  std::mutex mtx;

  // UwbKalmanModelCfg() {
  //   A.resize(uwb_generic::n_states, uwb_generic::n_states);
  //   B = B_t();
  //   H.resize(uwb_generic::n_measurements, uwb_generic::n_states);
  //   Q.resize(uwb_generic::n_states, uwb_generic::n_states);
  //   u = u_t();
  // }
};

class UwbKalmanFilter {
 public:
  explicit UwbKalmanFilter(const KalmanFilterCfg& cfg);

  void filter(const double range);
  void setDt(const double dt);

 private:
  void generateA_();
  void generateB_();
  void generateH_();
  void generateQ_();
  void generate_u_();
  void generateR_();

 private:
  KalmanFilterCfg cfg_; // TODO: maybe change later to non const, or non reference
  UwbKalmanModelCfg filter_model_;

  std::shared_ptr<lkf_t> lkf_;
};

} // namespace uvdar_ros_driver