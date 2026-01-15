#pragma once

#include <string>
#include <mutex>

#include <mrs_lib/lkf.h>
#include <uvdar_ros_driver/utils/i_logger.h>

namespace uvdar_ros_driver {

struct KalmanFilterCfg {
  double sigma_range;
  double sigma_acc;
  double update_period_s;
  double dt;
  double dt_max;
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
  H_t H; // measurement mapping
  Q_t Q; // process noise (model noise)
  R_t R; // measurement noise
  u_t u; // input
  statecov_t state_cov;
  double last_time;

  std::mutex mtx;
};

class UwbKalmanFilter {
 public:
  explicit UwbKalmanFilter(ILogger& logger, const KalmanFilterCfg& cfg);

  std::optional<statecov_t> filter(const double range, const double curr_time);
  void reset(const double range, const double curr_time);

 private:
  void generateA_();
  void generateB_();
  void generateH_();
  void generateQ_();
  void generate_u_();
  void generateR_();

  void setDt_(const double dt);

 private:
  ILogger& logger_;
  KalmanFilterCfg cfg_; // TODO: maybe change later to non const, or non reference
  UwbKalmanModelCfg filter_model_;

  std::shared_ptr<lkf_t> lkf_;

  bool is_initialized_{false};
};

} // namespace uvdar_ros_driver