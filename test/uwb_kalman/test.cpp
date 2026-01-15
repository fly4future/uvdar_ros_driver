#include <gtest/gtest.h>
#include <random>
#include <vector>
#include <cmath>
#include <fstream>

#include "../dummy_logger.h"
#include <uvdar_ros_driver/uwb_kalman/uwb_kalman_core.h>

static double rmse(const std::vector<double>& e) {
  double s = 0.0;
  for (double v : e)
    s += v * v;
  return std::sqrt(s / std::max<size_t>(1, e.size()));
}

TEST(UwbLkf, ConstantRangeReducesNoise) {
  DummyLogger logger;

  std::ofstream f("/home/user/ros_ws/src/uvdar_ros_driver/test/uwb_kf_debug.csv");
  f << "t,truth,meas,est,var\n";

  uvdar_ros_driver::KalmanFilterCfg cfg;
  cfg.dt          = 0.02;
  cfg.dt_max      = 0.5;
  cfg.sigma_acc   = 2.0;
  cfg.sigma_range = 0.3;
  uvdar_ros_driver::UwbKalmanFilter kf(logger, cfg);

  const double r_true     = 10.0;
  const double sigma_meas = 0.30;
  const double dt         = 0.05;
  const int N             = 400;

  std::mt19937 rng(123); // deterministic
  std::normal_distribution<double> gauss(0.0, sigma_meas);

  double t = 0.0;
  std::vector<double> err_meas, err_filt;

  for (int i = 0; i < N; i++) {
    t += dt;
    const double z = r_true + gauss(rng);

    auto out = kf.filter(z, t);
    if (!out)
      continue;

    const auto& sc     = *out;
    const double r_hat = sc.x(0);

    err_meas.push_back(z - r_true);
    err_filt.push_back(r_hat - r_true);

    f << t << "," << r_true << "," << z << "," << r_hat << "," << sc.P(0, 0) << "\n";

    // Covariance should stay non-negative on diagonal
    EXPECT_GE(sc.P(0, 0), 0.0);
    EXPECT_GE(sc.P(1, 1), 0.0);
  }

  const double meas_rmse = rmse(err_meas);
  const double filt_rmse = rmse(err_filt);

  // Filter should reduce RMSE vs raw measurements
  EXPECT_LT(filt_rmse, meas_rmse);
  f.close();
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}