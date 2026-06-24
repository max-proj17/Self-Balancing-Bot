#pragma once
#include <array>

// ── Kalman filter for a 2D state: [angle, rate] ───────────────────────────
//
//  State:   x  = [angle, rate]
//  Dynamics: F = [[1, dt], [0, 1]]   (constant-rate prediction)
//  Observation: H = I₂               (both states measured directly)
//  Process noise:  Q = diag(Q_angle, Q_rate)
//  Sensor noise:   R = diag(R_angle, R_rate)
//
// Usage:
//   KalmanFilter kf;
//   kf.reset();
//   double filtered_angle = kf.step(meas_angle, meas_rate, dt);
//   double filtered_rate  = kf.rate();

class KalmanFilter {
public:
    // Noise parameters
    double Q_angle = 1e-5;   // process noise: angle
    double Q_rate  = 1e-3;   // process noise: rate
    double R_angle = 0.01;   // sensor noise: angle
    double R_rate  = 0.01;   // sensor noise: rate

    // Reset state and covariance to initial values
    void reset();

    // Run one predict + update step.
    // Returns filtered angle. Call rate() for filtered rate.
    double step(double meas_angle, double meas_rate, double dt);

    double angle() const { return x_[0]; }
    double rate()  const { return x_[1]; }

private:
    std::array<double, 2> x_ = {0.0, 0.0};
    std::array<std::array<double,2>, 2> P_ = {{{1.0, 0.0}, {0.0, 1.0}}};
};