#include "kalman_filter.hpp"
#include <cmath>

void KalmanFilter::reset() {
    x_ = {0.0, 0.0};
    P_ = {{{1.0, 0.0}, {0.0, 1.0}}};
}

double KalmanFilter::step(double meas_angle, double meas_rate, double dt) {
    // ── Predict ───────────────────────────────────────────────────────────────
    // x' = F x       F = [[1,dt],[0,1]]
    double xa = x_[0] + dt * x_[1];
    double xr = x_[1];

    // P' = F P F^T + Q
    double p00 = P_[0][0], p01 = P_[0][1];
    double p10 = P_[1][0], p11 = P_[1][1];

    double fp00 = p00 + dt * p10,  fp01 = p01 + dt * p11;
    double fp10 = p10,              fp11 = p11;

    double pp00 = fp00 + dt * fp01 + Q_angle;
    double pp01 = fp01;
    double pp10 = fp10 + dt * fp11;
    double pp11 = fp11 + Q_rate;

    // ── Update ────────────────────────────────────────────────────────────────
    // S = P' + R     (H = I₂)
    double s00 = pp00 + R_angle,  s01 = pp01;
    double s10 = pp10,             s11 = pp11 + R_rate;

    // S⁻¹ analytic 2×2 inverse
    double det = s00 * s11 - s01 * s10;
    if (std::abs(det) < 1e-12) det = 1e-12;
    double inv00 =  s11 / det,  inv01 = -s01 / det;
    double inv10 = -s10 / det,  inv11 =  s00 / det;

    // K = P' S⁻¹
    double k00 = pp00 * inv00 + pp01 * inv10;
    double k01 = pp00 * inv01 + pp01 * inv11;
    double k10 = pp10 * inv00 + pp11 * inv10;
    double k11 = pp10 * inv01 + pp11 * inv11;

    // Innovation y = meas - predicted state
    double y0 = meas_angle - xa;
    double y1 = meas_rate  - xr;

    // State update
    x_[0] = xa + k00 * y0 + k01 * y1;
    x_[1] = xr + k10 * y0 + k11 * y1;

    // Covariance update: P = (I - K) P'
    double ik00 = 1.0 - k00,  ik01 = -k01;
    double ik10 = -k10,        ik11 = 1.0 - k11;
    P_[0][0] = ik00 * pp00 + ik01 * pp10;
    P_[0][1] = ik00 * pp01 + ik01 * pp11;
    P_[1][0] = ik10 * pp00 + ik11 * pp10;
    P_[1][1] = ik10 * pp01 + ik11 * pp11;

    return x_[0];
}