// src/balance_controller.cpp
#include "kalman_filter.hpp"

#include <cmath>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

static constexpr double MAX_VEL          = 50.0;  // rad/s — wheel velocity command limit
static constexpr double FALLEN_THRESHOLD = 0.70;  // rad (~40 degrees) — stop and wait for reset

class BalanceController : public rclcpp::Node {
public:
  BalanceController() : Node("balance_controller") {

    // ── PID gains ─────────────────────────────────────────────────────────
    //  Units (velocity-mode):
    //    P : (rad/s) per rad      of angle error
    //    I : (rad/s) per (rad·s)  of accumulated error   <- primary restoring force
    //    D : dimensionless        (rad/s) per (rad/s) of rate
    //
    //  I must exceed ~g/r ≈ 9.81/0.034 = (approx) 288 to overcome gravity.
    p_gain_ = 150.0;
    i_gain_ = 500.0;
    d_gain_ =  15.0;

    // ── Runtime-tunable parameters ────────────────────────────────────────
    declare_parameter("balance_point",      0.0);   // rad — trim offset for CoM
    declare_parameter("angle_noise_stddev", 0.0);   // rad — inject sensor noise
    declare_parameter("rate_noise_stddev",  0.0);   // rad/s
    declare_parameter("vel_noise_stddev",   0.0);   // rad/s — output noise
    declare_parameter("impulse_magnitude",  0.0);   // rad/s — periodic disturbance
    declare_parameter("impulse_period_s",   5.0);   // s

    rng_ = std::mt19937(std::random_device{}());

    // ── Kalman filter setup ───────────────────────────────────────────────
    kf_.Q_angle = 1e-5;
    kf_.Q_rate  = 1e-3;
    kf_.R_angle = 0.01;
    kf_.R_rate  = 0.01;
    kf_.reset();

    // ── ROS subscriptions & publishers ───────────────────────────────────
    imu_sub_   = create_subscription<sensor_msgs::msg::Imu>(
      "/imu_plugin/out", 10,
      std::bind(&BalanceController::imu_callback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BalanceController::joint_callback, this, _1));

    left_cmd_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/left_wheel_controller/commands", 10);
    right_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/right_wheel_controller/commands", 10);

    last_time_         = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_impulse_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    integral_          = 0.0;
    imu_msg_count_     = 0;
    joint_msg_count_   = 0;
    fallen_state_      = false;

    RCLCPP_INFO(get_logger(), "BalanceController ready  P=%.1f  I=%.1f  D=%.1f",
      p_gain_, i_gain_, d_gain_);
  }

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  right_cmd_pub_;

  // ── PID state ─────────────────────────────────────────────────────────────
  double p_gain_, i_gain_, d_gain_;
  double integral_;

  // ── Kalman filter ──────────────────────────────────────────────────────────
  KalmanFilter kf_;

  // ── Timing / bookkeeping ───────────────────────────────────────────────────
  rclcpp::Time last_time_;
  rclcpp::Time last_impulse_time_;
  std::mt19937 rng_;
  int  imu_msg_count_;
  int  joint_msg_count_;
  bool fallen_state_;

  // ── Publish velocity to both wheels ───────────────────────────────────────
  // left = +u, right = −u (mirrored joint axes -> same physical direction)
  void publish_velocity(double left_vel, double right_vel) {
    std_msgs::msg::Float64MultiArray l, r;
    l.data = {left_vel};
    r.data = {right_vel};
    left_cmd_pub_->publish(l);
    right_cmd_pub_->publish(r);
  }

  // ── IMU callback — runs the full estimation + control loop ────────────────
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_msg_count_++;

    // Extract roll angle and roll rate from IMU
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double meas_angle = roll;
    double meas_rate  = msg->angular_velocity.x;

    // Optional noise injection (set via ros2 param set; defaults at 0.0)
    double a_std = get_parameter("angle_noise_stddev").as_double();
    double r_std = get_parameter("rate_noise_stddev").as_double();
    if (a_std > 0.0) meas_angle += std::normal_distribution<double>(0.0, a_std)(rng_);
    if (r_std > 0.0) meas_rate  += std::normal_distribution<double>(0.0, r_std)(rng_);

    // ── dt ────────────────────────────────────────────────────────────────
    // Using IMU header stamp, not this->now(), because Gazebo's sim clock
    // arrives asynchronously and causes spurious dt=0 at startup.
    rclcpp::Time now = msg->header.stamp;
    double dt = (last_time_.nanoseconds() == 0)
                  ? 0.0 : (now - last_time_).seconds();
    if (dt < 0.0 || dt > 1.0) { last_time_ = now; return; }  // sim reset / jump
    if (dt == 0.0)             { last_time_ = now; return; }  // first message
    last_time_ = now;

    // ── Kalman filter ─────────────────────────────────────────────────────
    double filtered_angle = kf_.step(meas_angle, meas_rate, dt);
    double filtered_rate  = kf_.rate();

    // ── Fallen-over guard ─────────────────────────────────────────────────
    bool just_fell = !fallen_state_ && (std::abs(filtered_angle) > FALLEN_THRESHOLD);
    bool recovered =  fallen_state_ && (std::abs(filtered_angle) < FALLEN_THRESHOLD * 0.7);

    if (just_fell) {
      fallen_state_ = true;
      integral_     = 0.0;
      RCLCPP_WARN(get_logger(),
        "[FALLEN] %.2f° > %.2f° — zeroing output. Reset the simulation.",
        filtered_angle * 180.0 / M_PI, FALLEN_THRESHOLD * 180.0 / M_PI);
    }
    if (recovered) {
      fallen_state_ = false;
      RCLCPP_INFO(get_logger(), "[RECOVERED] back inside threshold — resuming.");
    }
    if (fallen_state_) {
      publish_velocity(0.0, 0.0);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[FALLEN] %.1f° — waiting for sim reset", filtered_angle * 180.0 / M_PI);
      return;
    }

    // ── PID ───────────────────────────────────────────────────────────────
    double balance_point = get_parameter("balance_point").as_double();
    double error         = balance_point - filtered_angle;

    // Anti-windup: only accumulate when not saturated
    double u_preview = p_gain_ * error + i_gain_ * integral_ + d_gain_ * (-filtered_rate);
    bool   saturated = std::abs(u_preview) > MAX_VEL;
    if (!saturated) {
      integral_ += error * dt;
      integral_  = std::clamp(integral_, -5.0, 5.0);
    }

    double p_term = p_gain_ * error;
    double i_term = i_gain_ * integral_;
    double d_term = d_gain_ * (-filtered_rate);
    double u      = std::clamp(p_term + i_term + d_term, -MAX_VEL, MAX_VEL);

    // Optional output noise
    double v_std = get_parameter("vel_noise_stddev").as_double();
    if (v_std > 0.0) u += std::normal_distribution<double>(0.0, v_std)(rng_);

    // Optional periodic disturbance impulse (set via ros2 param set)
    double impulse_mag    = get_parameter("impulse_magnitude").as_double();
    double impulse_period = get_parameter("impulse_period_s").as_double();
    double impulse        = 0.0;
    if (impulse_mag > 0.0 &&
        impulse_period > 0.0 &&
        (now - last_impulse_time_).seconds() >= impulse_period) {
      impulse            = impulse_mag;
      last_impulse_time_ = now;
      RCLCPP_INFO(get_logger(), "[IMPULSE] %.2f rad/s", impulse_mag);
    }

    publish_velocity(u + impulse, -(u + impulse));

    RCLCPP_INFO(get_logger(),
      "angle=%6.3f  rate=%6.3f  err=%6.3f  "
      "P=%6.2f  I=%6.2f  D=%6.2f  u=%6.2f%s",
      filtered_angle, filtered_rate, error,
      p_term, i_term, d_term, u,
      saturated ? "  [SAT]" : "");
  }

  // ── Joint state callback, diagnostic only ────────────────────────────────
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_msg_count_++;
    if (joint_msg_count_ > 3 && !fallen_state_) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if ((msg->name[i] == "left_wheel_joint" ||
             msg->name[i] == "right_wheel_joint") &&
             i < msg->velocity.size() &&
             std::abs(msg->velocity[i]) < 1e-6) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[WARN] %s velocity≈0 while controlling — "
            "is the velocity interface active?", msg->name[i].c_str());
        }
      }
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalanceController>());
  rclcpp::shutdown();
  return 0;
}