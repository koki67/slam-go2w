#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "wheel_legged_odometry/go2w_kinematics.hpp"

namespace wheel_legged_odometry {

struct MotorMap {
  // Order is FL, FR, RL, RR. Each row is hip, thigh, calf, wheel.
  std::array<std::array<int, 4>, 4> index = {{
    {{3, 4, 5, 13}},
    {{0, 1, 2, 12}},
    {{9, 10, 11, 15}},
    {{6, 7, 8, 14}},
  }};
};

struct EstimatorParameters {
  Go2wKinematicParameters kinematics;
  MotorMap motor_map;
  std::array<double, 4> wheel_velocity_sign = {{-1.0, -1.0, -1.0, -1.0}};
  double support_height_sigma = 0.06;
  double residual_sigma = 0.25;
  double min_support_weight = 0.03;
  double imu_yaw_rate_weight = 1.0;
  double solver_damping = 1.0e-6;
  double base_height_filter_alpha = 0.15;
  int robust_iterations = 3;
};

struct LowStateSample {
  std::array<double, 20> q = {};
  std::array<double, 20> dq = {};
  Eigen::Quaterniond imu_quat = Eigen::Quaterniond::Identity();
  Eigen::Vector3d imu_rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct SupportEstimate {
  Eigen::Vector3d contact_point_base = Eigen::Vector3d::Zero();
  Eigen::Vector3d contact_point_odom = Eigen::Vector3d::Zero();
  Eigen::Vector2d residual = Eigen::Vector2d::Zero();
  double height = 0.0;
  double weight = 0.0;
  bool active = false;
};

struct OdometryState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity_base = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_base = Eigen::Vector3d::Zero();
};

struct EstimatorUpdate {
  OdometryState state;
  std::array<LegJoints, 4> joints;
  std::array<LegJointVels, 4> joint_velocities;
  std::array<SupportEstimate, 4> supports;
  bool initialized = false;
};

class OdometryEstimator
{
public:
  explicit OdometryEstimator(const EstimatorParameters & params = {});

  EstimatorUpdate update(const LowStateSample & sample, double dt);

  const OdometryState & state() const { return state_; }
  bool initialized() const { return initialized_; }

  std::array<LegJoints, 4> extractJoints(const LowStateSample & sample) const;
  std::array<LegJointVels, 4> extractJointVelocities(
    const LowStateSample & sample) const;

private:
  void initialize(const LowStateSample & sample);
  Eigen::Quaterniond orientationFromImuYaw(
    const LowStateSample & sample,
    double yaw) const;

  EstimatorParameters params_;
  Go2wKinematics kinematics_;
  OdometryState state_;
  double yaw_ = 0.0;
  bool initialized_ = false;
};

}  // namespace wheel_legged_odometry
