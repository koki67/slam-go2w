#pragma once

#include <array>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace wheel_legged_odometry {

enum class LegIndex : int {
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
};

struct LegJoints {
  double hip = 0.0;
  double thigh = 0.0;
  double calf = 0.0;
  double wheel = 0.0;
};

struct LegJointVels {
  double hip = 0.0;
  double thigh = 0.0;
  double calf = 0.0;
  double wheel = 0.0;
};

struct LegKinematics {
  Eigen::Vector3d wheel_center_base = Eigen::Vector3d::Zero();
  Eigen::Matrix3d wheel_rotation_base = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 3, 3> center_jacobian =
    Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Vector3d rolling_axis_base = Eigen::Vector3d::UnitY();
};

struct Go2wKinematicParameters {
  double hip_x = 0.1934;
  double hip_y = 0.0465;
  double thigh_y = 0.0955;
  double thigh_length = 0.213;
  double calf_to_wheel = 0.2264;
  double wheel_radius = 0.0513;
};

class Go2wKinematics
{
public:
  static constexpr int kNumLegs = 4;

  explicit Go2wKinematics(const Go2wKinematicParameters & params = {});

  LegKinematics evaluate(int leg, const LegJoints & joints) const;

  Eigen::Vector3d jointVelocityAtWheelCenter(
    int leg,
    const LegJoints & joints,
    const LegJointVels & velocities) const;

  Eigen::Vector3d rollingTangent(
    const LegKinematics & kin,
    const Eigen::Vector3d & ground_normal_base) const;

  Eigen::Vector3d contactPoint(
    const LegKinematics & kin,
    const Eigen::Vector3d & ground_normal_base) const;

  const Go2wKinematicParameters & params() const { return params_; }

private:
  Eigen::Vector3d wheelCenterOnly(int leg, const LegJoints & joints) const;
  Eigen::Matrix3d wheelRotationOnly(int leg, const LegJoints & joints) const;

  Go2wKinematicParameters params_;
};

std::string legPrefix(int leg);

}  // namespace wheel_legged_odometry
