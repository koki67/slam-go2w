#include "wheel_legged_odometry/go2w_kinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace wheel_legged_odometry {

namespace {

Eigen::Matrix3d rotX(double q)
{
  Eigen::Matrix3d R;
  const double c = std::cos(q);
  const double s = std::sin(q);
  R << 1.0, 0.0, 0.0,
       0.0, c, -s,
       0.0, s, c;
  return R;
}

Eigen::Matrix3d rotY(double q)
{
  Eigen::Matrix3d R;
  const double c = std::cos(q);
  const double s = std::sin(q);
  R << c, 0.0, s,
       0.0, 1.0, 0.0,
       -s, 0.0, c;
  return R;
}

double sideSign(int leg)
{
  return (leg == static_cast<int>(LegIndex::FL) ||
          leg == static_cast<int>(LegIndex::RL)) ? 1.0 : -1.0;
}

double foreSign(int leg)
{
  return (leg == static_cast<int>(LegIndex::FL) ||
          leg == static_cast<int>(LegIndex::FR)) ? 1.0 : -1.0;
}

}  // namespace

Go2wKinematics::Go2wKinematics(const Go2wKinematicParameters & params)
: params_(params)
{
}

LegKinematics Go2wKinematics::evaluate(int leg, const LegJoints & joints) const
{
  if (leg < 0 || leg >= kNumLegs) {
    throw std::out_of_range("Go2wKinematics leg index out of range");
  }

  LegKinematics out;
  out.wheel_center_base = wheelCenterOnly(leg, joints);
  out.wheel_rotation_base = wheelRotationOnly(leg, joints);
  out.rolling_axis_base = out.wheel_rotation_base.col(1).normalized();

  const double eps = 1.0e-6;
  for (int col = 0; col < 3; ++col) {
    LegJoints perturbed = joints;
    if (col == 0) {
      perturbed.hip += eps;
    } else if (col == 1) {
      perturbed.thigh += eps;
    } else {
      perturbed.calf += eps;
    }
    out.center_jacobian.col(col) =
      (wheelCenterOnly(leg, perturbed) - out.wheel_center_base) / eps;
  }

  return out;
}

Eigen::Vector3d Go2wKinematics::jointVelocityAtWheelCenter(
  int leg,
  const LegJoints & joints,
  const LegJointVels & velocities) const
{
  const LegKinematics kin = evaluate(leg, joints);
  Eigen::Vector3d dq(velocities.hip, velocities.thigh, velocities.calf);
  return kin.center_jacobian * dq;
}

Eigen::Vector3d Go2wKinematics::rollingTangent(
  const LegKinematics & kin,
  const Eigen::Vector3d & ground_normal_base) const
{
  Eigen::Vector3d normal = ground_normal_base;
  if (normal.norm() < 1.0e-9) {
    normal = Eigen::Vector3d::UnitZ();
  } else {
    normal.normalize();
  }

  Eigen::Vector3d tangent = kin.rolling_axis_base.cross(normal);
  if (tangent.norm() < 1.0e-9) {
    tangent = Eigen::Vector3d::UnitX();
  } else {
    tangent.normalize();
  }
  return tangent;
}

Eigen::Vector3d Go2wKinematics::contactPoint(
  const LegKinematics & kin,
  const Eigen::Vector3d & ground_normal_base) const
{
  Eigen::Vector3d normal = ground_normal_base;
  if (normal.norm() < 1.0e-9) {
    normal = Eigen::Vector3d::UnitZ();
  } else {
    normal.normalize();
  }
  return kin.wheel_center_base - params_.wheel_radius * normal;
}

Eigen::Vector3d Go2wKinematics::wheelCenterOnly(
  int leg,
  const LegJoints & joints) const
{
  const double side = sideSign(leg);
  const double fore = foreSign(leg);

  const Eigen::Vector3d hip_origin(
    fore * params_.hip_x,
    side * params_.hip_y,
    0.0);
  const Eigen::Vector3d p_hip_to_thigh(0.0, side * params_.thigh_y, 0.0);

  const Eigen::Matrix3d R_hip = rotX(joints.hip);
  const Eigen::Matrix3d R_thigh = rotY(joints.thigh);
  const Eigen::Matrix3d R_calf = rotY(joints.calf);

  const Eigen::Vector3d p_thigh_to_calf =
    R_hip * R_thigh * Eigen::Vector3d(0.0, 0.0, -params_.thigh_length);
  const Eigen::Vector3d p_calf_to_wheel =
    R_hip * R_thigh * R_calf *
    Eigen::Vector3d(0.0, 0.0, -params_.calf_to_wheel);

  return hip_origin + R_hip * p_hip_to_thigh + p_thigh_to_calf +
    p_calf_to_wheel;
}

Eigen::Matrix3d Go2wKinematics::wheelRotationOnly(
  int /*leg*/,
  const LegJoints & joints) const
{
  return rotX(joints.hip) * rotY(joints.thigh) * rotY(joints.calf) *
    rotY(joints.wheel);
}

std::string legPrefix(int leg)
{
  switch (leg) {
    case static_cast<int>(LegIndex::FL): return "FL";
    case static_cast<int>(LegIndex::FR): return "FR";
    case static_cast<int>(LegIndex::RL): return "RL";
    case static_cast<int>(LegIndex::RR): return "RR";
    default: throw std::out_of_range("legPrefix leg index out of range");
  }
}

}  // namespace wheel_legged_odometry
