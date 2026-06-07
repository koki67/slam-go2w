#include "wheel_legged_odometry/odometry_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Dense>

namespace wheel_legged_odometry {

namespace {

double clampValue(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double robustWeight(double residual_norm, double sigma)
{
  if (sigma <= 1.0e-9) {
    return 1.0;
  }
  const double x = residual_norm / sigma;
  return 1.0 / (1.0 + x * x);
}

double gaussianWeight(double value, double sigma)
{
  if (sigma <= 1.0e-9) {
    return 1.0;
  }
  const double x = value / sigma;
  return std::exp(-0.5 * x * x);
}

Eigen::Quaterniond quatFromRpy(double roll, double pitch, double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

Eigen::Vector3d rpyFromQuaternion(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::asin(clampValue(-R(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto mid = values.begin() + static_cast<long>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  return *mid;
}

}  // namespace

OdometryEstimator::OdometryEstimator(const EstimatorParameters & params)
: params_(params),
  kinematics_(params.kinematics)
{
}

EstimatorUpdate OdometryEstimator::update(
  const LowStateSample & sample,
  double dt)
{
  if (!initialized_) {
    initialize(sample);
  }
  dt = clampValue(dt, 0.0, 0.1);

  const auto joints = extractJoints(sample);
  const auto velocities = extractJointVelocities(sample);
  const Eigen::Quaterniond orientation =
    orientationFromImuYaw(sample, yaw_).normalized();
  const Eigen::Matrix3d R_odom_base = orientation.toRotationMatrix();
  const Eigen::Vector3d ground_normal_base =
    R_odom_base.transpose() * Eigen::Vector3d::UnitZ();

  std::array<LegKinematics, 4> leg_kin;
  std::array<Eigen::Vector3d, 4> contact_base;
  std::array<Eigen::Vector3d, 4> known_velocity;
  std::array<double, 4> weights = {{1.0, 1.0, 1.0, 1.0}};
  std::array<SupportEstimate, 4> supports;

  for (int leg = 0; leg < 4; ++leg) {
    leg_kin[leg] = kinematics_.evaluate(leg, joints[leg]);
    contact_base[leg] = kinematics_.contactPoint(leg_kin[leg], ground_normal_base);
    const Eigen::Vector3d rolling_tangent =
      kinematics_.rollingTangent(leg_kin[leg], ground_normal_base);
    const Eigen::Vector3d joint_velocity =
      kinematics_.jointVelocityAtWheelCenter(leg, joints[leg], velocities[leg]);
    const Eigen::Vector3d wheel_velocity =
      params_.kinematics.wheel_radius *
      params_.wheel_velocity_sign[leg] *
      velocities[leg].wheel *
      rolling_tangent;

    known_velocity[leg] = joint_velocity + wheel_velocity;
    const Eigen::Vector3d contact_odom =
      state_.position + R_odom_base * contact_base[leg];
    supports[leg].contact_point_base = contact_base[leg];
    supports[leg].contact_point_odom = contact_odom;
    supports[leg].height = contact_odom.z();
    weights[leg] = gaussianWeight(
      supports[leg].height,
      params_.support_height_sigma);
  }

  Eigen::Vector3d twist = Eigen::Vector3d::Zero();
  for (int iter = 0; iter < std::max(1, params_.robust_iterations); ++iter) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity() * params_.solver_damping;
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d & p = contact_base[leg];
      Eigen::Matrix<double, 2, 3> A;
      A << 1.0, 0.0, -p.y(),
           0.0, 1.0,  p.x();
      Eigen::Vector2d rhs;
      rhs.x() = -known_velocity[leg].x() - sample.gyro.y() * p.z();
      rhs.y() = -known_velocity[leg].y() + sample.gyro.x() * p.z();
      const double w = std::sqrt(std::max(0.0, weights[leg]));
      H += (w * A).transpose() * (w * A);
      b += (w * A).transpose() * (w * rhs);
    }

    if (params_.imu_yaw_rate_weight > 0.0) {
      const double w = std::sqrt(params_.imu_yaw_rate_weight);
      Eigen::RowVector3d A;
      A << 0.0, 0.0, 1.0;
      H += (w * A).transpose() * (w * A);
      b += (w * A).transpose() * (w * sample.gyro.z());
    }

    twist = H.ldlt().solve(b);

    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d & p = contact_base[leg];
      Eigen::Vector2d residual;
      residual.x() = twist.x() - twist.z() * p.y() +
        sample.gyro.y() * p.z() + known_velocity[leg].x();
      residual.y() = twist.y() + twist.z() * p.x() -
        sample.gyro.x() * p.z() + known_velocity[leg].y();
      supports[leg].residual = residual;
      weights[leg] = gaussianWeight(
        supports[leg].height,
        params_.support_height_sigma) *
        robustWeight(residual.norm(), params_.residual_sigma);
    }
  }

  yaw_ += twist.z() * dt;
  state_.orientation = orientationFromImuYaw(sample, yaw_).normalized();
  const Eigen::Matrix3d R_new = state_.orientation.toRotationMatrix();
  state_.linear_velocity_base = Eigen::Vector3d(twist.x(), twist.y(), 0.0);
  state_.angular_velocity_base = Eigen::Vector3d(sample.gyro.x(), sample.gyro.y(), twist.z());
  state_.position += R_new * state_.linear_velocity_base * dt;

  double height_sum = 0.0;
  double height_weight_sum = 0.0;
  for (int leg = 0; leg < 4; ++leg) {
    supports[leg].weight = weights[leg];
    supports[leg].active = weights[leg] >= params_.min_support_weight;
    supports[leg].contact_point_odom =
      state_.position + R_new * supports[leg].contact_point_base;
    if (supports[leg].active) {
      const double required_base_z =
        -(R_new * supports[leg].contact_point_base).z();
      height_sum += weights[leg] * required_base_z;
      height_weight_sum += weights[leg];
    }
  }
  if (height_weight_sum > 1.0e-9) {
    const double target_z = height_sum / height_weight_sum;
    state_.position.z() =
      (1.0 - params_.base_height_filter_alpha) * state_.position.z() +
      params_.base_height_filter_alpha * target_z;
  }

  EstimatorUpdate out;
  out.state = state_;
  out.joints = joints;
  out.joint_velocities = velocities;
  out.supports = supports;
  out.initialized = initialized_;
  return out;
}

std::array<LegJoints, 4> OdometryEstimator::extractJoints(
  const LowStateSample & sample) const
{
  std::array<LegJoints, 4> out;
  for (int leg = 0; leg < 4; ++leg) {
    const auto & idx = params_.motor_map.index[leg];
    out[leg].hip = sample.q[idx[0]];
    out[leg].thigh = sample.q[idx[1]];
    out[leg].calf = sample.q[idx[2]];
    out[leg].wheel = sample.q[idx[3]];
  }
  return out;
}

std::array<LegJointVels, 4> OdometryEstimator::extractJointVelocities(
  const LowStateSample & sample) const
{
  std::array<LegJointVels, 4> out;
  for (int leg = 0; leg < 4; ++leg) {
    const auto & idx = params_.motor_map.index[leg];
    out[leg].hip = sample.dq[idx[0]];
    out[leg].thigh = sample.dq[idx[1]];
    out[leg].calf = sample.dq[idx[2]];
    out[leg].wheel = sample.dq[idx[3]];
  }
  return out;
}

void OdometryEstimator::initialize(const LowStateSample & sample)
{
  const auto joints = extractJoints(sample);
  Eigen::Quaterniond orientation = orientationFromImuYaw(sample, 0.0).normalized();
  const Eigen::Matrix3d R_odom_base = orientation.toRotationMatrix();
  const Eigen::Vector3d ground_normal_base =
    R_odom_base.transpose() * Eigen::Vector3d::UnitZ();

  std::vector<double> contact_heights;
  contact_heights.reserve(4);
  for (int leg = 0; leg < 4; ++leg) {
    const LegKinematics kin = kinematics_.evaluate(leg, joints[leg]);
    const Eigen::Vector3d contact_base =
      kinematics_.contactPoint(kin, ground_normal_base);
    contact_heights.push_back((R_odom_base * contact_base).z());
  }

  state_.position = Eigen::Vector3d(0.0, 0.0, -median(contact_heights));
  state_.orientation = orientation;
  state_.linear_velocity_base.setZero();
  state_.angular_velocity_base.setZero();
  yaw_ = 0.0;
  initialized_ = true;
}

Eigen::Quaterniond OdometryEstimator::orientationFromImuYaw(
  const LowStateSample & sample,
  double yaw) const
{
  double roll = sample.imu_rpy.x();
  double pitch = sample.imu_rpy.y();
  if (!std::isfinite(roll) || !std::isfinite(pitch) ||
      (std::abs(roll) < 1.0e-12 && std::abs(pitch) < 1.0e-12 &&
       sample.imu_quat.norm() > 1.0e-6)) {
    const Eigen::Vector3d rpy = rpyFromQuaternion(sample.imu_quat);
    roll = rpy.x();
    pitch = rpy.y();
  }
  return quatFromRpy(roll, pitch, yaw);
}

}  // namespace wheel_legged_odometry
