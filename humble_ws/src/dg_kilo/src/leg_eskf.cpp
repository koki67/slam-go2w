#include "dg_kilo/leg_eskf.hpp"
#include <cmath>

namespace dg_kilo {

LegEskf::LegEskf(const DgKiloParams & params,
                 std::shared_ptr<Go2wKinematics> kin)
: params_(params),
  kin_(kin),
  P_(Eigen::MatrixXd::Identity(kDim, kDim) * 0.01)
{
  x_.g = Eigen::Vector3d(0, 0, -params_.gravity);
}

void LegEskf::predict(
  double dt,
  const Eigen::Vector3d & acc_meas,
  const Eigen::Vector3d & gyr_meas,
  const std::array<LegJoints,4> & /*q*/,
  const std::array<double,4> & /*omega_wheels*/,
  const ContactState & /*contacts*/)
{
  // Nominal state propagation (Eq 21-23) — mirrors ImuProcessor::propagateStep
  Eigen::Vector3d gyr = gyr_meas - x_.bw;
  Eigen::Vector3d acc = acc_meas - x_.ba;

  Eigen::Vector3d angle = gyr * dt;
  double norm = angle.norm();
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  if (norm > 1e-12) {
    Eigen::AngleAxisd aa(norm, angle / norm);
    dR = aa.toRotationMatrix();
  }

  Eigen::Vector3d a_world = x_.R * acc + x_.g;
  x_.p = x_.p + x_.v * dt + 0.5 * a_world * dt * dt;
  x_.v = x_.v + a_world * dt;
  x_.R = x_.R * dR;

  // Error-state transition F (Eq 22, linearised)
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kDim, kDim);
  // δv += R·[acc]× · δR·dt (skew-symmetric of acc in world)
  Eigen::Matrix3d acc_skew;
  acc_skew <<     0, -a_world.z(),  a_world.y(),
             a_world.z(),      0, -a_world.x(),
            -a_world.y(),  a_world.x(),      0;
  F.block<3,3>(3,0) = acc_skew * dt;           // δp/δR (rough approx)
  F.block<3,3>(3,6) = Eigen::Matrix3d::Identity() * dt; // δp/δv

  // Process noise Q
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kDim, kDim);
  double sa = params_.imu_acc_noise;
  double sw = params_.imu_gyr_noise;
  Q.block<3,3>(0,0)   = Eigen::Matrix3d::Identity() * sw * sw * dt;
  Q.block<3,3>(6,6)   = Eigen::Matrix3d::Identity() * sa * sa * dt;
  Q.block<3,3>(9,9)   = Eigen::Matrix3d::Identity() * params_.imu_acc_bias * dt;
  Q.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * params_.imu_gyr_bias * dt;

  P_ = F * P_ * F.transpose() + Q;
}

void LegEskf::updateWheelContact(
  const std::array<LegJoints,4> & q,
  const std::array<double,4> & omega_wheels,
  const ContactState & contacts)
{
  // For each stance leg: residual = v_contact_measured (≈ 0 in no-slip perpendicular axes)
  // Only constrain the two non-rolling axes (longitudinal slip + lateral)
  for (int i = 0; i < 4; ++i) {
    if (!contacts.stance[i]) continue;

    WheelContactVelocity wc = kin_->contactVelocity(i, q[i], x_.v, x_.v /*omega placeholder*/, omega_wheels[i]);

    // Rolling axis in base_link
    Eigen::Vector3d t_roll = wc.t_rolling;

    // Two perpendicular axes
    Eigen::Vector3d t1 = t_roll.cross(Eigen::Vector3d(0,0,1)).normalized();
    if (t1.norm() < 0.1) t1 = t_roll.cross(Eigen::Vector3d(1,0,0)).normalized();
    Eigen::Vector3d t2 = t_roll.cross(t1).normalized();

    // Residual: expected 0 velocity along t1, t2
    Eigen::Vector2d r_wheel;
    r_wheel(0) = t1.dot(wc.v_contact);
    r_wheel(1) = t2.dot(wc.v_contact);

    // Measurement Jacobian H (2×kDim) w.r.t error state δv
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, kDim);
    H.block<1,3>(0,6) = t1.transpose();  // ∂r/∂δv
    H.block<1,3>(1,6) = t2.transpose();

    double noise_sq = params_.leg_eskf_wheel_slip_noise * params_.leg_eskf_wheel_slip_noise;
    Eigen::Matrix2d R_noise = Eigen::Matrix2d::Identity() * noise_sq;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_noise;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    Eigen::VectorXd dx = K * r_wheel;
    boxPlusAndReset(dx);
    P_ = (Eigen::MatrixXd::Identity(kDim, kDim) - K * H) * P_;
  }
}

void LegEskf::updateGround(
  const Eigen::MatrixXd & H_gpc,
  const Eigen::VectorXd & r_gpc,
  const Eigen::MatrixXd & R_gpc)
{
  Eigen::MatrixXd S = H_gpc * P_ * H_gpc.transpose() + R_gpc;
  Eigen::MatrixXd K = P_ * H_gpc.transpose() * S.inverse();
  Eigen::VectorXd dx = K * r_gpc;
  boxPlusAndReset(dx);
  P_ = (Eigen::MatrixXd::Identity(kDim, kDim) - K * H_gpc) * P_;
}

ContactState LegEskf::detectContact(
  const std::array<LegJoints,4> & /*q*/,
  const float motor_torques[16]) const
{
  ContactState cs;
  // Estimate foot force from calf torque (simplified: F ≈ τ_calf / l_calf)
  const double l_calf = kin_->lengths().l_calf;
  for (int i = 0; i < 4; ++i) {
    float tau_calf = motor_torques[i * 3 + 2];
    cs.foot_force_est[i] = std::abs(tau_calf) / static_cast<float>(l_calf);
    cs.stance[i] = (cs.foot_force_est[i] > params_.foot_force_threshold);
  }
  return cs;
}

void LegEskf::boxPlusAndReset(const Eigen::VectorXd & dx)
{
  // Apply error-state update to nominal state
  Eigen::Vector3d dR_vec = dx.segment<3>(0);
  double norm = dR_vec.norm();
  if (norm > 1e-12) {
    Eigen::AngleAxisd aa(norm, dR_vec / norm);
    x_.R = x_.R * aa.toRotationMatrix();
  }
  x_.p  += dx.segment<3>(3);
  x_.v  += dx.segment<3>(6);
  x_.ba += dx.segment<3>(9);
  x_.bw += dx.segment<3>(12);
}

} // namespace dg_kilo
