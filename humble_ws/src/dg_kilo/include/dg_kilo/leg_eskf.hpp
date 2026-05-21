#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/params.hpp"
#include "dg_kilo/go2w_kinematics.hpp"

namespace dg_kilo {

// Leg ESKF error-state (Eq 16/19): [δR(3), δp(3), δv(3), δba(3), δbω(3)].
// Propagation: Eq 21-23.
// Rolling-wheel contact measurement: constrains the two non-rolling axes.
// Ground update: Eq 32 (called externally by GroundPlaneEstimator).
class LegEskf
{
public:
  static constexpr int kDim = 15;  // error-state dimension

  explicit LegEskf(const DgKiloParams & params,
                   std::shared_ptr<Go2wKinematics> kin);

  // Propagate forward using IMU + leg kinematics.
  // dt: time step in seconds.
  // q: joint angles for all 4 legs.
  // omega_wheels: wheel angular velocities for all 4 legs.
  void predict(
    double dt,
    const Eigen::Vector3d & acc_meas,
    const Eigen::Vector3d & gyr_meas,
    const std::array<LegJoints,4> & q,
    const std::array<double,4> & omega_wheels,
    const ContactState & contacts);

  // Rolling-contact update for stance legs (Eq 32 applied to wheel residual).
  void updateWheelContact(
    const std::array<LegJoints,4> & q,
    const std::array<double,4> & omega_wheels,
    const ContactState & contacts);

  // Ground-plane update hook (called from GroundPlaneEstimator, Eq 32).
  void updateGround(
    const Eigen::MatrixXd & H_gpc,
    const Eigen::VectorXd & r_gpc,
    const Eigen::MatrixXd & R_gpc);

  // Retrieve current nominal state and covariance.
  const State & state() const { return x_; }
  const Eigen::MatrixXd & cov() const { return P_; }

  // Fill ContactState from motor forces and current FK.
  ContactState detectContact(
    const std::array<LegJoints,4> & q,
    const float motor_torques[16]) const;

private:
  void boxPlusAndReset(const Eigen::VectorXd & dx);

  DgKiloParams params_;
  std::shared_ptr<Go2wKinematics> kin_;

  State           x_;
  Eigen::MatrixXd P_;   // kDim × kDim
};

} // namespace dg_kilo
