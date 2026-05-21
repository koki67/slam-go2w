#include <gtest/gtest.h>
#include "dg_kilo/leg_eskf.hpp"
#include "dg_kilo/go2w_kinematics.hpp"
#include <cmath>
#include <Eigen/Eigenvalues>

namespace dg_kilo {

class LegEskfTest : public ::testing::Test {
protected:
  DgKiloParams params;
  std::shared_ptr<Go2wKinematics> kin;
  std::unique_ptr<LegEskf> eskf;

  LegEskfTest() {
    params.gravity                = 9.80511;
    params.imu_acc_noise          = 0.05;
    params.imu_gyr_noise          = 0.02;
    params.imu_acc_bias           = 1e-5;
    params.imu_gyr_bias           = 1e-5;
    params.foot_force_threshold   = 80.0;
    params.leg_eskf_wheel_slip_noise = 0.5;
    kin  = std::make_shared<Go2wKinematics>();
    eskf = std::make_unique<LegEskf>(params, kin);
  }

  std::array<LegJoints, 4> restPoseJoints() const {
    LegJoints q{0.0, -0.8, 1.6};
    return {q, q, q, q};
  }
};

// Static stance: after predict + contact update with zero IMU input
// (gravity-compensated), velocity should remain near zero.
TEST_F(LegEskfTest, StaticStanceLowVelocityDrift)
{
  std::array<LegJoints, 4> q = restPoseJoints();
  std::array<double, 4> omega_w = {0.0, 0.0, 0.0, 0.0};

  ContactState cs;
  for (int i = 0; i < 4; ++i) cs.stance[i] = true;

  // Simulate 1 second at 500 Hz
  Eigen::Vector3d acc(0, 0, params.gravity);  // gravity-only (no motion)
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  const double dt = 0.002;

  for (int step = 0; step < 500; ++step) {
    eskf->predict(dt, acc, gyr, q, omega_w, cs);
    eskf->updateWheelContact(q, omega_w, cs);
  }

  const State & s = eskf->state();
  EXPECT_NEAR(s.v.norm(), 0.0, 0.2)   << "Velocity should stay near zero";
  EXPECT_NEAR(s.p.norm(), 0.0, 0.3)   << "Position should stay near zero";
}

// Covariance should not diverge or go negative-definite during predict
TEST_F(LegEskfTest, CovariancePositiveDefinite)
{
  std::array<LegJoints, 4> q = restPoseJoints();
  std::array<double, 4> omega_w = {0.0, 0.0, 0.0, 0.0};
  ContactState cs;
  for (int i = 0; i < 4; ++i) cs.stance[i] = true;

  Eigen::Vector3d acc(0, 0, params.gravity);
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();

  for (int step = 0; step < 100; ++step) {
    eskf->predict(0.002, acc, gyr, q, omega_w, cs);
  }

  const Eigen::MatrixXd & P = eskf->cov();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P);
  EXPECT_GT(eig.eigenvalues().minCoeff(), -1e-10)
    << "Covariance has negative eigenvalue — likely diverged";
}

// Contact detection: high torque → stance, low torque → swing
TEST_F(LegEskfTest, ContactDetection)
{
  std::array<LegJoints, 4> q = restPoseJoints();
  float torques[16] = {};

  // Set calf torques for FL (motor 2) to simulate stance
  torques[2] = 30.0f;   // FL calf: high torque → stance
  torques[5] = 2.0f;    // FR calf: low torque → swing (depends on threshold)

  ContactState cs = eskf->detectContact(q, torques);

  // FL should be in stance (force_est ≈ 30/0.213 ≈ 141 > threshold 80)
  EXPECT_TRUE(cs.stance[0]) << "FL should be in stance";
  // FR depends on threshold but low torque → likely swing
  float fr_force = std::abs(torques[5]) / static_cast<float>(kin->lengths().l_calf);
  EXPECT_EQ(cs.stance[1], fr_force > params.foot_force_threshold);
}

// Ground update should reduce position uncertainty
TEST_F(LegEskfTest, GroundUpdateReducesUncertainty)
{
  std::array<LegJoints, 4> q = restPoseJoints();
  std::array<double, 4> omega_w = {0.0, 0.0, 0.0, 0.0};
  ContactState cs;
  for (int i = 0; i < 4; ++i) cs.stance[i] = true;

  eskf->predict(0.1,
    Eigen::Vector3d(0,0,params.gravity),
    Eigen::Vector3d::Zero(),
    q, omega_w, cs);

  double p_norm_before = eskf->cov().block<3,3>(3,3).norm();

  // Inject ground update: plane z = 0
  Eigen::MatrixXd H_gpc = Eigen::MatrixXd::Zero(1, LegEskf::kDim);
  H_gpc(0, 5) = 1.0;  // z component of position
  Eigen::VectorXd r_gpc = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd R_gpc = Eigen::MatrixXd::Identity(1,1) * 0.0004;

  eskf->updateGround(H_gpc, r_gpc, R_gpc);

  double p_norm_after = eskf->cov().block<3,3>(3,3).norm();
  EXPECT_LT(p_norm_after, p_norm_before)
    << "Ground update should reduce position covariance";
}

} // namespace dg_kilo

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
