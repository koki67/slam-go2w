#include <gtest/gtest.h>
#include "dg_kilo/go2w_kinematics.hpp"
#include <cmath>

namespace dg_kilo {

class KinematicsTest : public ::testing::Test {
protected:
  Go2wKinematics kin;
};

// At rest pose (all joints = 0), the foot should be directly below the hip
// offset by thigh + calf length in the -z direction.
TEST_F(KinematicsTest, RestPoseFootPosition)
{
  LegJoints q{0.0, 0.0, 0.0};
  for (int leg = 0; leg < 4; ++leg) {
    LegFkResult res = kin.legFk(leg, q);
    double y_sign = Go2wKinematics::kHipYSign[leg];
    Eigen::Vector3d hip = Go2wKinematics::kHipOrigin[leg];

    // Expected foot position: hip + (0, y_sign * d_hip, -(l_thigh + l_calf))
    double exp_x = hip.x();
    double exp_y = hip.y() + y_sign * kin.lengths().d_hip;
    double exp_z = hip.z() - (kin.lengths().l_thigh + kin.lengths().l_calf);

    EXPECT_NEAR(res.foot_pos_base.x(), exp_x, 1e-4) << "Leg " << leg << " X";
    EXPECT_NEAR(res.foot_pos_base.y(), exp_y, 1e-4) << "Leg " << leg << " Y";
    EXPECT_NEAR(res.foot_pos_base.z(), exp_z, 1e-4) << "Leg " << leg << " Z";
  }
}

// Jacobian check: numerical vs. analytical (numerical is our implementation)
TEST_F(KinematicsTest, JacobianNumerical)
{
  LegJoints q{0.1, -0.5, 0.9};
  LegFkResult res = kin.legFk(0, q);

  // Re-compute numerically ourselves and compare
  const double eps = 1e-5;
  for (int col = 0; col < 3; ++col) {
    LegJoints q_plus = q;
    switch(col) {
      case 0: q_plus.hip   += eps; break;
      case 1: q_plus.thigh += eps; break;
      case 2: q_plus.calf  += eps; break;
    }
    LegFkResult r_plus = kin.legFk(0, q_plus);
    Eigen::Vector3d J_col = (r_plus.foot_pos_base - res.foot_pos_base) / eps;
    for (int row = 0; row < 3; ++row) {
      EXPECT_NEAR(res.J(row, col), J_col(row), 1e-3)
        << "J(" << row << "," << col << ")";
    }
  }
}

// FL and FR should be symmetric about the XZ plane at rest
TEST_F(KinematicsTest, FLFRSymmetry)
{
  LegJoints q{0.0, -0.8, 1.4};
  LegFkResult fl = kin.legFk(0, q);
  LegFkResult fr = kin.legFk(1, q);

  EXPECT_NEAR(fl.foot_pos_base.x(),  fr.foot_pos_base.x(), 1e-4);
  EXPECT_NEAR(fl.foot_pos_base.y(), -fr.foot_pos_base.y(), 1e-4);
  EXPECT_NEAR(fl.foot_pos_base.z(),  fr.foot_pos_base.z(), 1e-4);
}

// Rolling-contact velocity: at rest with zero wheel speed, v_contact ≈ v_base
TEST_F(KinematicsTest, ContactVelocityAtRest)
{
  LegJoints q{0.0, -0.8, 1.6};
  Eigen::Vector3d v_base(0.5, 0.0, 0.0);
  Eigen::Vector3d omega_base = Eigen::Vector3d::Zero();
  double omega_wheel = 0.0;

  auto wc = kin.contactVelocity(0, q, v_base, omega_base, omega_wheel);
  // With omega_wheel=0 and pure translation, contact velocity ≈ base velocity
  // (ignoring cross-product contribution from omega_base×p)
  EXPECT_NEAR(wc.v_contact.x(), v_base.x(), 0.1);  // tolerance: omega×r term
}

} // namespace dg_kilo

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
