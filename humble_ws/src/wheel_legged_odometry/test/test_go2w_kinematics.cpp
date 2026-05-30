#include <gtest/gtest.h>

#include "wheel_legged_odometry/go2w_kinematics.hpp"

using wheel_legged_odometry::Go2wKinematics;
using wheel_legged_odometry::LegIndex;
using wheel_legged_odometry::LegJoints;

TEST(Go2wKinematics, StandingGeometryIsSymmetric)
{
  Go2wKinematics kin;
  LegJoints q;
  q.thigh = 0.9;
  q.calf = -1.8;

  const auto fl = kin.evaluate(static_cast<int>(LegIndex::FL), q);
  const auto fr = kin.evaluate(static_cast<int>(LegIndex::FR), q);
  const auto rl = kin.evaluate(static_cast<int>(LegIndex::RL), q);
  const auto rr = kin.evaluate(static_cast<int>(LegIndex::RR), q);

  EXPECT_GT(fl.wheel_center_base.x(), 0.0);
  EXPECT_GT(fr.wheel_center_base.x(), 0.0);
  EXPECT_LT(rl.wheel_center_base.x(), 0.0);
  EXPECT_LT(rr.wheel_center_base.x(), 0.0);
  EXPECT_GT(fl.wheel_center_base.y(), 0.0);
  EXPECT_LT(fr.wheel_center_base.y(), 0.0);
  EXPECT_GT(rl.wheel_center_base.y(), 0.0);
  EXPECT_LT(rr.wheel_center_base.y(), 0.0);
  EXPECT_NEAR(fl.wheel_center_base.z(), fr.wheel_center_base.z(), 1.0e-9);
}

TEST(Go2wKinematics, RollingTangentFollowsBodyForwardAxisAtNeutralPose)
{
  Go2wKinematics kin;
  LegJoints q;
  q.thigh = 0.9;
  q.calf = -1.8;

  const auto fl = kin.evaluate(static_cast<int>(LegIndex::FL), q);
  const auto tangent = kin.rollingTangent(fl, Eigen::Vector3d::UnitZ());

  EXPECT_GT(tangent.x(), 0.5);
  EXPECT_NEAR(tangent.y(), 0.0, 1.0e-6);
}
