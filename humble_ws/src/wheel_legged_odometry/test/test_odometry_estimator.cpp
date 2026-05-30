#include <gtest/gtest.h>

#include "wheel_legged_odometry/odometry_estimator.hpp"

using wheel_legged_odometry::EstimatorParameters;
using wheel_legged_odometry::LowStateSample;
using wheel_legged_odometry::OdometryEstimator;

namespace {

LowStateSample standingSample()
{
  LowStateSample sample;
  sample.imu_quat = Eigen::Quaterniond::Identity();
  sample.imu_rpy.setZero();
  sample.gyro.setZero();

  // Deploy motor map order:
  // FL=3,4,5,13; FR=0,1,2,12; RL=9,10,11,15; RR=6,7,8,14.
  for (int leg = 0; leg < 4; ++leg) {
    (void)leg;
  }
  sample.q[3] = -0.1;
  sample.q[4] = 0.9;
  sample.q[5] = -1.8;
  sample.q[0] = 0.1;
  sample.q[1] = 0.9;
  sample.q[2] = -1.8;
  sample.q[9] = -0.1;
  sample.q[10] = 0.9;
  sample.q[11] = -1.8;
  sample.q[6] = 0.1;
  sample.q[7] = 0.9;
  sample.q[8] = -1.8;
  return sample;
}

}  // namespace

TEST(OdometryEstimator, ExtractsDeployMotorMap)
{
  OdometryEstimator estimator;
  LowStateSample sample;
  for (int i = 0; i < 20; ++i) {
    sample.q[i] = static_cast<double>(i);
    sample.dq[i] = static_cast<double>(100 + i);
  }

  const auto joints = estimator.extractJoints(sample);
  const auto velocities = estimator.extractJointVelocities(sample);

  EXPECT_DOUBLE_EQ(joints[0].hip, 3.0);
  EXPECT_DOUBLE_EQ(joints[0].wheel, 13.0);
  EXPECT_DOUBLE_EQ(joints[1].hip, 0.0);
  EXPECT_DOUBLE_EQ(joints[2].wheel, 15.0);
  EXPECT_DOUBLE_EQ(joints[3].calf, 8.0);
  EXPECT_DOUBLE_EQ(velocities[0].hip, 103.0);
  EXPECT_DOUBLE_EQ(velocities[3].wheel, 114.0);
}

TEST(OdometryEstimator, InitializesBaseHeightFromFirstLowState)
{
  OdometryEstimator estimator;
  const auto update = estimator.update(standingSample(), 0.002);

  EXPECT_TRUE(update.initialized);
  EXPECT_GT(update.state.position.z(), 0.1);
  EXPECT_NEAR(update.state.position.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(update.state.position.y(), 0.0, 1.0e-9);
}

TEST(OdometryEstimator, WheelRollingCanMoveBaseWithoutLegMotion)
{
  EstimatorParameters params;
  params.wheel_velocity_sign = {{-1.0, -1.0, -1.0, -1.0}};
  OdometryEstimator estimator(params);

  LowStateSample sample = standingSample();
  estimator.update(sample, 0.002);
  sample.dq[13] = 2.0;
  sample.dq[12] = 2.0;
  sample.dq[15] = 2.0;
  sample.dq[14] = 2.0;
  const auto update = estimator.update(sample, 0.1);

  EXPECT_GT(update.state.linear_velocity_base.x(), 0.02);
  EXPECT_GT(update.state.position.x(), 0.0);
}

TEST(OdometryEstimator, HighSwingCandidateIsDownweighted)
{
  OdometryEstimator estimator;
  LowStateSample sample = standingSample();
  estimator.update(sample, 0.002);

  sample.q[4] = 1.57;
  sample.q[5] = 0.0;
  const auto update = estimator.update(sample, 0.02);

  EXPECT_LT(update.supports[0].weight, 0.5);
}
