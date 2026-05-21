#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "dg_kilo/params.hpp"
#include "dg_kilo/ground_constraint_factor.hpp"

namespace dg_kilo {

// GTSAM iSAM2 factor graph backend (Eq 33).
// Accumulates LIO BetweenFactors, leg-odom BetweenFactors, ground factors,
// and loop-closure factors.  Exposes the optimised pose trajectory.
class FactorGraphBackend
{
public:
  explicit FactorGraphBackend(const DgKiloParams & params);

  // Add a new keyframe pose and LIO odometry edge from the previous keyframe.
  // lio_cov: 6×6 covariance (from GN Hessian inverse).
  void addLioFactor(
    const gtsam::Pose3 & pose,
    const gtsam::Pose3 & T_prev_curr,
    const Eigen::Matrix<double,6,6> & lio_cov);

  // Add a leg-odometry edge between the two most recent keyframe poses.
  // leg_cov: 6×6 from leg ESKF P.
  void addLegFactor(
    const gtsam::Pose3 & T_prev_curr,
    const Eigen::Matrix<double,6,6> & leg_cov);

  // Add a ground constraint factor for the current keyframe.
  void addGroundFactor(
    const std::array<Eigen::Vector3d,4> & foot_pos_body,
    const Eigen::Vector3d & plane_normal_world,
    double plane_d,
    const Eigen::Vector3d & n_prev,
    uint8_t stance_mask);

  // Add a loop-closure BetweenFactor.
  void addLoopFactor(
    size_t query_idx,
    size_t match_idx,
    const gtsam::Pose3 & T_match_query);

  // Run iSAM2 update and return the optimised current pose.
  gtsam::Pose3 update();

  // Return the full optimised trajectory.
  std::vector<gtsam::Pose3> trajectory() const;

  size_t size() const { return key_; }

private:
  DgKiloParams params_;
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values               init_vals_;
  size_t                      key_ = 0;
};

} // namespace dg_kilo
