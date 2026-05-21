#include "dg_kilo/factor_graph_backend.hpp"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <cmath>

using gtsam::symbol_shorthand::X;

namespace dg_kilo {

namespace {

gtsam::SharedNoiseModel covToNoise(const Eigen::Matrix<double,6,6> & cov)
{
  // Convert 6×6 covariance to GTSAM noise model (Pose3 convention: rot then trans)
  gtsam::Matrix6 cov6;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      cov6(r, c) = cov(r, c);
  return gtsam::noiseModel::Gaussian::Covariance(cov6);
}

gtsam::Pose3 isometry3dToPose3(const Eigen::Isometry3d & T)
{
  gtsam::Rot3 R = gtsam::Rot3(T.rotation());
  gtsam::Point3 t(T.translation().x(), T.translation().y(), T.translation().z());
  return gtsam::Pose3(R, t);
}

} // namespace

FactorGraphBackend::FactorGraphBackend(const DgKiloParams & params)
: params_(params)
{
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = params_.isam2_relinearize_threshold;
  p.relinearizeSkip      = params_.isam2_relinearize_skip;
  isam2_ = gtsam::ISAM2(p);
}

void FactorGraphBackend::addLioFactor(
  const gtsam::Pose3 & pose,
  const gtsam::Pose3 & T_prev_curr,
  const Eigen::Matrix<double,6,6> & lio_cov)
{
  if (key_ == 0) {
    // Prior factor at first keyframe
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), pose, prior_noise));
    init_vals_.insert(X(0), pose);
  } else {
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(key_-1), X(key_), T_prev_curr, covToNoise(lio_cov)));
    init_vals_.insert(X(key_), pose);
  }
  ++key_;
}

void FactorGraphBackend::addLegFactor(
  const gtsam::Pose3 & T_prev_curr,
  const Eigen::Matrix<double,6,6> & leg_cov)
{
  if (key_ < 2) return;
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
    X(key_-2), X(key_-1), T_prev_curr, covToNoise(leg_cov)));
}

void FactorGraphBackend::addGroundFactor(
  const std::array<Eigen::Vector3d,4> & foot_pos_body,
  const Eigen::Vector3d & plane_normal_world,
  double plane_d,
  const Eigen::Vector3d & n_prev,
  uint8_t stance_mask)
{
  if (key_ == 0) return;

  int n_stance = __builtin_popcount(stance_mask);
  if (n_stance == 0) return;

  int dim = n_stance + 1;
  gtsam::SharedNoiseModel noise =
    gtsam::noiseModel::Isotropic::Sigma(dim, 0.02);

  graph_.add(std::make_shared<GroundConstraintFactor>(
    X(key_-1),
    foot_pos_body,
    plane_normal_world,
    plane_d,
    n_prev,
    stance_mask,
    noise));
}

void FactorGraphBackend::addLoopFactor(
  size_t query_idx,
  size_t match_idx,
  const gtsam::Pose3 & T_match_query)
{
  auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << 0.5, 0.5, 0.5, 0.1, 0.1, 0.1).finished());
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
    X(match_idx), X(query_idx), T_match_query, loop_noise));
}

gtsam::Pose3 FactorGraphBackend::update()
{
  isam2_.update(graph_, init_vals_);
  isam2_.update();  // extra iteration for convergence
  graph_.resize(0);
  init_vals_.clear();

  if (key_ == 0) return gtsam::Pose3();
  return isam2_.calculateEstimate<gtsam::Pose3>(X(key_-1));
}

std::vector<gtsam::Pose3> FactorGraphBackend::trajectory() const
{
  std::vector<gtsam::Pose3> traj;
  gtsam::Values result = isam2_.calculateEstimate();
  for (size_t i = 0; i < key_; ++i) {
    if (result.exists(X(i))) {
      traj.push_back(result.at<gtsam::Pose3>(X(i)));
    }
  }
  return traj;
}

} // namespace dg_kilo
