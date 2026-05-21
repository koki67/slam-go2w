#include "dg_kilo/ground_plane_estimator.hpp"
#include <cmath>
#include <Eigen/Eigenvalues>

namespace dg_kilo {

GroundPlaneEstimator::GroundPlaneEstimator(const DgKiloParams & params)
: params_(params) {}

std::optional<GroundEstimate> GroundPlaneEstimator::estimate(
  const std::array<Eigen::Vector3d, 4> & foot_pos_world,
  const ContactState & contacts,
  IkdTreeWrapper & map) const
{
  // Collect stance foot positions
  std::vector<int> stance_legs;
  for (int i = 0; i < 4; ++i) {
    if (contacts.stance[i]) stance_legs.push_back(i);
  }
  if (static_cast<int>(stance_legs.size()) < 2) return std::nullopt;

  GroundEstimate est;
  est.stance_count = static_cast<int>(stance_legs.size());

  // For each stance foot: 5-NN lookup + PCA plane fit (Eq 24-25)
  std::vector<Eigen::Vector3d> all_pts;
  for (int i : stance_legs) {
    PointXYZI q;
    q.x = static_cast<float>(foot_pos_world[i].x());
    q.y = static_cast<float>(foot_pos_world[i].y());
    q.z = static_cast<float>(foot_pos_world[i].z());
    std::vector<PointXYZI> nn;
    std::vector<float> dists;
    int found = map.nearestKSearch(q, params_.ground_min_neighbors, nn, dists);
    for (int k = 0; k < found; ++k) {
      all_pts.emplace_back(nn[k].x, nn[k].y, nn[k].z);
    }
  }

  if (static_cast<int>(all_pts.size()) < params_.ground_min_neighbors) {
    // Fall back to foot-position-only plane fit
    for (int i : stance_legs) all_pts.push_back(foot_pos_world[i]);
  }

  // PCA plane fit
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & p : all_pts) centroid += p;
  centroid /= all_pts.size();

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto & p : all_pts) {
    Eigen::Vector3d d = p - centroid;
    cov += d * d.transpose();
  }
  cov /= all_pts.size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  // Minimum eigenvalue → plane normal
  Eigen::Vector3d n = eig.eigenvectors().col(0);
  if (n.z() < 0) n = -n;  // ensure upward-facing normal
  est.normal = n;

  double d = -n.dot(centroid);

  // Height residuals r_d (Eq 24-25)
  int n_constraints = static_cast<int>(stance_legs.size()) + (have_prev_ ? 1 : 0);
  est.H_gpc = Eigen::MatrixXd::Zero(n_constraints, 15);  // 15 = LegESKF kDim
  est.r_gpc = Eigen::VectorXd::Zero(n_constraints);
  est.R_gpc = Eigen::MatrixXd::Identity(n_constraints, n_constraints)
              * params_.height_residual_sigma * params_.height_residual_sigma;

  for (int k = 0; k < static_cast<int>(stance_legs.size()); ++k) {
    int i = stance_legs[k];
    est.r_gpc(k) = heightResidual(foot_pos_world[i], n, d);
    // Jacobian: ∂r_d/∂δp = n^T (foot height residual differentiates through p)
    est.H_gpc.block<1,3>(k, 3) = n.transpose();
  }

  // Coplanar residual r_g (Eq 26-29)
  if (have_prev_) {
    auto r_g = coplanarResidual(n, prev_normal_);
    if (r_g) {
      int row = static_cast<int>(stance_legs.size());
      est.r_gpc(row) = *r_g;
      est.H_gpc.block<1,3>(row, 0) = prev_normal_.cross(n).transpose();  // approx Jacobian
      est.R_gpc(row, row) = params_.coplanar_residual_sigma * params_.coplanar_residual_sigma;
    }
  }

  prev_normal_ = n;
  have_prev_   = true;
  return est;
}

double GroundPlaneEstimator::heightResidual(
  const Eigen::Vector3d & foot_pos_world,
  const Eigen::Vector3d & plane_normal,
  double plane_d) const
{
  // r_d = n^T · p_foot + d  (signed distance to plane)
  return plane_normal.dot(foot_pos_world) + plane_d;
}

std::optional<double> GroundPlaneEstimator::coplanarResidual(
  const Eigen::Vector3d & n_curr,
  const Eigen::Vector3d & n_prev) const
{
  // r_g = angle between n_curr and n_prev (Eq 26-29 simplified as sin of angle)
  double cos_ang = n_curr.dot(n_prev);
  cos_ang = std::min(1.0, std::max(-1.0, cos_ang));
  double angle_deg = std::acos(cos_ang) * 180.0 / M_PI;
  if (angle_deg > params_.ground_max_angle_deg) return std::nullopt;
  return std::asin(std::sqrt(1.0 - cos_ang * cos_ang));  // sin(angle) as residual
}

} // namespace dg_kilo
