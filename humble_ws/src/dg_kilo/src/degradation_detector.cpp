#include "dg_kilo/degradation_detector.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <Eigen/Eigenvalues>

namespace dg_kilo {

DegradationDetector::DegradationDetector(const DgKiloParams & params)
: params_(params) {}

DegradationResult DegradationDetector::detect(const Eigen::Matrix<double,6,6> & H)
{
  DegradationResult res;

  // Extract the positional submatrix (top-left 3×3) for geometric eigenvalue analysis
  Eigen::Matrix3d Hpos = H.block<3,3>(0,0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(Hpos);
  Eigen::Vector3d evals = eig.eigenvalues();   // ascending order
  Eigen::Matrix3d evecs = eig.eigenvectors();

  double min_eval = evals(0);
  res.min_eigenvalue = min_eval;

  // Rolling window update
  window_.push_back(min_eval);
  if (static_cast<int>(window_.size()) > params_.degradation_window) {
    window_.pop_front();
  }

  // Compute μ and σ over window (Eq 13-14)
  double mu  = 0.0;
  double sig = 0.0;
  if (!window_.empty()) {
    mu = std::accumulate(window_.begin(), window_.end(), 0.0) / window_.size();
    for (double v : window_) sig += (v - mu) * (v - mu);
    sig = std::sqrt(sig / window_.size());
  }

  // Threshold τ = μ − α·σ (Eq 15)
  double tau = mu - params_.degradation_alpha * sig;
  res.threshold = tau;

  // Axis degraded when eigenvalue < τ
  int ndeg = 0;
  for (int i = 0; i < 3; ++i) {
    if (evals(i) < tau) ++ndeg;
  }

  // Eigenvector-consistency check: |angle| > consistency_deg confirms degradation
  bool consistent = false;
  if (ndeg > 0 && have_last_) {
    double cos_ang = std::abs(last_min_eigvec_.dot(evecs.col(0)));
    cos_ang = std::min(1.0, cos_ang);
    double deg = std::acos(cos_ang) * 180.0 / M_PI;
    consistent = (deg < params_.degradation_dir_consistency_deg);
  }

  res.degraded_axes = ndeg;
  res.degraded = (ndeg > 0) && (have_last_ ? consistent : false);

  // Update last eigenvector
  last_min_eigvec_ = evecs.col(0);
  have_last_ = true;

  return res;
}

} // namespace dg_kilo
