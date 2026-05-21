#pragma once
#include <deque>
#include <Eigen/Core>
#include "dg_kilo/params.hpp"

namespace dg_kilo {

struct DegradationResult {
  bool   degraded       = false;
  int    degraded_axes  = 0;
  double min_eigenvalue = 0.0;
  double threshold      = 0.0;
};

// Hessian eigenvalue analysis + 30-frame rolling threshold (Alg 1, Eq 13-15).
// An eigenvector-consistency check (≤10°) confirms persistent degradation.
class DegradationDetector
{
public:
  explicit DegradationDetector(const DgKiloParams & params);

  // Analyse the 6×6 Gauss-Newton Hessian from scan-to-map.
  // Updates internal rolling stats and returns diagnosis.
  DegradationResult detect(const Eigen::Matrix<double,6,6> & H);

private:
  DgKiloParams params_;

  // Rolling window of minimum positional eigenvalues (upper-left 3×3 of H)
  std::deque<double> window_;

  // Last eigenvector of the minimum eigenvalue axis
  Eigen::Vector3d last_min_eigvec_ = Eigen::Vector3d::Zero();
  bool            have_last_       = false;
};

} // namespace dg_kilo
