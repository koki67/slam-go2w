#pragma once
#include <Eigen/Core>
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/ikd_tree_wrapper.hpp"

namespace dg_kilo {

// Iterated Extended Kalman Filter scan-to-map update (Gauss-Newton iteration).
// Implements the scan-to-map residuals d_e (edge), d_p (planar), d_c (combined)
// as in Eq 10-11 of DG-KILO, then closes the IESKF loop as in Eq 4-5.
class Iekf
{
public:
  static constexpr int kMaxIter = 30;
  static constexpr double kConvergeThr = 1e-4;

  explicit Iekf(const DgKiloParams & params);

  // Perform scan-to-map update.  Updates x and P; returns GN summary.
  GnResult scanToMap(
    State & x,
    Eigen::MatrixXd & P,
    const Features & feats,
    IkdTreeWrapper & map);

private:
  GnResult buildHessian(
    const State & x,
    const Features & feats,
    IkdTreeWrapper & map) const;

  DgKiloParams params_;
};

} // namespace dg_kilo
