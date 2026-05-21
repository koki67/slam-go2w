#pragma once
#include <Eigen/Core>
#include <optional>
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/params.hpp"
#include "dg_kilo/ikd_tree_wrapper.hpp"

namespace dg_kilo {

struct GroundEstimate {
  Eigen::Vector3d normal;    // unit upward normal in world frame
  double          height;    // mean foot height above ground
  Eigen::MatrixXd H_gpc;    // stacked Jacobian (Eq 30)
  Eigen::VectorXd r_gpc;    // stacked residual (Eq 31)
  Eigen::MatrixXd R_gpc;    // noise covariance
  int             stance_count = 0;
};

// Per-stance-foot 5-NN ikd-Tree lookup + PCA plane fit.
// Height residual r_d (Eq 24-25) and coplanar normal-angle residual r_g (Eq 26-29).
class GroundPlaneEstimator
{
public:
  explicit GroundPlaneEstimator(const DgKiloParams & params);

  // Estimate ground plane from stance-foot positions in world frame.
  // Returns nullopt when fewer than 2 stance feet are detected.
  std::optional<GroundEstimate> estimate(
    const std::array<Eigen::Vector3d, 4> & foot_pos_world,
    const ContactState & contacts,
    IkdTreeWrapper & map) const;

  // Height residual for one foot (Eq 24-25).
  double heightResidual(
    const Eigen::Vector3d & foot_pos_world,
    const Eigen::Vector3d & plane_normal,
    double plane_d) const;

  // Coplanar residual between current and previous plane normal (Eq 26-29).
  // Returns nullopt when the previous estimate is unavailable.
  std::optional<double> coplanarResidual(
    const Eigen::Vector3d & n_curr,
    const Eigen::Vector3d & n_prev) const;

private:
  DgKiloParams params_;
  mutable Eigen::Vector3d prev_normal_ = Eigen::Vector3d(0,0,1);
  mutable bool have_prev_ = false;
};

} // namespace dg_kilo
