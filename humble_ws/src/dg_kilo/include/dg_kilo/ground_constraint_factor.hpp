#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>

namespace dg_kilo {

// Custom GTSAM factor encoding the DG-KILO ground constraints (Eq 30-31).
// Error = [r_d(foot 0..3); r_g] (4 height residuals + 1 coplanar residual).
// Jacobians are analytic w.r.t. Pose3 right-perturbation.
class GroundConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  // foot_pos_body: foot positions in base_link frame (from FK)
  // plane_normal_world: estimated ground-plane unit normal in world
  // plane_d: plane equation d (n·x + d = 0)
  // n_prev: previous normal for coplanar residual
  // stance: bitmask of stance feet (bit i = leg i)
  GroundConstraintFactor(
    gtsam::Key key,
    const std::array<Eigen::Vector3d, 4> & foot_pos_body,
    const Eigen::Vector3d & plane_normal_world,
    double plane_d,
    const Eigen::Vector3d & n_prev,
    uint8_t stance_mask,
    const gtsam::SharedNoiseModel & model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override;

private:
  std::array<Eigen::Vector3d, 4> foot_pos_body_;
  Eigen::Vector3d plane_normal_world_;
  double          plane_d_;
  Eigen::Vector3d n_prev_;
  uint8_t         stance_mask_;
};

} // namespace dg_kilo
