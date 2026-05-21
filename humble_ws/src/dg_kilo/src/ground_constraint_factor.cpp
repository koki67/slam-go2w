#include "dg_kilo/ground_constraint_factor.hpp"
#include <cmath>

namespace dg_kilo {

GroundConstraintFactor::GroundConstraintFactor(
  gtsam::Key key,
  const std::array<Eigen::Vector3d, 4> & foot_pos_body,
  const Eigen::Vector3d & plane_normal_world,
  double plane_d,
  const Eigen::Vector3d & n_prev,
  uint8_t stance_mask,
  const gtsam::SharedNoiseModel & model)
: gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
  foot_pos_body_(foot_pos_body),
  plane_normal_world_(plane_normal_world),
  plane_d_(plane_d),
  n_prev_(n_prev),
  stance_mask_(stance_mask)
{}

gtsam::Vector GroundConstraintFactor::evaluateError(
  const gtsam::Pose3 & pose,
  boost::optional<gtsam::Matrix &> H) const
{
  // Count stance legs
  int n_stance = __builtin_popcount(stance_mask_);
  int dim = n_stance + 1;  // height residuals + 1 coplanar

  gtsam::Vector err = gtsam::Vector::Zero(dim);
  gtsam::Matrix Jac = gtsam::Matrix::Zero(dim, 6);

  // Transform foot positions to world frame
  int row = 0;
  for (int i = 0; i < 4; ++i) {
    if (!(stance_mask_ & (1 << i))) continue;

    // p_world = R * p_body + t
    gtsam::Point3 p_body_gtsam(
      foot_pos_body_[i].x(), foot_pos_body_[i].y(), foot_pos_body_[i].z());

    gtsam::Matrix36 J_transform;
    gtsam::Point3 p_world_gtsam = pose.transformFrom(p_body_gtsam, J_transform);

    Eigen::Vector3d p_world(p_world_gtsam.x(), p_world_gtsam.y(), p_world_gtsam.z());

    // Height residual r_d = n^T * p + d (Eq 24-25)
    err(row) = plane_normal_world_.dot(p_world) + plane_d_;

    // Jacobian: ∂r_d/∂ξ = n^T · J_transform
    if (H) {
      Jac.row(row) = plane_normal_world_.transpose() * J_transform.block<3,6>(0,0);
    }
    ++row;
  }

  // Coplanar residual r_g = sin(angle between n_curr and n_prev) (Eq 26-29)
  // Approximate n_curr as plane_normal_world_ (we don't re-estimate inside factor)
  double cos_ang = std::min(1.0, std::abs(plane_normal_world_.dot(n_prev_)));
  err(row) = std::sqrt(std::max(0.0, 1.0 - cos_ang * cos_ang));
  // Jacobian of coplanar w.r.t. pose is approximately zero (plane is pre-estimated)
  // so leave row `row` of Jac as zeros.

  if (H) *H = Jac;
  return err;
}

} // namespace dg_kilo
