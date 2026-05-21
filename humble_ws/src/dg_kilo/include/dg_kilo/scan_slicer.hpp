#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/params.hpp"

namespace dg_kilo {

// Adaptive scan slicing driven by leg-odometry velocity (Eq 2-3).
// Slices a full PandarXT-16 sweep into N segments based on estimated
// linear and angular speed, then stitches each segment using the relative
// transform from the leg ESKF.
class ScanSlicer
{
public:
  explicit ScanSlicer(const DgKiloParams & params);

  // Compute the number of slices for this sweep given current body velocity.
  int numSlices(const Eigen::Vector3d & v, const Eigen::Vector3d & omega) const;

  // Slice the full cloud into segments; each segment is returned in the
  // frame of its midpoint timestamp.
  std::vector<CloudPtr> slice(const CloudPtr & cloud, int n_slices) const;

  // Stitch sliced segments back together using relative transforms.
  // transforms[i] = T_{world ← slice_i} (from leg ESKF interpolation)
  CloudPtr stitch(
    const std::vector<CloudPtr> & slices,
    const std::vector<Eigen::Isometry3d> & transforms) const;

private:
  DgKiloParams params_;
};

} // namespace dg_kilo
