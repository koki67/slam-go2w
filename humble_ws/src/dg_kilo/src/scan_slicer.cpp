#include "dg_kilo/scan_slicer.hpp"
#include <algorithm>
#include <cmath>

namespace dg_kilo {

ScanSlicer::ScanSlicer(const DgKiloParams & params)
: params_(params) {}

int ScanSlicer::numSlices(const Eigen::Vector3d & v, const Eigen::Vector3d & omega) const
{
  // Eq 2-3: adaptive segmentation based on translational and rotational speed.
  double speed  = v.norm();
  double ang    = omega.norm();
  double sigma_v = speed / params_.slice_v_max;
  double sigma_w = ang  / params_.slice_omega_max;
  double sigma   = std::max(sigma_v, sigma_w);  // dominant motion

  int n = static_cast<int>(std::ceil(
    params_.slice_min_segments +
    (params_.slice_max_segments - params_.slice_min_segments) * sigma));
  n = std::clamp(n, params_.slice_min_segments, params_.slice_max_segments);
  return n;
}

std::vector<CloudPtr> ScanSlicer::slice(const CloudPtr & cloud, int n_slices) const
{
  std::vector<CloudPtr> result(n_slices);
  for (auto & c : result) c = std::make_shared<Cloud>();

  if (!cloud || cloud->empty()) return result;

  const int N = static_cast<int>(cloud->size());
  int pts_per_slice = N / n_slices;

  for (int s = 0; s < n_slices; ++s) {
    int start = s * pts_per_slice;
    int end   = (s == n_slices - 1) ? N : (s + 1) * pts_per_slice;
    for (int i = start; i < end; ++i) {
      result[s]->push_back((*cloud)[i]);
    }
  }
  return result;
}

CloudPtr ScanSlicer::stitch(
  const std::vector<CloudPtr> & slices,
  const std::vector<Eigen::Isometry3d> & transforms) const
{
  CloudPtr out = std::make_shared<Cloud>();
  if (slices.empty()) return out;

  // Transform each slice into the frame of the first slice
  const Eigen::Isometry3d T0_inv = transforms[0].inverse();
  for (size_t s = 0; s < slices.size() && s < transforms.size(); ++s) {
    Eigen::Isometry3d T_rel = T0_inv * transforms[s];
    for (const auto & pt : *slices[s]) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      Eigen::Vector3d pout = T_rel * p;
      PointXYZI ppt = pt;
      ppt.x = static_cast<float>(pout.x());
      ppt.y = static_cast<float>(pout.y());
      ppt.z = static_cast<float>(pout.z());
      out->push_back(ppt);
    }
  }
  return out;
}

} // namespace dg_kilo
