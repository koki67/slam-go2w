#include "dg_kilo/loop_closure.hpp"
#include <pcl/registration/gicp.h>
#include <cmath>

namespace dg_kilo {

LoopClosure::LoopClosure(const DgKiloParams & params)
: params_(params) {}

void LoopClosure::addKeyframe(
  size_t idx,
  const Eigen::Isometry3d & T_world_body,
  const CloudPtr & planar_submap)
{
  Keyframe kf;
  kf.idx           = idx;
  kf.T_world_body  = T_world_body;
  kf.planar        = planar_submap;
  kf.stamp         = static_cast<double>(idx) * 0.1;  // placeholder: 10 Hz keyframe rate
  keyframes_.push_back(kf);
}

std::optional<LoopCandidate> LoopClosure::detect(size_t query_idx) const
{
  if (query_idx >= keyframes_.size()) return std::nullopt;
  const Keyframe & qkf = keyframes_[query_idx];

  LoopCandidate best;
  best.fitness_score = std::numeric_limits<double>::max();
  bool found = false;

  for (size_t i = 0; i + params_.loop_history_size < query_idx; ++i) {
    const Keyframe & ckf = keyframes_[i];

    // Time gap guard
    double dt = qkf.stamp - ckf.stamp;
    if (dt < params_.loop_time_gap) continue;

    // Radius guard
    Eigen::Vector3d dp = qkf.T_world_body.translation() - ckf.T_world_body.translation();
    if (dp.norm() > params_.loop_search_radius) continue;

    // Submap-to-submap GICP on planar features
    if (!qkf.planar || qkf.planar->empty() || !ckf.planar || ckf.planar->empty()) continue;

    pcl::GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI> gicp;
    gicp.setInputSource(qkf.planar);
    gicp.setInputTarget(ckf.planar);
    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setMaximumIterations(50);

    Cloud aligned;
    gicp.align(aligned);

    if (!gicp.hasConverged()) continue;
    double score = gicp.getFitnessScore();
    if (score > params_.loop_icp_fit_score) continue;
    if (score >= best.fitness_score) continue;

    best.fitness_score = score;
    best.query_idx     = query_idx;
    best.match_idx     = i;

    // Build relative transform from GICP result
    Eigen::Matrix4f T4 = gicp.getFinalTransformation();
    best.T_match_query = Eigen::Isometry3d(T4.cast<double>());
    found = true;
  }

  if (!found) return std::nullopt;
  return best;
}

} // namespace dg_kilo
