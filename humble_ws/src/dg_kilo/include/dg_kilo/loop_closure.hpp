#pragma once
#include <optional>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/params.hpp"

namespace dg_kilo {

struct LoopCandidate {
  size_t query_idx;
  size_t match_idx;
  Eigen::Isometry3d T_match_query;  // relative transform (GICP result)
  double fitness_score;
};

// Radius-search loop closure with submap-to-submap GICP (Alg 1 step).
// On success, the caller pushes a BetweenFactor<Pose3> to the factor graph.
class LoopClosure
{
public:
  explicit LoopClosure(const DgKiloParams & params);

  // Register a new keyframe (pose in world, planar-feature submap).
  void addKeyframe(
    size_t idx,
    const Eigen::Isometry3d & T_world_body,
    const CloudPtr & planar_submap);

  // Attempt loop detection for the latest keyframe.
  // Returns the best candidate if GICP converges below fit_score threshold.
  std::optional<LoopCandidate> detect(size_t query_idx) const;

private:
  struct Keyframe {
    size_t idx;
    Eigen::Isometry3d T_world_body;
    CloudPtr planar;
    double stamp;
  };

  DgKiloParams params_;
  std::vector<Keyframe> keyframes_;
};

} // namespace dg_kilo
