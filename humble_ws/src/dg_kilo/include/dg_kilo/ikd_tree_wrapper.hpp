#pragma once
#include <vector>
#include <Eigen/Core>
#include "dg_kilo/common_types.hpp"
#include "ikd-Tree/ikd_Tree.h"

namespace dg_kilo {

// Thin wrapper around KD_TREE<PointXYZI> providing the operations
// needed by DG-KILO: map management, NN search, radius search.
class IkdTreeWrapper
{
public:
  IkdTreeWrapper();

  // Add (or update) points in the map.
  void addPoints(const CloudPtr & cloud, bool downsample = false);

  // Remove points inside an axis-aligned box (map sliding window).
  void boxDelete(const Eigen::Vector3d & lo, const Eigen::Vector3d & hi);

  // k-nearest neighbour search.  Returns distances² and neighbours.
  int nearestKSearch(
    const PointXYZI & query,
    int k,
    std::vector<PointXYZI> & pts,
    std::vector<float> & dists);

  // Radius search.
  int radiusSearch(
    const PointXYZI & query,
    float radius,
    std::vector<PointXYZI> & pts);

  size_t size();

private:
  KD_TREE<PointXYZI> tree_;
};

} // namespace dg_kilo
