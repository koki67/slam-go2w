#include "dg_kilo/feature_extractor.hpp"
#include <algorithm>
#include <cmath>

namespace dg_kilo {

FeatureExtractor::FeatureExtractor(const DgKiloParams & params)
: params_(params) {}

// LOAM smoothness c_j (Eq 6): sum of squared differences to 5 neighbours each side
double FeatureExtractor::smoothness(const Cloud & ring, int j) const
{
  const int W = 5;
  const int n = static_cast<int>(ring.size());
  if (j < W || j >= n - W) return 0.0;
  Eigen::Vector3f centre(ring[j].x, ring[j].y, ring[j].z);
  Eigen::Vector3f accum = Eigen::Vector3f::Zero();
  for (int k = j - W; k <= j + W; ++k) {
    if (k == j) continue;
    accum += Eigen::Vector3f(ring[k].x, ring[k].y, ring[k].z);
  }
  Eigen::Vector3f diff = (2 * W) * centre - accum;
  return diff.squaredNorm() / (centre.squaredNorm() + 1e-6f);
}

// Intensity gradient G_j (Eq 7-9)
double FeatureExtractor::intensityGradient(const Cloud & ring, int j) const
{
  const int n = static_cast<int>(ring.size());
  if (j < 1 || j >= n - 1) return 0.0;
  double gi = std::abs(ring[j+1].intensity - ring[j-1].intensity);
  // Normalise by range to make it range-invariant
  double range = std::sqrt(ring[j].x*ring[j].x + ring[j].y*ring[j].y + ring[j].z*ring[j].z);
  return gi / (range + 1e-3);
}

Features FeatureExtractor::extract(const CloudPtr & cloud, bool intensity_active) const
{
  Features feats;
  if (!cloud || cloud->empty()) return feats;

  // Assume points are already scan-line organised (PandarXT-16: 16 rings).
  // We split by azimuth-grouped rings here for simplicity.
  const int N = static_cast<int>(cloud->size());
  const int n_scan = params_.lidar_n_scan;
  const int pts_per_ring = N / n_scan;
  if (pts_per_ring < 10) return feats;

  for (int ring = 0; ring < n_scan; ++ring) {
    Cloud r;
    for (int i = ring * pts_per_ring; i < (ring + 1) * pts_per_ring && i < N; ++i) {
      r.push_back((*cloud)[i]);
    }

    // Compute smoothness for each point
    std::vector<double> c(r.size(), 0.0);
    std::vector<double> g(r.size(), 0.0);
    for (int j = 5; j < static_cast<int>(r.size()) - 5; ++j) {
      c[j] = smoothness(r, j);
      if (intensity_active) g[j] = intensityGradient(r, j);
    }

    // Mark picked points to avoid clustering
    std::vector<bool> picked(r.size(), false);

    // Edges: highest smoothness above threshold
    for (int j = 5; j < static_cast<int>(r.size()) - 5; ++j) {
      if (!picked[j] && c[j] > params_.edge_threshold) {
        feats.edge->push_back(r[j]);
        picked[j] = true;
      }
    }

    // Planars: lowest smoothness below threshold
    for (int j = 5; j < static_cast<int>(r.size()) - 5; ++j) {
      if (!picked[j] && c[j] < params_.planar_threshold) {
        feats.planar->push_back(r[j]);
        picked[j] = true;
      }
    }

    // Intensity features (gated by degradation)
    if (intensity_active) {
      for (int j = 5; j < static_cast<int>(r.size()) - 5; ++j) {
        if (!picked[j] &&
            g[j] > params_.intensity_gradient_min &&
            g[j] < params_.intensity_gradient_max) {
          feats.intensity->push_back(r[j]);
        }
      }
    }
  }
  return feats;
}

} // namespace dg_kilo
