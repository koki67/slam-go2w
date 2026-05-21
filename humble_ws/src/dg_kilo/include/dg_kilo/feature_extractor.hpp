#pragma once
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/params.hpp"

namespace dg_kilo {

// LOAM-style geometric feature extraction (Eq 6) plus
// intensity-gradient features (Eq 7-9, gated by degradation flag).
class FeatureExtractor
{
public:
  explicit FeatureExtractor(const DgKiloParams & params);

  // Extract edge + planar features from an undistorted organised cloud.
  // intensity_active: when true, also populate feats.intensity.
  Features extract(const CloudPtr & cloud, bool intensity_active) const;

private:
  double smoothness(const Cloud & ring, int j) const;
  double intensityGradient(const Cloud & ring, int j) const;

  DgKiloParams params_;
};

} // namespace dg_kilo
