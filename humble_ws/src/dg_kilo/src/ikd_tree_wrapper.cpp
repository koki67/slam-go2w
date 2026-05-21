#include "dg_kilo/ikd_tree_wrapper.hpp"

namespace dg_kilo {

IkdTreeWrapper::IkdTreeWrapper()
: tree_(0.5, 0.6, 0.2)  // (delete_param, balance_param, box_length)
{}

void IkdTreeWrapper::addPoints(const CloudPtr & cloud, bool /*downsample*/)
{
  PointVector pts(cloud->points.begin(), cloud->points.end());
  tree_.Add_Points(pts, true);
}

void IkdTreeWrapper::boxDelete(const Eigen::Vector3d & lo, const Eigen::Vector3d & hi)
{
  BoxPointType box;
  box.vertex_min[0] = static_cast<float>(lo.x());
  box.vertex_min[1] = static_cast<float>(lo.y());
  box.vertex_min[2] = static_cast<float>(lo.z());
  box.vertex_max[0] = static_cast<float>(hi.x());
  box.vertex_max[1] = static_cast<float>(hi.y());
  box.vertex_max[2] = static_cast<float>(hi.z());
  tree_.Delete_Point_Boxes({box});
}

int IkdTreeWrapper::nearestKSearch(
  const PointXYZI & query,
  int k,
  std::vector<PointXYZI> & pts,
  std::vector<float> & dists) const
{
  PointVector result;
  std::vector<float> dist_result;
  tree_.Nearest_Search(
    const_cast<PointXYZI &>(query), k, result, dist_result);
  pts   = std::vector<PointXYZI>(result.begin(), result.end());
  dists = dist_result;
  return static_cast<int>(result.size());
}

int IkdTreeWrapper::radiusSearch(
  const PointXYZI & query,
  float radius,
  std::vector<PointXYZI> & pts) const
{
  PointVector result;
  std::vector<float> dists;
  tree_.Nearest_Search(
    const_cast<PointXYZI &>(query),
    static_cast<int>(tree_.size()),
    result,
    dists,
    radius * radius);
  pts = std::vector<PointXYZI>(result.begin(), result.end());
  return static_cast<int>(result.size());
}

size_t IkdTreeWrapper::size() const
{
  return static_cast<size_t>(tree_.size());
}

} // namespace dg_kilo
