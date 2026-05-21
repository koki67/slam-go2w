#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace dg_kilo {

// Convenience point type (x,y,z + intensity for intensity-feature path)
using PointXYZI = pcl::PointXYZI;
using Cloud     = pcl::PointCloud<PointXYZI>;
using CloudPtr  = Cloud::Ptr;

// IMU measurement bundle
struct ImuData {
  double stamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;
};

// State: R, p, v, ba, bw, g  (FAST-LIO2 state vector convention)
struct State {
  Eigen::Matrix3d R   = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p   = Eigen::Vector3d::Zero();
  Eigen::Vector3d v   = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba  = Eigen::Vector3d::Zero();
  Eigen::Vector3d bw  = Eigen::Vector3d::Zero();
  Eigen::Vector3d g   = Eigen::Vector3d(0, 0, -9.80511);
};

// 4-legged contact state
struct ContactState {
  bool stance[4]             = {false,false,false,false};
  Eigen::Vector3d foot_vel_world[4];  // wheel-contact velocity in world
  float foot_force_est[4]    = {0,0,0,0};
};

// Feature clouds per scan
struct Features {
  CloudPtr edge;
  CloudPtr planar;
  CloudPtr intensity;
  Features()
  : edge(new Cloud), planar(new Cloud), intensity(new Cloud) {}
};

// Scan-to-map Gauss-Newton result
struct GnResult {
  Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
  Eigen::Matrix<double,6,1> b = Eigen::Matrix<double,6,1>::Zero();
  double residual = 0.0;
  int inliers     = 0;
};

} // namespace dg_kilo
