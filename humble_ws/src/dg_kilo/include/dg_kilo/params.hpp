#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rclcpp {
class Node;
}

namespace dg_kilo {

struct DgKiloParams {
  // --- Topics / frames ---
  std::string pointcloud_topic  = "/points_raw";
  std::string imu_topic         = "/go2w/imu";
  std::string lowstate_topic    = "lowstate";
  std::string map_frame         = "map";
  std::string odom_frame        = "odom";
  std::string base_frame        = "base_link";
  std::string lidar_frame       = "hesai_lidar";
  std::string imu_frame         = "imu_link";

  // --- Extrinsics (base_link → sensor) ---
  Eigen::Vector3d    extrinsic_lidar_translation = Eigen::Vector3d(0.1634, 0.0, 0.116);
  Eigen::Quaterniond extrinsic_lidar_quaternion  = Eigen::Quaterniond(0.7071068,0,0,0.7071068);
  Eigen::Vector3d    extrinsic_imu_translation   = Eigen::Vector3d::Zero();
  Eigen::Quaterniond extrinsic_imu_quaternion    = Eigen::Quaterniond(0.7071068,0,0,0.7071068);

  // --- LiDAR / slicing ---
  int    lidar_n_scan        = 16;
  double lidar_rate_hz       = 10.0;
  double lidar_fov_deg       = 360.0;
  double lidar_min_range     = 0.5;
  double lidar_max_range     = 100.0;
  double slice_omega_max     = 2.0;
  double slice_v_max         = 1.5;
  int    slice_min_segments  = 1;
  int    slice_max_segments  = 4;

  // --- IMU noises ---
  double imu_acc_noise   = 0.05;
  double imu_gyr_noise   = 0.02;
  double imu_acc_bias    = 1e-5;
  double imu_gyr_bias    = 1e-5;
  double gravity         = 9.80511;

  // --- Feature extraction ---
  double edge_threshold    = 1.0;
  double planar_threshold  = 0.1;
  double downsample_leaf   = 0.2;

  // --- Intensity features ---
  double intensity_gradient_min         = 5.0;
  double intensity_gradient_max         = 200.0;
  bool   intensity_only_when_degraded   = true;

  // --- Degradation ---
  int    degradation_window              = 30;
  double degradation_alpha               = 2.5;
  double degradation_dir_consistency_deg = 10.0;

  // --- Wheels / legs ---
  double wheel_radius = 0.0513;
  std::vector<std::string> foot_links   = {"FL_foot","FR_foot","RL_foot","RR_foot"};
  std::vector<std::string> wheel_joints = {"FL_foot_joint","FR_foot_joint","RL_foot_joint","RR_foot_joint"};
  std::vector<int> motor_index_legs    = {0,1,2,3,4,5,6,7,8,9,10,11};
  std::vector<int> motor_index_wheels  = {12,13,14,15};

  // --- Leg ESKF ---
  double foot_force_threshold          = 80.0;
  double leg_eskf_position_noise       = 0.01;
  double leg_eskf_velocity_noise       = 0.1;
  double leg_eskf_foot_noise           = 0.05;
  double leg_eskf_wheel_slip_noise     = 0.5;

  // --- Ground ---
  double ground_search_radius    = 0.25;
  int    ground_min_neighbors    = 5;
  double ground_max_angle_deg    = 8.0;
  double height_residual_sigma   = 0.02;
  double coplanar_residual_sigma = 0.02;

  // --- Factor graph ---
  double keyframe_distance       = 0.3;
  double keyframe_angle_deg      = 10.0;
  double isam2_relinearize_threshold = 0.1;
  int    isam2_relinearize_skip  = 1;

  // --- Loop closure ---
  bool   loop_enabled        = true;
  double loop_search_radius  = 15.0;
  double loop_time_gap       = 30.0;
  double loop_icp_fit_score  = 0.3;
  int    loop_history_size   = 25;

  // Load all parameters from a rclcpp::Node
  void loadFromNode(rclcpp::Node & node);
};

} // namespace dg_kilo
