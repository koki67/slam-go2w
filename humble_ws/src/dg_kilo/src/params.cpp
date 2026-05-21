#include "dg_kilo/params.hpp"
#include <rclcpp/rclcpp.hpp>

namespace dg_kilo {

void DgKiloParams::loadFromNode(rclcpp::Node & node)
{
  auto p = [&](const std::string & name, auto & val) {
    node.declare_parameter(name, val);
    node.get_parameter(name, val);
  };

  p("pointcloud_topic",  pointcloud_topic);
  p("imu_topic",         imu_topic);
  p("lowstate_topic",    lowstate_topic);
  p("map_frame",         map_frame);
  p("odom_frame",        odom_frame);
  p("base_frame",        base_frame);
  p("lidar_frame",       lidar_frame);
  p("imu_frame",         imu_frame);

  p("lidar_n_scan",      lidar_n_scan);
  p("lidar_rate_hz",     lidar_rate_hz);
  p("lidar_min_range",   lidar_min_range);
  p("lidar_max_range",   lidar_max_range);
  p("slice_omega_max",   slice_omega_max);
  p("slice_v_max",       slice_v_max);
  p("slice_min_segments",slice_min_segments);
  p("slice_max_segments",slice_max_segments);

  p("imu_acc_noise",  imu_acc_noise);
  p("imu_gyr_noise",  imu_gyr_noise);
  p("imu_acc_bias",   imu_acc_bias);
  p("imu_gyr_bias",   imu_gyr_bias);
  p("gravity",        gravity);

  p("edge_threshold",   edge_threshold);
  p("planar_threshold", planar_threshold);
  p("downsample_leaf",  downsample_leaf);

  p("intensity_gradient_min",       intensity_gradient_min);
  p("intensity_gradient_max",       intensity_gradient_max);
  p("intensity_only_when_degraded", intensity_only_when_degraded);

  p("degradation_window",              degradation_window);
  p("degradation_alpha",               degradation_alpha);
  p("degradation_dir_consistency_deg", degradation_dir_consistency_deg);

  p("wheel_radius",              wheel_radius);
  p("foot_force_threshold",      foot_force_threshold);
  p("leg_eskf_position_noise",   leg_eskf_position_noise);
  p("leg_eskf_velocity_noise",   leg_eskf_velocity_noise);
  p("leg_eskf_foot_noise",       leg_eskf_foot_noise);
  p("leg_eskf_wheel_slip_noise", leg_eskf_wheel_slip_noise);

  p("ground_search_radius",    ground_search_radius);
  p("ground_min_neighbors",    ground_min_neighbors);
  p("ground_max_angle_deg",    ground_max_angle_deg);
  p("height_residual_sigma",   height_residual_sigma);
  p("coplanar_residual_sigma", coplanar_residual_sigma);

  p("keyframe_distance",           keyframe_distance);
  p("keyframe_angle_deg",          keyframe_angle_deg);
  p("isam2_relinearize_threshold", isam2_relinearize_threshold);
  p("isam2_relinearize_skip",      isam2_relinearize_skip);

  p("loop_enabled",       loop_enabled);
  p("loop_search_radius", loop_search_radius);
  p("loop_time_gap",      loop_time_gap);
  p("loop_icp_fit_score", loop_icp_fit_score);
  p("loop_history_size",  loop_history_size);
}

} // namespace dg_kilo
