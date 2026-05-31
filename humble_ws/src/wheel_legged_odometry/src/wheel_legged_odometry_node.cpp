#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unitree_go/msg/low_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wheel_legged_odometry/odometry_estimator.hpp"

namespace wheel_legged_odometry {

namespace {

std::vector<std::string> jointNames()
{
  return {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FL_wheel_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FR_wheel_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RL_wheel_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RR_wheel_joint",
  };
}

std::array<std::array<int, 4>, 4> motorMapFromVector(
  const std::vector<int64_t> & values)
{
  std::array<std::array<int, 4>, 4> out = {{
    {{3, 4, 5, 13}},
    {{0, 1, 2, 12}},
    {{9, 10, 11, 15}},
    {{6, 7, 8, 14}},
  }};
  if (values.size() != 16) {
    return out;
  }
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 4; ++j) {
      out[leg][j] = static_cast<int>(values[leg * 4 + j]);
    }
  }
  return out;
}

std::array<double, 4> wheelSignsFromVector(const std::vector<double> & values)
{
  std::array<double, 4> out = {{-1.0, -1.0, -1.0, -1.0}};
  if (values.size() == 4) {
    for (int i = 0; i < 4; ++i) {
      out[i] = values[i];
    }
  }
  return out;
}

geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  return out;
}

}  // namespace

class WheelLeggedOdometryNode : public rclcpp::Node
{
public:
  WheelLeggedOdometryNode()
  : Node("wheel_legged_odometry")
  {
    loadParameters();
    estimator_ = std::make_unique<OdometryEstimator>(estimator_params_);

    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    sub_lowstate_ = create_subscription<unitree_go::msg::LowState>(
      lowstate_topic_, sensor_qos,
      [this](unitree_go::msg::LowState::SharedPtr msg) {
        lowstateCallback(msg);
      });

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 2);
    pub_joint_state_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 20);
    pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 2);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    path_msg_.header.frame_id = odom_frame_;

    RCLCPP_INFO(
      get_logger(),
      "Wheel-legged odometry ready: lowstate=%s odom=%s joint_states=%s",
      lowstate_topic_.c_str(),
      odom_topic_.c_str(),
      joint_state_topic_.c_str());
  }

private:
  void loadParameters()
  {
    lowstate_topic_ = declare_parameter<std::string>("lowstate_topic", "lowstate");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/wheel_legged_odometry/odom");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/wheel_legged_odometry/path");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/wheel_legged_odometry/joint_states");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/wheel_legged_odometry/support_markers");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    path_max_poses_ = declare_parameter<int>("path_max_poses", 1000);
    path_publish_period_ = 1.0 / std::max(
      1.0, declare_parameter<double>("path_publish_rate_hz", 20.0));
    marker_publish_period_ = 1.0 / std::max(
      1.0, declare_parameter<double>("marker_publish_rate_hz", 20.0));

    estimator_params_.kinematics.hip_x =
      declare_parameter<double>("kinematics.hip_x", 0.1934);
    estimator_params_.kinematics.hip_y =
      declare_parameter<double>("kinematics.hip_y", 0.0465);
    estimator_params_.kinematics.thigh_y =
      declare_parameter<double>("kinematics.thigh_y", 0.0955);
    estimator_params_.kinematics.thigh_length =
      declare_parameter<double>("kinematics.thigh_length", 0.213);
    estimator_params_.kinematics.calf_to_wheel =
      declare_parameter<double>("kinematics.calf_to_wheel", 0.2264);
    estimator_params_.kinematics.wheel_radius =
      declare_parameter<double>("kinematics.wheel_radius", 0.0513);

    const auto motor_indices = declare_parameter<std::vector<int64_t>>(
      "motor_indices",
      std::vector<int64_t>{3, 4, 5, 13, 0, 1, 2, 12, 9, 10, 11, 15, 6, 7, 8, 14});
    estimator_params_.motor_map.index = motorMapFromVector(motor_indices);
    estimator_params_.wheel_velocity_sign = wheelSignsFromVector(
      declare_parameter<std::vector<double>>(
        "wheel_velocity_sign", std::vector<double>{-1.0, -1.0, -1.0, -1.0}));
    estimator_params_.support_height_sigma =
      declare_parameter<double>("support_height_sigma", 0.06);
    estimator_params_.residual_sigma =
      declare_parameter<double>("residual_sigma", 0.25);
    estimator_params_.min_support_weight =
      declare_parameter<double>("min_support_weight", 0.03);
    estimator_params_.imu_yaw_rate_weight =
      declare_parameter<double>("imu_yaw_rate_weight", 1.0);
    estimator_params_.solver_damping =
      declare_parameter<double>("solver_damping", 1.0e-6);
    estimator_params_.base_height_filter_alpha =
      declare_parameter<double>("base_height_filter_alpha", 0.15);
    estimator_params_.robust_iterations =
      declare_parameter<int>("robust_iterations", 3);
  }

  rclcpp::Time tickToRosTime(uint32_t tick_ms)
  {
    if (!have_tick_anchor_) {
      first_tick_ = tick_ms;
      tick_anchor_time_ = get_clock()->now();
      have_tick_anchor_ = true;
      return tick_anchor_time_;
    }

    int64_t delta_ms =
      static_cast<int64_t>(tick_ms) - static_cast<int64_t>(first_tick_);
    if (delta_ms < std::numeric_limits<int32_t>::min()) {
      delta_ms += static_cast<int64_t>(1) << 32;
    }
    if (delta_ms < -1000) {
      first_tick_ = tick_ms;
      tick_anchor_time_ = get_clock()->now();
      return tick_anchor_time_;
    }
    return tick_anchor_time_ + rclcpp::Duration(
      std::chrono::milliseconds(delta_ms));
  }

  void lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg)
  {
    const rclcpp::Time stamp = tickToRosTime(msg->tick);

    // Derive dt from the ROS time delta between consecutive stamps so that
    // the integration step matches the odom stamp progression in both live
    // and replay modes. In replay the bag player controls callback cadence,
    // so wall-clock dt would silently diverge from the stamp progression
    // whenever the player runs faster or slower than real-time.
    double dt = 0.002;
    if (have_last_stamp_) {
      dt = (stamp - last_stamp_).seconds();
      if (dt <= 0.0) {
        dt = 0.002;
      }
    }
    last_stamp_ = stamp;
    have_last_stamp_ = true;

    LowStateSample sample;
    const size_t n_motors = std::min<size_t>(20, msg->motor_state.size());
    for (size_t i = 0; i < n_motors; ++i) {
      sample.q[i] = msg->motor_state[i].q;
      sample.dq[i] = msg->motor_state[i].dq;
    }

    sample.imu_quat = Eigen::Quaterniond(
      msg->imu_state.quaternion[0],
      msg->imu_state.quaternion[1],
      msg->imu_state.quaternion[2],
      msg->imu_state.quaternion[3]);
    if (sample.imu_quat.norm() < 1.0e-6) {
      sample.imu_quat = Eigen::Quaterniond::Identity();
    }
    sample.imu_rpy = Eigen::Vector3d(
      msg->imu_state.rpy[0],
      msg->imu_state.rpy[1],
      msg->imu_state.rpy[2]);
    sample.gyro = Eigen::Vector3d(
      msg->imu_state.gyroscope[0],
      msg->imu_state.gyroscope[1],
      msg->imu_state.gyroscope[2]);

    const EstimatorUpdate update = estimator_->update(sample, dt);
    publishJointState(stamp, update);
    publishOdometry(stamp, update);
    maybePublishPath(stamp, update);
    maybePublishMarkers(stamp, update);
  }

  void publishJointState(
    const rclcpp::Time & stamp,
    const EstimatorUpdate & update)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = jointNames();
    msg.position.reserve(16);
    msg.velocity.reserve(16);
    for (int leg = 0; leg < 4; ++leg) {
      msg.position.push_back(update.joints[leg].hip);
      msg.position.push_back(update.joints[leg].thigh);
      msg.position.push_back(update.joints[leg].calf);
      msg.position.push_back(update.joints[leg].wheel);
      msg.velocity.push_back(update.joint_velocities[leg].hip);
      msg.velocity.push_back(update.joint_velocities[leg].thigh);
      msg.velocity.push_back(update.joint_velocities[leg].calf);
      msg.velocity.push_back(update.joint_velocities[leg].wheel);
    }
    pub_joint_state_->publish(msg);
  }

  void publishOdometry(
    const rclcpp::Time & stamp,
    const EstimatorUpdate & update)
  {
    const auto & state = update.state;

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = state.position.x();
      tf_msg.transform.translation.y = state.position.y();
      tf_msg.transform.translation.z = state.position.z();
      tf_msg.transform.rotation = toMsg(state.orientation);
      tf_broadcaster_->sendTransform(tf_msg);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = state.position.x();
    odom.pose.pose.position.y = state.position.y();
    odom.pose.pose.position.z = state.position.z();
    odom.pose.pose.orientation = toMsg(state.orientation);
    odom.twist.twist.linear.x = state.linear_velocity_base.x();
    odom.twist.twist.linear.y = state.linear_velocity_base.y();
    odom.twist.twist.linear.z = state.linear_velocity_base.z();
    odom.twist.twist.angular.x = state.angular_velocity_base.x();
    odom.twist.twist.angular.y = state.angular_velocity_base.y();
    odom.twist.twist.angular.z = state.angular_velocity_base.z();

    // Set covariance proportional to support quality so downstream fusers
    // (AMCL, robot_localization/EKF, nav2) can correctly balance this
    // odometry against other sources. Zero covariance means infinite
    // confidence, which is misleading for a kinematic estimator that
    // relies on force-free support inference.
    double total_weight = 0.0;
    double mean_residual = 0.0;
    for (const auto & s : update.supports) {
      total_weight += s.weight;
      mean_residual += s.residual.norm();
    }
    mean_residual /= std::max(1, static_cast<int>(update.supports.size()));

    // Scale: confidence increases with more/better support.
    const double pose_var =
      std::max(1.0e-6, 1.0e-3 * (1.0 - total_weight * 0.25) *
                     (1.0 + mean_residual * 5.0));
    const double twist_var =
      std::max(1.0e-4, 1.0e-2 * (1.0 - total_weight * 0.25) *
                     (1.0 + mean_residual * 5.0));
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 6 + i] = pose_var;
      odom.twist.covariance[i * 6 + i] = twist_var;
    }

    pub_odom_->publish(odom);
  }

  void maybePublishPath(
    const rclcpp::Time & stamp,
    const EstimatorUpdate & update)
  {
    if (have_last_path_stamp_ &&
        (stamp - last_path_stamp_).seconds() < path_publish_period_) {
      return;
    }
    last_path_stamp_ = stamp;
    have_last_path_stamp_ = true;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = odom_frame_;
    pose.pose.position.x = update.state.position.x();
    pose.pose.position.y = update.state.position.y();
    pose.pose.position.z = update.state.position.z();
    pose.pose.orientation = toMsg(update.state.orientation);
    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose);
    while (path_max_poses_ > 0 &&
           path_msg_.poses.size() > static_cast<size_t>(path_max_poses_)) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    pub_path_->publish(path_msg_);
  }

  void maybePublishMarkers(
    const rclcpp::Time & stamp,
    const EstimatorUpdate & update)
  {
    if (have_last_marker_stamp_ &&
        (stamp - last_marker_stamp_).seconds() < marker_publish_period_) {
      return;
    }
    last_marker_stamp_ = stamp;
    have_last_marker_stamp_ = true;

    visualization_msgs::msg::MarkerArray array;
    for (int leg = 0; leg < 4; ++leg) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = odom_frame_;
      marker.ns = "wheel_support";
      marker.id = leg;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = update.supports[leg].contact_point_odom.x();
      marker.pose.position.y = update.supports[leg].contact_point_odom.y();
      marker.pose.position.z = update.supports[leg].contact_point_odom.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      const double w = std::max(0.0, std::min(1.0, update.supports[leg].weight));
      marker.color.r = 1.0 - w;
      marker.color.g = w;
      marker.color.b = 0.1;
      marker.color.a = 0.25 + 0.75 * w;
      array.markers.push_back(marker);
    }
    pub_markers_->publish(array);
  }

  std::string lowstate_topic_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string joint_state_topic_;
  std::string marker_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  int path_max_poses_ = 1000;
  double path_publish_period_ = 0.05;
  double marker_publish_period_ = 0.05;

  EstimatorParameters estimator_params_;
  std::unique_ptr<OdometryEstimator> estimator_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_lowstate_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_msg_;

  bool have_tick_anchor_ = false;
  uint32_t first_tick_ = 0;
  rclcpp::Time tick_anchor_time_{0, 0, RCL_ROS_TIME};
  bool have_last_stamp_ = false;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_path_stamp_ = false;
  rclcpp::Time last_path_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_marker_stamp_ = false;
  rclcpp::Time last_marker_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace wheel_legged_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_legged_odometry::WheelLeggedOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
