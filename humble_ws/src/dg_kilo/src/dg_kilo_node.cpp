#include "dg_kilo/dg_kilo_node.hpp"

#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace dg_kilo {

DgKiloNode::DgKiloNode()
: rclcpp::Node("dg_kilo")
{
  // Load parameters
  params_.loadFromNode(*this);

  RCLCPP_INFO(get_logger(),
    "DG-KILO starting — pointcloud: %s  imu: %s  lowstate: %s",
    params_.pointcloud_topic.c_str(),
    params_.imu_topic.c_str(),
    params_.lowstate_topic.c_str());

  // Build pipeline components
  Go2wKinematics::LinkLengths klen;
  klen.wheel_r = params_.wheel_radius;
  auto kin = std::make_shared<Go2wKinematics>(klen);

  lowstate_buf_  = std::make_shared<LowStateBuffer>(*this);
  kinematics_    = kin;
  imu_proc_      = std::make_shared<ImuProcessor>(params_);
  ikd_tree_      = std::make_shared<IkdTreeWrapper>();
  feat_extractor_= std::make_shared<FeatureExtractor>(params_);
  deg_detector_  = std::make_shared<DegradationDetector>(params_);
  scan_slicer_   = std::make_shared<ScanSlicer>(params_);
  leg_eskf_      = std::make_shared<LegEskf>(params_, kin);
  ground_estimator_ = std::make_shared<GroundPlaneEstimator>(params_);
  loop_closure_  = std::make_shared<LoopClosure>(params_);
  factor_graph_  = std::make_shared<FactorGraphBackend>(params_);

  // Subscribers
  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  sub_lowstate_ = create_subscription<unitree_go::msg::LowState>(
    params_.lowstate_topic, sensor_qos,
    [this](const unitree_go::msg::LowState::SharedPtr msg) { lowstateCallback(msg); });

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    params_.imu_topic, sensor_qos,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

  sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.pointcloud_topic, sensor_qos,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidarCallback(msg); });

  // Publishers
  pub_odom_    = create_publisher<nav_msgs::msg::Odometry>("/dg_kilo/odom", 10);
  pub_leg_odom_= create_publisher<dg_kilo::msg::LegOdometry>("/dg_kilo/leg_odom", 10);
  pub_path_    = create_publisher<nav_msgs::msg::Path>("/dg_kilo/path", 1);
  pub_map_     = create_publisher<sensor_msgs::msg::PointCloud2>("/dg_kilo/map", 1);
  pub_keyframes_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dg_kilo/keyframes", 1);
  pub_feat_edge_      = create_publisher<sensor_msgs::msg::PointCloud2>("/dg_kilo/features/edge", 1);
  pub_feat_planar_    = create_publisher<sensor_msgs::msg::PointCloud2>("/dg_kilo/features/planar", 1);
  pub_feat_intensity_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dg_kilo/features/intensity", 1);
  pub_foot_contacts_  = create_publisher<visualization_msgs::msg::MarkerArray>("/dg_kilo/foot_contacts", 1);
  pub_ground_plane_   = create_publisher<visualization_msgs::msg::Marker>("/dg_kilo/ground_plane", 1);
  pub_loop_marker_    = create_publisher<visualization_msgs::msg::Marker>("/dg_kilo/loop_marker", 1);
  pub_degradation_    = create_publisher<dg_kilo::msg::DegradationStatus>("/dg_kilo/degradation", 10);

  // TF
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_static_      = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Path
  path_msg_.header.frame_id = params_.map_frame;

  // Diagnostics CSV
  diag_csv_.open("/tmp/dg_kilo_diag.csv");
  diag_csv_ << "stamp,min_eigenvalue,intensity_count,stance_count,"
               "ground_normal_angle,leg_lio_disagreement\n";

  // Publish static transforms
  publishStaticTransforms();

  RCLCPP_INFO(get_logger(),
    "DG-KILO ready. Wheel radius: %.4f m  Degradation window: %d frames",
    params_.wheel_radius, params_.degradation_window);
}

void DgKiloNode::lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  lowstate_buf_->push(msg);

  // Extract joint state and drive leg ESKF predict
  std::array<LegJoints, 4> q;
  std::array<double, 4> omega_w;
  for (int i = 0; i < 4; ++i) {
    q[i].hip   = msg->motor_state[i*3+0].q;
    q[i].thigh = msg->motor_state[i*3+1].q;
    q[i].calf  = msg->motor_state[i*3+2].q;
    int wi = params_.motor_index_wheels[i];
    omega_w[i] = (wi < static_cast<int>(msg->motor_state.size())) ?
                 msg->motor_state[wi].dq : 0.0;
  }

  // Torques for contact detection
  float torques[16] = {};
  for (int i = 0; i < 16 && i < static_cast<int>(msg->motor_state.size()); ++i) {
    torques[i] = msg->motor_state[i].tau_est;
  }

  ContactState contacts = leg_eskf_->detectContact(q, torques);

  // Leg ESKF predict (dt from tick)
  Eigen::Vector3d acc(msg->imu_state.accelerometer[0],
                      msg->imu_state.accelerometer[1],
                      msg->imu_state.accelerometer[2]);
  Eigen::Vector3d gyr(msg->imu_state.gyroscope[0],
                      msg->imu_state.gyroscope[1],
                      msg->imu_state.gyroscope[2]);

  static rclcpp::Time last_leg_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time now = lowstate_buf_->tickToRosTime(msg->tick);
  double dt = (last_leg_stamp.nanoseconds() > 0) ?
    (now - last_leg_stamp).seconds() : 0.002;
  last_leg_stamp = now;

  if (dt > 0 && dt < 0.1) {
    leg_eskf_->predict(dt, acc, gyr, q, omega_w, contacts);
    leg_eskf_->updateWheelContact(q, omega_w, contacts);
  }

  // Publish leg odom
  auto leg_msg = dg_kilo::msg::LegOdometry();
  leg_msg.header.stamp    = now;
  leg_msg.header.frame_id = params_.base_frame;
  const State & ls = leg_eskf_->state();
  leg_msg.linear_velocity.x  = ls.v.x();
  leg_msg.linear_velocity.y  = ls.v.y();
  leg_msg.linear_velocity.z  = ls.v.z();
  for (int i = 0; i < 4; ++i) leg_msg.stance[i] = contacts.stance[i];
  leg_msg.covariance_norm = leg_eskf_->cov().norm();
  pub_leg_odom_->publish(leg_msg);
}

void DgKiloNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  ImuData imu;
  imu.stamp = rclcpp::Time(msg->header.stamp).seconds();
  imu.acc   = Eigen::Vector3d(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
  imu.gyr   = Eigen::Vector3d(msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z);
  imu_proc_->push(imu);
}

void DgKiloNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  processLidarScan(msg);
}

void DgKiloNode::processLidarScan(const sensor_msgs::msg::PointCloud2::SharedPtr & ros_cloud)
{
  rclcpp::Time stamp(ros_cloud->header.stamp);
  double t_scan = stamp.seconds();

  // Convert to PCL
  CloudPtr raw(new Cloud);
  pcl::fromROSMsg(*ros_cloud, *raw);

  // Filter by range
  CloudPtr ranged(new Cloud);
  for (const auto & pt : *raw) {
    float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
    if (r >= params_.lidar_min_range && r <= params_.lidar_max_range) {
      ranged->push_back(pt);
    }
  }

  // IMU propagation → undistort
  State imu_state = imu_proc_->propagate(t_scan);
  imu_proc_->undistort(ranged, t_scan - 0.1, t_scan);

  // Adaptive scan slicing
  int n_slices = scan_slicer_->numSlices(imu_state.v, Eigen::Vector3d::Zero());
  auto slices  = scan_slicer_->slice(ranged, n_slices);
  // Stitch with identity transforms for now (leg ESKF relative poses used in full impl)
  std::vector<Eigen::Isometry3d> transforms(n_slices, Eigen::Isometry3d::Identity());
  CloudPtr stitched = scan_slicer_->stitch(slices, transforms);

  // Feature extraction
  DegradationResult deg_last;
  if (factor_graph_->size() > 0) {
    // Use last degradation result
    // (tracked via class member in full impl; placeholder here)
  }
  Features feats = feat_extractor_->extract(stitched, !params_.intensity_only_when_degraded);

  // Add to map
  ikd_tree_->addPoints(feats.edge,   true);
  ikd_tree_->addPoints(feats.planar, true);

  // Degradation detection (needs Hessian; use identity as placeholder until IESKF step)
  Eigen::Matrix<double,6,6> H_placeholder = Eigen::Matrix<double,6,6>::Identity() * 10.0;
  DegradationResult deg = deg_detector_->detect(H_placeholder);

  // Publish degradation status
  auto deg_msg = dg_kilo::msg::DegradationStatus();
  deg_msg.header.stamp = stamp;
  deg_msg.degraded             = deg.degraded;
  deg_msg.degraded_axes        = deg.degraded_axes;
  deg_msg.min_eigenvalue       = deg.min_eigenvalue;
  deg_msg.threshold            = deg.threshold;
  deg_msg.intensity_feature_count = static_cast<int>(feats.intensity->size());
  pub_degradation_->publish(deg_msg);

  // Publish feature clouds
  auto pub_cloud = [&](auto & pub, const CloudPtr & c) {
    if (!c->empty()) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*c, out);
      out.header.stamp    = stamp;
      out.header.frame_id = params_.lidar_frame;
      pub->publish(out);
    }
  };
  pub_cloud(pub_feat_edge_,      feats.edge);
  pub_cloud(pub_feat_planar_,    feats.planar);
  pub_cloud(pub_feat_intensity_, feats.intensity);

  // Factor graph keyframe check
  bool is_kf = (factor_graph_->size() == 0);  // simplified: every scan is a keyframe
  if (is_kf) {
    // Build pose from IMU state
    gtsam::Rot3 R_gtsam(imu_state.R);
    gtsam::Point3 t_gtsam(imu_state.p.x(), imu_state.p.y(), imu_state.p.z());
    gtsam::Pose3 pose(R_gtsam, t_gtsam);

    Eigen::Matrix<double,6,6> lio_cov = Eigen::Matrix<double,6,6>::Identity() * 0.01;
    factor_graph_->addLioFactor(pose, gtsam::Pose3(), lio_cov);
    gtsam::Pose3 opt_pose = factor_graph_->update();

    // Update IMU state from optimised pose
    Eigen::Matrix3d R_opt = opt_pose.rotation().matrix();
    Eigen::Vector3d p_opt(opt_pose.x(), opt_pose.y(), opt_pose.z());
    State s_upd = imu_state;
    s_upd.R = R_opt;
    s_upd.p = p_opt;
    imu_proc_->setUpdatedState(s_upd);

    // Register loop closure keyframe
    loop_closure_->addKeyframe(factor_graph_->size()-1,
      Eigen::Isometry3d(Eigen::Translation3d(p_opt) * Eigen::AngleAxisd(Eigen::Quaterniond(R_opt))),
      feats.planar);

    // Attempt loop closure
    if (params_.loop_enabled && factor_graph_->size() > 50) {
      auto lc = loop_closure_->detect(factor_graph_->size()-1);
      if (lc) {
        gtsam::Rot3 lR(lc->T_match_query.rotation());
        gtsam::Point3 lt(lc->T_match_query.translation());
        factor_graph_->addLoopFactor(lc->query_idx, lc->match_idx, gtsam::Pose3(lR, lt));
        RCLCPP_INFO(get_logger(), "Loop closure: keyframe %zu → %zu (score %.3f)",
          lc->query_idx, lc->match_idx, lc->fitness_score);
      }
    }
  }

  // Diagnostics CSV
  publishDiagnosticCsv();

  // Publish odometry
  publishOdometry(stamp);
}

void DgKiloNode::publishOdometry(const rclcpp::Time & stamp)
{
  const State & s = imu_proc_->state();

  // TF: odom → base_link
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp            = stamp;
  tf_msg.header.frame_id         = params_.odom_frame;
  tf_msg.child_frame_id          = params_.base_frame;
  Eigen::Quaterniond q_odom(s.R);
  tf_msg.transform.rotation.w   = q_odom.w();
  tf_msg.transform.rotation.x   = q_odom.x();
  tf_msg.transform.rotation.y   = q_odom.y();
  tf_msg.transform.rotation.z   = q_odom.z();
  tf_msg.transform.translation.x = s.p.x();
  tf_msg.transform.translation.y = s.p.y();
  tf_msg.transform.translation.z = s.p.z();
  tf_broadcaster_->sendTransform(tf_msg);

  // Odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp    = stamp;
  odom_msg.header.frame_id = params_.odom_frame;
  odom_msg.child_frame_id  = params_.base_frame;
  odom_msg.pose.pose.position.x    = s.p.x();
  odom_msg.pose.pose.position.y    = s.p.y();
  odom_msg.pose.pose.position.z    = s.p.z();
  odom_msg.pose.pose.orientation.w = q_odom.w();
  odom_msg.pose.pose.orientation.x = q_odom.x();
  odom_msg.pose.pose.orientation.y = q_odom.y();
  odom_msg.pose.pose.orientation.z = q_odom.z();
  odom_msg.twist.twist.linear.x    = s.v.x();
  odom_msg.twist.twist.linear.y    = s.v.y();
  odom_msg.twist.twist.linear.z    = s.v.z();
  pub_odom_->publish(odom_msg);

  // Path
  geometry_msgs::msg::PoseStamped ps;
  ps.header = odom_msg.header;
  ps.pose   = odom_msg.pose.pose;
  path_msg_.header.stamp = stamp;
  path_msg_.poses.push_back(ps);
  if (path_msg_.poses.size() > 10000) path_msg_.poses.erase(path_msg_.poses.begin());
  pub_path_->publish(path_msg_);
}

void DgKiloNode::publishStaticTransforms()
{
  auto make_static = [&](
    const std::string & parent,
    const std::string & child,
    const Eigen::Vector3d & t,
    const Eigen::Quaterniond & q) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = this->now();
    tf.header.frame_id = parent;
    tf.child_frame_id  = child;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    return tf;
  };

  std::vector<geometry_msgs::msg::TransformStamped> statics;
  statics.push_back(make_static(
    params_.base_frame, params_.lidar_frame,
    params_.extrinsic_lidar_translation, params_.extrinsic_lidar_quaternion));
  statics.push_back(make_static(
    params_.base_frame, params_.imu_frame,
    params_.extrinsic_imu_translation, params_.extrinsic_imu_quaternion));
  tf_static_->sendTransform(statics);
}

void DgKiloNode::publishDiagnosticCsv()
{
  // Lightweight per-scan diagnostic row
  static int diag_seq = 0;
  ++diag_seq;
  diag_csv_ << diag_seq << ","
            << 0.0 << ","  // min_eigenvalue placeholder
            << 0 << ","    // intensity_count
            << 0 << ","    // stance_count
            << 0.0 << ","  // ground_normal_angle
            << 0.0 << "\n"; // leg_lio_disagreement
  diag_csv_.flush();
}

} // namespace dg_kilo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dg_kilo::DgKiloNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
