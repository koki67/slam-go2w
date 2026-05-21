#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <fstream>

#include "dg_kilo/params.hpp"
#include "dg_kilo/lowstate_buffer.hpp"
#include "dg_kilo/go2w_kinematics.hpp"
#include "dg_kilo/imu_processor.hpp"
#include "dg_kilo/ikd_tree_wrapper.hpp"
#include "dg_kilo/feature_extractor.hpp"
#include "dg_kilo/degradation_detector.hpp"
#include "dg_kilo/scan_slicer.hpp"
#include "dg_kilo/leg_eskf.hpp"
#include "dg_kilo/ground_plane_estimator.hpp"
#include "dg_kilo/loop_closure.hpp"
#include "dg_kilo/factor_graph_backend.hpp"

#include "dg_kilo/msg/leg_odometry.hpp"
#include "dg_kilo/msg/degradation_status.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace dg_kilo {

class DgKiloNode : public rclcpp::Node
{
public:
  DgKiloNode();

private:
  // Callbacks
  void lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Processing pipeline
  void processLidarScan(const sensor_msgs::msg::PointCloud2::SharedPtr & cloud);
  void publishOdometry(const rclcpp::Time & stamp);
  void publishStaticTransforms();
  void publishDiagnosticCsv();

  // Parameters
  DgKiloParams params_;

  // Pipeline components
  std::shared_ptr<LowStateBuffer> lowstate_buf_;
  std::shared_ptr<Go2wKinematics> kinematics_;
  std::shared_ptr<ImuProcessor>   imu_proc_;
  std::shared_ptr<IkdTreeWrapper> ikd_tree_;
  std::shared_ptr<FeatureExtractor>     feat_extractor_;
  std::shared_ptr<DegradationDetector>  deg_detector_;
  std::shared_ptr<ScanSlicer>           scan_slicer_;
  std::shared_ptr<LegEskf>             leg_eskf_;
  std::shared_ptr<GroundPlaneEstimator> ground_estimator_;
  std::shared_ptr<LoopClosure>          loop_closure_;
  std::shared_ptr<FactorGraphBackend>   factor_graph_;

  // Subscribers
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr  sub_lowstate_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       pub_odom_;
  rclcpp::Publisher<dg_kilo::msg::LegOdometry>::SharedPtr     pub_leg_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keyframes_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feat_edge_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feat_planar_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feat_intensity_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_foot_contacts_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ground_plane_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_loop_marker_;
  rclcpp::Publisher<dg_kilo::msg::DegradationStatus>::SharedPtr pub_degradation_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

  // Path accumulation
  nav_msgs::msg::Path path_msg_;

  // Diagnostics CSV
  std::ofstream diag_csv_;
};

} // namespace dg_kilo
