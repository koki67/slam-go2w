#pragma once
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include "unitree_go/msg/low_state.hpp"

namespace dg_kilo {

// Ring buffer keyed by Unitree tick (milliseconds, uint32).
// Replicates imu_publisher's tickToRosTime anchor logic so leg-odom timestamps
// stay coherent with /go2w/imu even during DDS burst delivery.
class LowStateBuffer
{
public:
  static constexpr size_t kCapacity = 2048;  // covers ~4 s at 500 Hz

  explicit LowStateBuffer(rclcpp::Node & node);

  // Ingest a new LowState message.
  void push(const unitree_go::msg::LowState::SharedPtr & msg);

  // Interpolate a LowState at the given ROS timestamp.
  // Returns nullopt when the stamp falls outside the buffered window.
  std::optional<unitree_go::msg::LowState> lookup(const rclcpp::Time & stamp) const;

  // Convert Unitree tick_ms → ROS time using the anchored offset.
  rclcpp::Time tickToRosTime(uint32_t tick_ms);

private:
  struct Entry {
    rclcpp::Time    ros_time;
    unitree_go::msg::LowState state;
  };

  mutable std::mutex mutex_;
  std::deque<Entry>  buf_;
  rclcpp::Node &     node_;

  // Anchor state (mirrors imu_publisher.cpp)
  bool     have_anchor_  = false;
  uint32_t first_tick_   = 0;
  rclcpp::Time base_time_{0, 0, RCL_ROS_TIME};
};

} // namespace dg_kilo
