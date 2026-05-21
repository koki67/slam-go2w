#include "dg_kilo/lowstate_buffer.hpp"
#include <chrono>

namespace dg_kilo {

LowStateBuffer::LowStateBuffer(rclcpp::Node & node)
: node_(node) {}

void LowStateBuffer::push(const unitree_go::msg::LowState::SharedPtr & msg)
{
  rclcpp::Time ros_t = tickToRosTime(msg->tick);

  std::lock_guard<std::mutex> lk(mutex_);
  Entry e;
  e.ros_time = ros_t;
  e.state    = *msg;
  buf_.push_back(e);
  while (buf_.size() > kCapacity) buf_.pop_front();
}

rclcpp::Time LowStateBuffer::tickToRosTime(uint32_t tick_ms)
{
  // Mirrors imu_publisher.cpp::tickToRosTime — anchor once, then offset.
  if (!have_anchor_) {
    first_tick_ = tick_ms;
    base_time_  = node_.get_clock()->now();
    have_anchor_ = true;
    return base_time_;
  }

  int64_t delta_ms =
    static_cast<int64_t>(tick_ms) - static_cast<int64_t>(first_tick_);

  // uint32 ms wraps every ~49.7 days
  if (delta_ms < std::numeric_limits<int32_t>::min()) {
    delta_ms += static_cast<int64_t>(1) << 32;
  }

  // Controller restart: re-anchor
  if (delta_ms < -1000) {
    first_tick_ = tick_ms;
    base_time_  = node_.get_clock()->now();
    return base_time_;
  }

  return base_time_ + rclcpp::Duration(std::chrono::milliseconds(delta_ms));
}

std::optional<unitree_go::msg::LowState> LowStateBuffer::lookup(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (buf_.empty()) return std::nullopt;

  // Find bracketing entries
  const Entry * before = nullptr;
  const Entry * after  = nullptr;
  for (const auto & e : buf_) {
    if (e.ros_time <= stamp) before = &e;
    else { after = &e; break; }
  }

  if (!before) return std::nullopt;  // stamp before buffer
  if (!after)  return before->state; // stamp after buffer — extrapolate not attempted

  // Linear interpolation of motor positions (sufficient for FK)
  double alpha = (stamp - before->ros_time).seconds() /
                 (after->ros_time - before->ros_time).seconds();
  alpha = std::max(0.0, std::min(1.0, alpha));

  unitree_go::msg::LowState interp = before->state;
  for (int i = 0; i < 20 && i < static_cast<int>(interp.motor_state.size()); ++i) {
    float q_a = before->state.motor_state[i].q;
    float q_b = after->state.motor_state[i].q;
    interp.motor_state[i].q = q_a + static_cast<float>(alpha) * (q_b - q_a);

    float dq_a = before->state.motor_state[i].dq;
    float dq_b = after->state.motor_state[i].dq;
    interp.motor_state[i].dq = dq_a + static_cast<float>(alpha) * (dq_b - dq_a);
  }
  return interp;
}

} // namespace dg_kilo
