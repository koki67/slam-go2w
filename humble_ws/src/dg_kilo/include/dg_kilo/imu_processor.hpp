#pragma once
#include <deque>
#include <Eigen/Core>
#include "dg_kilo/common_types.hpp"
#include "dg_kilo/params.hpp"

namespace dg_kilo {

// FAST-LIO2-style IMU pre-integrator.
// Maintains a propagated state + covariance between consecutive LiDAR sweeps.
// After each LiDAR update the propagated states are reset to the posterior.
class ImuProcessor
{
public:
  static constexpr int kStateDim = 18; // R(9) p(3) v(3) ba(3) bw(3)

  explicit ImuProcessor(const DgKiloParams & params);

  // Feed a new IMU measurement.
  void push(const ImuData & imu);

  // Forward-propagate state from last_stamp to target_stamp using buffered IMUs.
  // Returns the propagated state and fills state_cov_.
  State propagate(double target_stamp);

  // Backward distortion: undo the motion distortion for a point cloud
  // measured in [sweep_start, sweep_end] using the propagated IMU states.
  void undistort(CloudPtr & cloud, double sweep_start, double sweep_end) const;

  // Update posterior state after LiDAR Kalman step.
  void setUpdatedState(const State & s);

  const State & state() const { return state_; }
  const Eigen::MatrixXd & cov() const { return state_cov_; }

private:
  void propagateStep(const ImuData & a, const ImuData & b, double dt);

  DgKiloParams    params_;
  State           state_;
  Eigen::MatrixXd state_cov_;
  std::deque<ImuData> imu_buf_;
  double last_stamp_ = -1.0;

  // Store intermediate propagated states for undistortion
  std::vector<std::pair<double, State>> prop_states_;
};

} // namespace dg_kilo
