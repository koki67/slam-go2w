#include "dg_kilo/imu_processor.hpp"
#include <stdexcept>

namespace dg_kilo {

ImuProcessor::ImuProcessor(const DgKiloParams & params)
: params_(params),
  state_cov_(Eigen::MatrixXd::Identity(kStateDim, kStateDim) * 1e-4)
{
  state_.g = Eigen::Vector3d(0, 0, -params_.gravity);
}

void ImuProcessor::push(const ImuData & imu)
{
  imu_buf_.push_back(imu);
  // Keep buffer bounded (10 s at 500 Hz = 5000 samples)
  while (imu_buf_.size() > 5000) imu_buf_.pop_front();
}

State ImuProcessor::propagate(double target_stamp)
{
  if (imu_buf_.empty()) return state_;
  if (last_stamp_ < 0) {
    last_stamp_ = imu_buf_.front().stamp;
  }

  prop_states_.clear();
  prop_states_.push_back({last_stamp_, state_});

  auto it = imu_buf_.begin();
  // Skip IMU samples before last_stamp
  while (it != imu_buf_.end() && it->stamp <= last_stamp_) ++it;

  ImuData prev = {last_stamp_, state_.v /*acc placeholder*/, state_.bw /*gyr placeholder*/};
  if (it != imu_buf_.begin()) {
    auto p = std::prev(it);
    prev = *p;
  }

  for (; it != imu_buf_.end(); ++it) {
    double t1 = it->stamp;
    if (t1 > target_stamp) {
      // Interpolate to target_stamp
      double alpha = (target_stamp - prev.stamp) / (t1 - prev.stamp + 1e-12);
      ImuData interp;
      interp.stamp = target_stamp;
      interp.acc   = prev.acc + alpha * (it->acc - prev.acc);
      interp.gyr   = prev.gyr + alpha * (it->gyr - prev.gyr);
      double dt = target_stamp - prev.stamp;
      if (dt > 0) propagateStep(prev, interp, dt);
      prop_states_.push_back({target_stamp, state_});
      break;
    }
    double dt = t1 - prev.stamp;
    if (dt > 0) propagateStep(prev, *it, dt);
    prop_states_.push_back({t1, state_});
    prev = *it;
  }

  last_stamp_ = target_stamp;
  return state_;
}

void ImuProcessor::propagateStep(const ImuData & a, const ImuData & b, double dt)
{
  // Mid-point integration (FAST-LIO2 backward propagation style, Eq 4-5)
  Eigen::Vector3d gyr_mid = 0.5 * (a.gyr + b.gyr) - state_.bw;
  Eigen::Vector3d acc_mid = 0.5 * (a.acc + b.acc) - state_.ba;

  Eigen::Vector3d angle = gyr_mid * dt;
  double angle_norm = angle.norm();
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  if (angle_norm > 1e-12) {
    Eigen::AngleAxisd aa(angle_norm, angle / angle_norm);
    dR = aa.toRotationMatrix();
  }

  Eigen::Vector3d acc_world = state_.R * acc_mid + state_.g;

  state_.p  = state_.p + state_.v * dt + 0.5 * acc_world * dt * dt;
  state_.v  = state_.v + acc_world * dt;
  state_.R  = state_.R * dR;

  // Simple first-order covariance propagation (Q diagonal)
  // Full IESKF covariance tracked in 18×18; here simplified to 15×15 for skeleton
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
  // ∂p/∂v, ∂v/∂R (skew of acc_world)
  F.block<3,3>(3,6) = Eigen::Matrix3d::Identity() * dt;
  // Noise
  double sa = params_.imu_acc_noise;
  double sw = params_.imu_gyr_noise;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  Q.block<3,3>(6,6)  = Eigen::Matrix3d::Identity() * sa * sa * dt;
  Q.block<3,3>(0,0)  = Eigen::Matrix3d::Identity() * sw * sw * dt;
  Q.block<3,3>(9,9)  = Eigen::Matrix3d::Identity() * params_.imu_acc_bias * dt;
  Q.block<3,3>(12,12)= Eigen::Matrix3d::Identity() * params_.imu_gyr_bias * dt;

  state_cov_ = F * state_cov_ * F.transpose() + Q;
}

void ImuProcessor::undistort(CloudPtr & cloud, double sweep_start, double sweep_end) const
{
  if (prop_states_.size() < 2) return;

  // Interpolate pose at each point timestamp and transform to sweep_start frame.
  State s0 = prop_states_.front().second;
  for (auto & pt : *cloud) {
    // Point timestamp encoded in intensity field (convention from Hesai driver)
    double t = sweep_start + (pt.intensity / 1e9);
    // Clamp
    t = std::max(sweep_start, std::min(sweep_end, t));

    // Interpolate state
    State s_t = s0;
    for (size_t i = 1; i < prop_states_.size(); ++i) {
      if (prop_states_[i].first >= t) {
        double alpha = (t - prop_states_[i-1].first) /
                       (prop_states_[i].first - prop_states_[i-1].first + 1e-12);
        const State & sa = prop_states_[i-1].second;
        const State & sb = prop_states_[i].second;
        s_t.p = sa.p + alpha * (sb.p - sa.p);
        // Slerp rotation
        Eigen::Quaterniond qa(sa.R), qb(sb.R);
        s_t.R = qa.slerp(alpha, qb).toRotationMatrix();
        break;
      }
    }

    // T_{s0}^{-1} * T_{s_t}
    Eigen::Vector3d p_world(pt.x, pt.y, pt.z);
    Eigen::Vector3d p_world_origin = s_t.R * p_world + s_t.p;
    Eigen::Vector3d p_local = s0.R.transpose() * (p_world_origin - s0.p);
    pt.x = static_cast<float>(p_local.x());
    pt.y = static_cast<float>(p_local.y());
    pt.z = static_cast<float>(p_local.z());
  }
}

void ImuProcessor::setUpdatedState(const State & s)
{
  state_ = s;
  imu_buf_.clear();  // clear pre-sweep buffer
}

} // namespace dg_kilo
