#pragma once
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace dg_kilo {

// Joint angles for one leg: [hip, thigh, calf]
struct LegJoints {
  double hip   = 0.0;
  double thigh = 0.0;
  double calf  = 0.0;
};

// Forward-kinematics result for one leg
struct LegFkResult {
  Eigen::Vector3d    foot_pos_base;    // wheel centre in base_link
  Eigen::Matrix3d    foot_rot_base;    // R_base_foot
  Eigen::Matrix<double,3,3> J;         // foot Jacobian d(foot_pos)/d(q) 3×3
};

// Wheel-contact velocity for one leg
struct WheelContactVelocity {
  Eigen::Vector3d v_contact;  // in base_link frame
  Eigen::Vector3d t_rolling;  // wheel rolling axis unit vector in base_link
};

class Go2wKinematics
{
public:
  // Leg indices: 0=FL, 1=FR, 2=RL, 3=RR
  static constexpr int kNumLegs = 4;

  // Link lengths parsed from URDF at construction.
  struct LinkLengths {
    double d_hip   = 0.0955; // hip lateral offset (y)
    double l_thigh = 0.213;  // thigh length (z)
    double l_calf  = 0.213;  // calf length  (z)
    double wheel_r = 0.0513; // wheel radius
  };

  // Hip origins in base_link (x, y, z)
  static const std::array<Eigen::Vector3d, kNumLegs> kHipOrigin;
  // Hip x-axis sign (left legs +1, right legs -1) for the abduction joint
  static const std::array<double, kNumLegs> kHipYSign;

  Go2wKinematics();
  explicit Go2wKinematics(const LinkLengths & lengths);

  // Full FK for one leg (joint angles in radians).
  LegFkResult legFk(int leg_idx, const LegJoints & q) const;

  // Rolling-contact velocity: v_contact = v_FK + r·ω_wheel · t_rolling
  //   q         — 3-joint angles
  //   v_base    — base linear velocity in base_link
  //   omega_base— base angular velocity in base_link
  //   omega_wheel — wheel angular velocity (motor_state[12+leg])
  WheelContactVelocity contactVelocity(
    int leg_idx,
    const LegJoints & q,
    const Eigen::Vector3d & v_base,
    const Eigen::Vector3d & omega_base,
    double omega_wheel) const;

  const LinkLengths & lengths() const { return lengths_; }

private:
  LinkLengths lengths_;
};

} // namespace dg_kilo
