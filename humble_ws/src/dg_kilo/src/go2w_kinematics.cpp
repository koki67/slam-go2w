#include "dg_kilo/go2w_kinematics.hpp"
#include <cmath>

namespace dg_kilo {

// Hip origins in base_link (FL, FR, RL, RR)
const std::array<Eigen::Vector3d, Go2wKinematics::kNumLegs> Go2wKinematics::kHipOrigin = {{
  { 0.1934,  0.0465, 0.0},   // FL
  { 0.1934, -0.0465, 0.0},   // FR
  {-0.1934,  0.0465, 0.0},   // RL
  {-0.1934, -0.0465, 0.0},   // RR
}};

// +1 for left legs (abduction moves foot outward toward +y), -1 for right
const std::array<double, Go2wKinematics::kNumLegs> Go2wKinematics::kHipYSign = {{1, -1, 1, -1}};

Go2wKinematics::Go2wKinematics()
: Go2wKinematics(LinkLengths{}) {}

Go2wKinematics::Go2wKinematics(const LinkLengths & lengths)
: lengths_(lengths) {}

LegFkResult Go2wKinematics::legFk(int leg_idx, const LegJoints & q) const
{
  const double q_hip   = q.hip;
  const double q_thigh = q.thigh;
  const double q_calf  = q.calf;
  const double s  = kHipYSign[leg_idx];
  const double l1 = lengths_.d_hip;    // hip y offset
  const double l2 = lengths_.l_thigh;  // thigh
  const double l3 = lengths_.l_calf;   // calf

  // FK from hip joint to foot in hip frame (Rz not applied; hip abducts about x)
  // Unravel the kinematic chain:
  //   p_hip→thigh in hip-frame = [0, s·l1, 0]
  //   Rotate about hip-x by q_hip
  //   p_thigh→calf in thigh-frame = [0, 0, -l2]
  //   Rotate about thigh-y by q_thigh
  //   p_calf→foot in calf-frame = [0, 0, -l3]
  //   Rotate about calf-y by q_calf
  //   (wheel axis is calf-y, so foot_joint is continuous about y)

  // Rotation from hip abduction
  Eigen::Matrix3d R_hip;
  R_hip << 1,          0,           0,
           0,  std::cos(q_hip), -std::sin(q_hip),
           0,  std::sin(q_hip),  std::cos(q_hip);

  // Thigh pitch
  Eigen::Matrix3d R_thigh;
  R_thigh <<  std::cos(q_thigh), 0, std::sin(q_thigh),
              0,                 1, 0,
             -std::sin(q_thigh), 0, std::cos(q_thigh);

  // Calf pitch
  Eigen::Matrix3d R_calf;
  R_calf <<  std::cos(q_calf), 0, std::sin(q_calf),
             0,                1, 0,
            -std::sin(q_calf), 0, std::cos(q_calf);

  // Foot position in hip frame
  Eigen::Vector3d p_hip_thigh(0, s * l1, 0);
  Eigen::Vector3d p_thigh_calf = R_hip * Eigen::Vector3d(0, 0, -l2);
  Eigen::Vector3d p_calf_foot  = R_hip * R_thigh * Eigen::Vector3d(0, 0, -l3);

  Eigen::Vector3d p_hip_foot = p_hip_thigh + p_thigh_calf + p_calf_foot;

  // Transform to base_link: p_base_foot = hip_origin + R_base_hip * p_hip_foot
  // R_base_hip = Identity (hip joint has no rotation in rest pose; base_link aligns with body)
  LegFkResult res;
  res.foot_pos_base = kHipOrigin[leg_idx] + p_hip_foot;
  res.foot_rot_base = R_hip * R_thigh * R_calf;

  // Foot Jacobian: d(foot_pos_base)/d(q) [3×3] via numerical differentiation
  // (analytic version left for future optimisation)
  const double eps = 1e-5;
  for (int col = 0; col < 3; ++col) {
    LegJoints q_plus = q;
    switch(col) {
      case 0: q_plus.hip   += eps; break;
      case 1: q_plus.thigh += eps; break;
      case 2: q_plus.calf  += eps; break;
    }
    LegFkResult r_plus = legFk(leg_idx, q_plus);
    res.J.col(col) = (r_plus.foot_pos_base - res.foot_pos_base) / eps;
  }

  return res;
}

WheelContactVelocity Go2wKinematics::contactVelocity(
  int leg_idx,
  const LegJoints & q,
  const Eigen::Vector3d & v_base,
  const Eigen::Vector3d & omega_base,
  double omega_wheel) const
{
  LegFkResult fk = legFk(leg_idx, q);

  // Wheel rolling axis in base_link frame: the calf-y axis transformed to base_link
  Eigen::Vector3d t_rolling_base = fk.foot_rot_base.col(1);  // y-axis of foot frame

  // Foot velocity from rigid body kinematics: v_foot = v_base + omega_base × p_foot
  Eigen::Vector3d v_fk = v_base + omega_base.cross(fk.foot_pos_base);

  // Add wheel spin contribution: r·ω_wheel along the rolling tangent
  // The tangent direction perpendicular to t_rolling in the vertical plane
  Eigen::Vector3d t_tangent = Eigen::Vector3d(0,0,1).cross(t_rolling_base).normalized();
  Eigen::Vector3d v_roll = lengths_.wheel_r * omega_wheel * t_tangent;

  WheelContactVelocity wc;
  wc.v_contact = v_fk + v_roll;
  wc.t_rolling = t_rolling_base;
  return wc;
}

} // namespace dg_kilo
