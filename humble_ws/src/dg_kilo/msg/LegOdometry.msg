std_msgs/Header header

# Estimated base_link velocity in base_link frame (from leg ESKF)
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 angular_velocity

# Per-foot contact flags (FL, FR, RL, RR)
bool[4] stance

# Per-foot velocity in world frame (wheel-contact point)
geometry_msgs/Vector3[4] foot_velocity_world

# Leg ESKF state covariance norm (scalar summary for diagnostics)
float64 covariance_norm
