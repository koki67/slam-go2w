std_msgs/Header header

# True when at least one eigen-axis is degraded per Alg 1 (Eq 13-15)
bool degraded

# Number of degraded eigenvector axes (0-3)
int32 degraded_axes

# Minimum Hessian eigenvalue (log scale indicator of geometric quality)
float64 min_eigenvalue

# Rolling threshold τ = μ − α·σ over the last N frames
float64 threshold

# Number of intensity features added this scan (non-zero only when degraded)
int32 intensity_feature_count
