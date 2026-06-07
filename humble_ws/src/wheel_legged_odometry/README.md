# wheel_legged_odometry

Minimal force-free kinematic odometry for the Unitree Go2W.

The node subscribes to `unitree_go/msg/LowState`, extracts all 16 motor states,
computes Go2W FK for the four leg-wheel chains, and estimates `odom ->
base_link` from kinematic support consistency. It does not require foot-force
contact sensing.

## Topics

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Subscribe | `lowstate` | `unitree_go/LowState` | 16 motor states + IMU |
| Publish | `/wheel_legged_odometry/odom` | `nav_msgs/Odometry` | Base odometry |
| Publish | `/wheel_legged_odometry/path` | `nav_msgs/Path` | Trajectory |
| Publish | `/wheel_legged_odometry/joint_states` | `sensor_msgs/JointState` | Robot model animation |
| Publish | `/wheel_legged_odometry/support_markers` | `visualization_msgs/MarkerArray` | Inferred support weights |

## Estimator

The first valid low-state defines the odometry origin. FK and IMU gravity are
used to initialize the robot posture and local ground reference. Each update
solves a small robust weighted least-squares problem for base planar twist:

- leg joint velocity moves the wheel contact candidate through FK Jacobians,
- wheel joint velocity contributes rolling tangent velocity,
- candidates near the inferred ground plane are weighted higher,
- high kinematic residuals are downweighted,
- IMU yaw-rate acts as a stabilizing prior.

This is a kinematic odometry baseline, not SLAM and not a full contact/physics
estimator. If a wheel spins in free space and remains geometrically plausible as
a support candidate, v1 can accumulate odometry error.

## Go2W Motor Map

Default motor map is ordered by leg `FL, FR, RL, RR`, each as
`hip, thigh, calf, wheel`:

```yaml
motor_indices: [3, 4, 5, 13, 0, 1, 2, 12, 9, 10, 11, 15, 6, 7, 8, 14]
```

This matches the Go2W deployment configs under
`unitree_rl_mjlab_research/deploy/robots/go2w`.

## Usage

Online robot-side:

```bash
catmux_create_session /external/catmux/online_wheel_legged_odometry.yaml
```

Record replayable outputs:

```bash
catmux_create_session /external/catmux/record_wheel_legged_odometry.yaml
```

Record raw inputs for offline reconstruction:

```bash
catmux_create_session /external/catmux/record_raw_wheel_legged.yaml
```

Desktop replay from raw:

```bash
bash scripts/wheel_legged_odometry/reconstruct_raw.sh humble_ws/bags/raw_wheel_legged_YYYYMMDD_HHMMSS
```
