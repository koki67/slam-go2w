# dg_kilo — DG-KILO for Unitree Go2W

ROS 2 Humble package implementing **DG-KILO** (Kinematic-Inertial-LiDAR Odometry
Based on Degradation Optimization and Ground Constraints) for the Unitree Go2W
wheeled-legged quadruped.

> Xu et al., *DG-KILO: A Kinematic-Inertial-LiDAR Odometry Based on Degradation
> Optimization and Ground Constraints*, IEEE Transactions on Instrumentation &
> Measurement, 2026.

## Key Features

- **IESKF LIO** with Hessian eigenvalue-based degradation detection (Alg 1, Eq 13-15)
- **Intensity-gradient features** (Eq 7-9) activated only in degraded scenes
- **Adaptive scan slicing/stitching** driven by leg-odometry velocity (Eq 2-3)
- **Leg ESKF** with rolling-wheel contact model (Go2W adaptation of Co-RaL)
- **Ground height + coplanar constraints** (Eq 24-29) from stance-foot PCA plane fits
- **GTSAM iSAM2 factor graph** fusing LIO + leg-odom + ground + loop-closure (Eq 33)

## Topics

| Direction | Topic | Type | Rate |
|---|---|---|---|
| Subscribe | `/points_raw` | `sensor_msgs/PointCloud2` | 10 Hz |
| Subscribe | `/go2w/imu` | `sensor_msgs/Imu` | 500 Hz |
| Subscribe | `lowstate` | `unitree_go/LowState` | 500 Hz |
| Publish | `/dg_kilo/odom` | `nav_msgs/Odometry` | ~10 Hz |
| Publish | `/dg_kilo/leg_odom` | `dg_kilo/LegOdometry` | ~500 Hz |
| Publish | `/dg_kilo/path` | `nav_msgs/Path` | ~10 Hz |
| Publish | `/dg_kilo/map` | `sensor_msgs/PointCloud2` | ~1 Hz |
| Publish | `/dg_kilo/features/{edge,planar,intensity}` | `sensor_msgs/PointCloud2` | 10 Hz |
| Publish | `/dg_kilo/degradation` | `dg_kilo/DegradationStatus` | 10 Hz |
| Publish | `/dg_kilo/foot_contacts` | `visualization_msgs/MarkerArray` | 10 Hz |
| Publish | `/dg_kilo/ground_plane` | `visualization_msgs/Marker` | 10 Hz |
| Publish | `/dg_kilo/loop_marker` | `visualization_msgs/Marker` | on event |

## TF Tree

```
map → odom (loop-closure correction)
odom → base_link (fused estimate)
base_link → hesai_lidar (static, from go2w_calibration.yaml)
base_link → imu_link (static, from go2w_calibration.yaml)
base_link → {FL,FR,RL,RR}_foot (from URDF FK, published per-scan)
```

## Configuration

See `config/dgkilo/go2w.yaml` for the full parameter reference. Key parameters
are aligned with `config/sensor/go2w_calibration.yaml` — do not modify extrinsics
in one without updating the other.

## Build

```bash
# Inside the offline_dgkilo devcontainer:
colcon build --packages-select dg_kilo
colcon test  --packages-select dg_kilo
```

## Running

### Online (on robot)

```bash
catmux_create_session /external/catmux/online_dgkilo.yaml
```

### Offline (reconstruct from raw bag)

The input bag **must** contain `/lowstate` in addition to `/go2w/imu` and
`/points_raw`. Use `catmux/record_raw_legged.yaml` to record such a bag.
Existing raw bags captured before DG-KILO was added will not work.

```bash
bash scripts/dgkilo/reconstruct_raw.sh <bag_directory>
```

### Playback (replay DG-KILO bag)

```bash
bash scripts/dgkilo/playback.sh <bag_directory>
```

## Go2W Wheel-Contact Adaptation

The standard DG-KILO paper assumes point feet. For the Go2W, each "foot" is a
wheel with radius `wheel_radius = 0.0513 m`. The ESKF measurement model
constrains only the two **non-rolling** axes per wheel (longitudinal slip and
lateral); the rolling axis is left unconstrained with high noise
(`leg_eskf_wheel_slip_noise`). See `src/leg_eskf.cpp::updateWheelContact()`.

## Paper Citation

```bibtex
@article{xu2026dgkilo,
  author  = {Xu, ... and others},
  title   = {{DG-KILO}: A Kinematic-Inertial-LiDAR Odometry Based on
             Degradation Optimization and Ground Constraints},
  journal = {IEEE Transactions on Instrumentation and Measurement},
  year    = {2026},
}
```

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `/dg_kilo/leg_odom` silent | `/lowstate` not publishing | Check `ros2 topic hz lowstate` |
| Divergence on flat ground | `foot_force_threshold` mis-tuned | Record static bag; histogram `foot_force_est` |
| Loop closure never triggers | Bag is open-loop traverse | Expected; set `loop_enabled: false` |
| Ground markers below floor | `wheel_radius` wrong | Verify vs URDF `ORIGIN.md` |
