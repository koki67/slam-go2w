# slam-go2w

Unified SLAM repository for the Unitree GO2-W robot with Hesai PandarXT-16 LiDAR.

Choose your SLAM algorithm — currently supports **D-LIO** (online + offline) and **GLIM** (offline), with an extensible layout for adding more.

## Table of contents

- [Repository layout](#repository-layout)
- [Submodules](#submodules)
- [Quick start: Online D-LIO (on robot)](#quick-start-online-d-lio-on-robot)
  - [Desktop RViz over WiFi](#desktop-rviz-over-wifi)
- [Quick start: Record D-LIO outputs (on robot)](#quick-start-record-d-lio-outputs-on-robot)
- [Quick start: Record raw sensor data](#quick-start-record-raw-sensor-data)
- [Quick start: Desktop replay of recorded D-LIO outputs](#quick-start-desktop-replay-of-recorded-d-lio-outputs)
- [Quick start: Desktop offline D-LIO reconstruction](#quick-start-desktop-offline-d-lio-reconstruction)
- [Quick start: Desktop offline GLIM processing](#quick-start-desktop-offline-glim-processing)
- [Catmux sessions](#catmux-sessions)
- [Sensor calibration](#sensor-calibration)
- [Adding a new SLAM algorithm](#adding-a-new-slam-algorithm)
- [License](#license)

## Repository layout

```
slam-go2w/
├── humble_ws/src/                  ROS 2 Humble workspace
│   ├── go2_unitree_ros2/           IMU republisher (submodule)
│   ├── unitree_ros2/               Unitree DDS + messages (submodule)
│   ├── go2w-hesai-lidar-driver/  Hesai XT16 driver (submodule)
│   └── direct_lidar_inertial_odometry/  D-LIO algorithm (submodule)
├── docker/
│   ├── robot/                      ARM64 robot Docker image
│   └── desktop/glim/               Desktop GLIM offline wrapper
├── .devcontainer/                  Desktop D-LIO devcontainer
├── catmux/                         Robot-side terminal sessions
├── config/
│   ├── sensor/                     Shared calibration reference
│   ├── dlio/                       D-LIO-specific configs
│   └── glim/offline/               GLIM override configs + experiments
├── scripts/
│   ├── dlio/                       D-LIO offline scripts
│   └── glim/                       GLIM offline pipeline scripts
├── tools/                          Bag validation and analysis tools
└── output/                         Run artifacts (gitignored)
```

## Submodules

| Package | Repository | Branch | Purpose |
|---------|-----------|--------|---------|
| go2_unitree_ros2 | koki67/go2_unitree_ros2 | imu_publisher | GO2-W IMU republisher |
| unitree_ros2 | koki67/unitree_ros2 | master | Unitree ROS2 bindings & DDS |
| go2w-hesai-lidar-driver | koki67/go2w-hesai-lidar-driver | main | Hesai XT16 ROS2 driver |
| direct_lidar_inertial_odometry | koki67/direct_lidar_inertial_odometry | feature/ros2 | D-LIO SLAM algorithm |

Clone with submodules:
```bash
git clone --recurse-submodules https://github.com/koki67/slam-go2w.git
```

The examples below assume this repository is cloned at `~/ws/slam-go2w`. Adjust the path if your workspace lives elsewhere.

## Quick start: Online D-LIO (on robot)

1. Build the robot Docker image:
   ```bash
   docker build -f ~/ws/slam-go2w/docker/robot/Dockerfile -t go2w-humble:latest ~/ws/slam-go2w
   ```

2. Start the container:
   ```bash
   bash ~/ws/slam-go2w/docker/robot/run.sh
   ```

3. Inside the container, build the workspace:
   ```bash
   cd /external/humble_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Create and attach the online D-LIO tmux session:
   ```bash
   catmux_create_session /external/catmux/online_dlio.yaml
   ```

This session sources `humble_ws/src/unitree_ros2/setup.sh`, which enables CycloneDDS on `eth0` and also `wlan0` when that interface exists on the robot host. If the robot host `wlan0` is configured on your WiFi network, the D-LIO topics are published over WiFi from this containerized setup as well.

### Desktop RViz over WiFi

For desktop visualization, use this repository's devcontainer. It is based on `osrf/ros:humble-desktop` and already runs with host networking, which lets RViz join the same WiFi DDS traffic as the robot.

1. Open `~/ws/slam-go2w` in VS Code and reopen it in the devcontainer.
2. On the Ubuntu host, allow local root access to the X server if needed:
   ```bash
   xhost +si:localuser:root
   ```
3. Inside the devcontainer, run:
   ```bash
   bash scripts/dlio/live_rviz.sh --iface enp97s0
   ```

Replace `enp97s0` with the actual desktop-side interface name. Keep `ROS_DOMAIN_ID` matched between robot and desktop if you set one manually.

If the network setup changes, run `ip -br addr` and use the interface that is `UP` on the same subnet as the robot.

## Quick start: Record D-LIO outputs (on robot)

Use this workflow when you want a compact replay bag that contains the online D-LIO outputs:
```bash
catmux_create_session /external/catmux/record_dlio.yaml
```

This session runs the same live robot-side stack as `online_dlio.yaml` and additionally records these topics for replay and visualization:
- `/dlio/odom_node/odom`
- `/dlio/odom_node/path`
- `/dlio/odom_node/keyframes`
- `/dlio/odom_node/pointcloud/deskewed`
- `/map`
- `/tf`
- `/tf_static`

Bags are saved to `/external/bags/dlio_YYYYMMDD_HHMMSS`.
Use this bag type with `bash scripts/dlio/playback.sh humble_ws/bags/dlio_YYYYMMDD_HHMMSS` for desktop replay.

## Quick start: Record raw sensor data

Record raw IMU + LiDAR data for offline processing with any algorithm:
```bash
catmux_create_session /external/catmux/record_raw.yaml
```
Bags are saved to `/external/bags/raw_YYYYMMDD_HHMMSS`.

## Quick start: Desktop replay of recorded D-LIO outputs

Use this workflow to replay a `dlio_YYYYMMDD_HHMMSS` bag recorded by `catmux/record_dlio.yaml`. The wrapper script in this repository is `scripts/dlio/playback.sh`.

1. Open this repository in VS Code and reopen it in the devcontainer
2. Once the container is ready, open an integrated terminal and run:
   ```bash
   bash scripts/dlio/playback.sh humble_ws/bags/dlio_YYYYMMDD_HHMMSS
   ```

RViz2 opens automatically alongside the bag player. The bag plays once. Close the RViz2 window or press `Ctrl+C` to stop both.

## Quick start: Desktop offline D-LIO reconstruction

1. Open this repository in VS Code and reopen it in the devcontainer.
2. Once the container is ready, run reconstruction:
   ```bash
   bash scripts/dlio/reconstruct_raw.sh <bag_directory>
   ```

Use this reconstruction flow for raw sensor bags such as `humble_ws/bags/raw_YYYYMMDD_HHMMSS`. Use `scripts/dlio/playback.sh` only for bags that already contain recorded D-LIO outputs such as `dlio_YYYYMMDD_HHMMSS`.

## Quick start: Desktop offline GLIM processing

1. Build the GLIM Docker image:
   ```bash
   bash scripts/glim/build_image.sh
   ```

2. Run GLIM on a raw bag:
   ```bash
   bash scripts/glim/run_glim_offline.sh --bag ~/ws/slam-go2w/humble_ws/bags/raw_YYYYMMDD_HHMMSS
   ```

   The command prints the output `results=` directory when it finishes.

3. Visualize a specific run by passing that result directory path:
   ```bash
   bash scripts/glim/visualize_glim_run.sh ~/ws/slam-go2w/output/results/<run_name>
   ```

   `--latest` is available if you just want to open the most recently modified run under `output/results/`, but the standard flow is to pass the exact run directory path you want to inspect.

## Catmux sessions

| Session | File | Purpose |
|---------|------|---------|
| D-LIO online | `catmux/online_dlio.yaml` | Online D-LIO (sensors + SLAM) |
| D-LIO record | `catmux/record_dlio.yaml` | Record D-LIO SLAM outputs |
| Raw record | `catmux/record_raw.yaml` | Record raw inputs for offline processing |
| D-LIO playback | `catmux/playback_dlio.yaml` | Replay recorded D-LIO outputs |

Create any of these tmux sessions with `catmux_create_session /external/<path-to-yaml>`.
For example, to record D-LIO outputs:
```bash
catmux_create_session /external/catmux/record_dlio.yaml
```
If a catmux session is already running and you only want to reconnect to it, use `catmux attach`.
For `catmux/playback_dlio.yaml`, playback stops after one pass.

## Sensor calibration

The shared calibration reference is at `config/sensor/go2w_calibration.yaml`.

Key values:
- **Extrinsic (base_link -> LiDAR)**: `[0.1634, 0.0, 0.116]`
- **IMU topic**: `/go2w/imu` (frame: `imu_link`)
- **LiDAR topic**: `/points_raw` (frame: `hesai_lidar`)
- **LiDAR IP**: `192.168.123.20`

Algorithm-specific configs contain their own copies in native formats.
Verify consistency against this reference when updating calibration.

## Adding a new SLAM algorithm

The layout is designed for easy extension:

1. Add source as a submodule to `humble_ws/src/<algorithm>/`
2. Create `config/<algorithm>/` for algorithm-specific parameters
3. Create `scripts/<algorithm>/` for offline processing workflows
4. Add `catmux/test_<algorithm>.yaml` for robot-side sessions
5. Optionally add `docker/desktop/<algorithm>/` for a special Docker image

The shared sensor infrastructure (submodules, raw recording, calibration, bag validation tools) is reused automatically.

## License

MIT License. See [LICENSE](LICENSE).
