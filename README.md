# slam-go2w

Unified SLAM repository for the Unitree GO2-W robot with Hesai PandarXT-16 LiDAR.

Choose your SLAM algorithm — currently supports **D-LIO** (online + offline) and **GLIM** (offline), with an extensible layout for adding more.

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

You can clone this repository into any directory. In the commands below, replace `/path/to/slam-go2w` with your local checkout path.

## Quick start: Online D-LIO (on robot)

1. Build the robot Docker image:
   ```bash
   cd /path/to/slam-go2w
   docker build -f docker/robot/Dockerfile -t go2w-humble:latest .
   ```

2. Start the container:
   ```bash
   bash docker/robot/run.sh
   ```

3. Inside the container, build the workspace:
   ```bash
   cd /external/humble_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Create and attach the D-LIO tmux session:
   ```bash
   catmux_create_session /external/catmux/test_dlio.yaml
   ```

Use `catmux_create_session` to start a session from a YAML config. The plain `catmux` command is only a wrapper around the dedicated tmux server that catmux uses, so it is useful for commands such as `catmux attach` after a session already exists.

This session sources `humble_ws/src/unitree_ros2/setup.sh`, which enables CycloneDDS on `eth0` and also `wlan0` when that interface exists on the robot host. Because `docker/robot/run.sh` starts the container with `--net=host`, the container shares the robot host network stack. If the robot host `wlan0` is configured on your WiFi SLAM network (for example `192.168.111.201`), the D-LIO topics are still published over WiFi from this containerized setup as well.

### Desktop RViz over WiFi

To view the live SLAM from a desktop PC on the same network, configure CycloneDDS on the desktop to use the PC-side network interface, then run RViz with the D-LIO config:

This native-host flow assumes you already built `humble_ws` locally and therefore have `humble_ws/install/setup.bash`. If you are using this repository's devcontainer for desktop visualization, skip this snippet and use `bash scripts/dlio/live_rviz.sh --iface <desktop_interface>` from inside the devcontainer instead.

```bash
source /opt/ros/humble/setup.bash
source /path/to/slam-go2w/humble_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="wlan0" priority="2" multicast="true" />
</Interfaces></General></Domain></CycloneDDS>'
rviz2 -d /path/to/slam-go2w/humble_ws/src/direct_lidar_inertial_odometry/launch/dlio.rviz
```

Replace `wlan0` with the actual desktop-side interface name if needed. On this desktop host, the matching interface is `enp97s0`, not `wlan0`. Keep `ROS_DOMAIN_ID` matched between robot and desktop if you set one manually.

If your desktop runs Ubuntu 24 and you do not want a native ROS 2 Humble install, use this repository's devcontainer instead. The devcontainer is based on `osrf/ros:humble-desktop` and already runs with host networking, which lets RViz join the same WiFi DDS traffic as the robot.

1. Open this repository in VS Code and reopen it in the devcontainer.
2. On the Ubuntu host, allow local Docker GUI access if needed:
   ```bash
   xhost +local:root
   ```
3. Inside the devcontainer, run:
   ```bash
   bash scripts/dlio/live_rviz.sh --iface enp97s0
   ```

On this desktop PC, `ip -br addr` shows `enp97s0` is `UP` on `192.168.111.100/24`, so `enp97s0` is the correct interface to use here. `wlan0` is the robot-side name that often appears in the online SLAM setup, but the desktop can use a different interface name entirely even when it is on the same subnet. You can re-check with:

```bash
ip -br addr
```

If the network setup changes, use the interface that is `UP` on the same subnet as the robot, for example `192.168.111.x`. This devcontainer flow is intended for desktop visualization only; the robot-side online SLAM stack still uses `docker/robot/run.sh`.

## Quick start: Record raw sensor data

Record raw IMU + LiDAR data for offline processing with any algorithm:
```bash
catmux_create_session /external/catmux/record_raw.yaml
```
Bags are saved to `/external/bags/raw_YYYYMMDD_HHMMSS`.

## Quick start: Desktop replay of recorded D-LIO outputs

Use this workflow to replay a bag that already contains recorded D-LIO outputs. The wrapper script in this repository is `scripts/dlio/playback.sh`.

1. Open this repository in VS Code and reopen it in the devcontainer
2. Once the container is ready, open an integrated terminal and run:
   ```bash
   bash scripts/dlio/playback.sh humble_ws/bags/slam_YYYYMMDD_HHMMSS
   ```

RViz2 opens automatically alongside the bag player. The bag loops continuously. Close the RViz2 window or press `Ctrl+C` to stop both.

## Quick start: Desktop offline D-LIO reconstruction

1. Open the devcontainer in VS Code (or build manually)
2. The `postCreate.sh` script builds D-LIO automatically
3. Run reconstruction:
   ```bash
   bash scripts/dlio/reconstruct_shared_raw.sh <bag_directory>
   ```

Use this reconstruction flow for raw sensor bags such as `humble_ws/bags/raw_YYYYMMDD_HHMMSS`. Use `scripts/dlio/playback.sh` only for bags that already contain recorded D-LIO outputs such as `slam_YYYYMMDD_HHMMSS`.

## Quick start: Desktop offline GLIM processing

1. Build the GLIM Docker image:
   ```bash
   bash scripts/glim/build_image.sh
   ```

2. Run GLIM on a raw bag:
   ```bash
   bash scripts/glim/run_glim_offline.sh --bag /path/to/raw_bag
   ```

   The command prints the output `results=` directory when it finishes.

3. Visualize a specific run by passing that result directory path:
   ```bash
   bash scripts/glim/visualize_glim_run.sh /path/to/output/results/<run_name>
   ```

   `--latest` is still available as a convenience shortcut, but the standard flow is to visualize the exact run path you want.

## Catmux sessions

| Session | File | Purpose |
|---------|------|---------|
| D-LIO test | `catmux/test_dlio.yaml` | Online D-LIO (sensors + SLAM) |
| D-LIO record | `catmux/record_dlio_output.yaml` | Record D-LIO SLAM outputs |
| Raw record | `catmux/record_raw.yaml` | Record raw inputs for offline processing |
| D-LIO playback | `catmux/playback_dlio.yaml` | Replay recorded D-LIO outputs |

Create any of these tmux sessions with `catmux_create_session /external/<path-to-yaml>`.
For example, to record D-LIO outputs:
```bash
catmux_create_session /external/catmux/record_dlio_output.yaml
```
If a catmux session is already running and you only want to reconnect to it, use `catmux attach`.
For `catmux/playback_dlio.yaml`, playback stops after one pass by default; set `bag_play_args: --loop` if you want continuous replay.

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
