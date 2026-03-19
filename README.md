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
git clone --recurse-submodules https://github.com/koki67/slam-go2w.git ~/ws/slam-go2w
```

## Quick start: Online D-LIO (on robot)

1. Build the robot Docker image:
   ```bash
   cd ~/ws/slam-go2w
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

4. Run D-LIO with catmux:
   ```bash
   catmux /external/catmux/test_dlio.yaml
   ```

## Quick start: Record raw sensor data

Record raw IMU + LiDAR data for offline processing with any algorithm:
```bash
catmux /external/catmux/record_raw.yaml
```
Bags are saved to `/external/bags/raw_YYYYMMDD_HHMMSS`.

## Quick start: Desktop offline D-LIO reconstruction

1. Open the devcontainer in VS Code (or build manually)
2. The `postCreate.sh` script builds D-LIO automatically
3. Run reconstruction:
   ```bash
   bash scripts/dlio/reconstruct_shared_raw.sh <bag_directory>
   ```

## Quick start: Desktop offline GLIM processing

1. Build the GLIM Docker image:
   ```bash
   bash scripts/glim/build_image.sh
   ```

2. Run GLIM on a raw bag:
   ```bash
   bash scripts/glim/run_glim_offline.sh --bag /path/to/raw_bag
   ```

3. Visualize results:
   ```bash
   bash scripts/glim/visualize_glim_run.sh --latest
   ```

## Catmux sessions

| Session | File | Purpose |
|---------|------|---------|
| D-LIO test | `catmux/test_dlio.yaml` | Online D-LIO (sensors + SLAM) |
| D-LIO record | `catmux/record_dlio_output.yaml` | Record D-LIO SLAM outputs |
| Raw record | `catmux/record_raw.yaml` | Record raw inputs for offline processing |
| D-LIO playback | `catmux/playback_dlio.yaml` | Replay recorded D-LIO outputs |

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
