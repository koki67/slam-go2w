#!/bin/bash
# Reconstruct LIO-SAM outputs from a raw sensor bag and open RViz2.

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ_CFG="$REPO_ROOT/config/liosam/liosam.rviz"
PARAMS_FILE="$REPO_ROOT/config/liosam/go2w.yaml"
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/.devcontainer/offline_liosam/install/setup.bash"
    "$REPO_ROOT/humble_ws/install/setup.bash"
)

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then
    BAG="$REPO_ROOT/$BAG"
fi

if [ ! -d "$BAG" ]; then
    echo "Error: bag directory not found: $BAG" >&2
    exit 1
fi

if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Error: metadata.yaml not found in bag directory: $BAG" >&2
    exit 1
fi

bag_topics=$(python3 -c "
import yaml, sys
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
info = d.get('rosbag2_bagfile_information', {})
for t in info.get('topics_with_message_count', []):
    print(t['topic_metadata']['name'])
" "$BAG/metadata.yaml" 2>/dev/null) \
    || { echo "Error: failed to parse $BAG/metadata.yaml" >&2; exit 1; }

for required in /points_raw /go2w/imu; do
    if ! grep -qxF "$required" <<< "$bag_topics"; then
        echo "Error: bag is missing required topic: $required" >&2
        echo "Topics found in bag:" >&2
        while IFS= read -r t; do echo "  $t" >&2; done <<< "$bag_topics"
        exit 1
    fi
done

CLOCK_ARG=(--clock)
if grep -qxF "/clock" <<< "$bag_topics"; then
    CLOCK_ARG=()
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ -f "$candidate" ]; then
        WS_SETUP="$candidate"
        break
    fi
done

if [ -z "$WS_SETUP" ]; then
    echo "Error: no LIO-SAM workspace setup was found." >&2
    echo "Looked for:" >&2
    for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
        echo "  - $candidate" >&2
    done
    echo "Create the desktop devcontainer or build the workspace first, then rerun this script." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix lio_sam >/dev/null 2>&1; then
    echo "Error: lio_sam is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    if [ -f "$REPO_ROOT/.devcontainer/postCreate.sh" ]; then
        echo "In the desktop devcontainer, rerun: bash .devcontainer/postCreate.sh" >&2
    fi
    exit 1
fi

cleanup() {
    echo "Stopping raw-bag LIO-SAM reconstruction..."
    for pid in "${BAG_PID:-}" "${RELAY_PID:-}" "${STATIC_TF_PIDS[@]:-}" "${LIOSAM_PIDS[@]:-}"; do
        if [ -n "${pid:-}" ]; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    for _ in 1 2 3 4 5; do
        local any_alive=0
        for pid in "${BAG_PID:-}" "${RELAY_PID:-}" "${STATIC_TF_PIDS[@]:-}" "${LIOSAM_PIDS[@]:-}"; do
            if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        [ "$any_alive" -eq 0 ] && break
        sleep 1
    done
    for pid in "${BAG_PID:-}" "${RELAY_PID:-}" "${STATIC_TF_PIDS[@]:-}" "${LIOSAM_PIDS[@]:-}"; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
    pkill -KILL -x lio_sam_imuPreintegration 2>/dev/null || true
    pkill -KILL -x lio_sam_imageProjection 2>/dev/null || true
    pkill -KILL -x lio_sam_featureExtraction 2>/dev/null || true
    pkill -KILL -x lio_sam_mapOptimization 2>/dev/null || true
    pkill -KILL -f hesai_to_velodyne.py 2>/dev/null || true
}

trap cleanup EXIT INT TERM

for proc in lio_sam_imuPreintegration lio_sam_imageProjection lio_sam_featureExtraction lio_sam_mapOptimization; do
    if pgrep -x "$proc" >/dev/null 2>&1; then
        echo "Found leftover $proc from a previous run; terminating it..."
        pkill -INT -x "$proc" 2>/dev/null || true
    fi
done
pkill -INT -f hesai_to_velodyne.py 2>/dev/null || true
sleep 1
for proc in lio_sam_imuPreintegration lio_sam_imageProjection lio_sam_featureExtraction lio_sam_mapOptimization; do
    pkill -KILL -x "$proc" 2>/dev/null || true
done
pkill -KILL -f hesai_to_velodyne.py 2>/dev/null || true

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "LIO-SAM setup: $WS_SETUP"
echo "Mode: replay raw topics, run Hesai relay + LIO-SAM offline, visualize generated outputs"
echo ""

python3 "$REPO_ROOT/scripts/liosam/hesai_to_velodyne.py" &
RELAY_PID=$!

STATIC_TF_PIDS=()
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom &
STATIC_TF_PIDS+=("$!")
ros2 run tf2_ros static_transform_publisher 0.1634 0 0.116 0 0 0.7071068 0.7071068 base_link hesai_lidar &
STATIC_TF_PIDS+=("$!")

LIOSAM_PIDS=()
for executable in \
    lio_sam_imuPreintegration \
    lio_sam_imageProjection \
    lio_sam_featureExtraction \
    lio_sam_mapOptimization; do
    ros2 run lio_sam "$executable" --ros-args --params-file "$PARAMS_FILE" -p use_sim_time:=true &
    LIOSAM_PIDS+=("$!")
done

sleep 3
for pid in "$RELAY_PID" "${STATIC_TF_PIDS[@]}" "${LIOSAM_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "Error: LIO-SAM stack exited during startup." >&2
        wait "$pid" || true
        exit 1
    fi
done

ros2 bag play "$BAG" "${CLOCK_ARG[@]}" "${EXTRA_ARGS[@]}" &
BAG_PID=$!

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
