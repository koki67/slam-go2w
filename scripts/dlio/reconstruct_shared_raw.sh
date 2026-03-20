#!/bin/bash
# Reconstruct D-LIO outputs from a raw sensor bag and open RViz2.
#
# Usage (from anywhere inside the repository):
#   bash scripts/dlio/reconstruct_shared_raw.sh <bag_directory> [--world-only] [ros2 bag play args...]
#
# Examples:
#   bash scripts/dlio/reconstruct_shared_raw.sh humble_ws/bags/raw_20260312_024403
#   bash scripts/dlio/reconstruct_shared_raw.sh humble_ws/bags/raw_20260312_024403 --rate 2.0
#   bash scripts/dlio/reconstruct_shared_raw.sh humble_ws/bags/raw_20260312_024403 --world-only

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [--world-only] [ros2 bag play args...]}"
shift
WORLD_ONLY=false
EXTRA_ARGS=()

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ_CFG="$REPO_ROOT/config/dlio/dlio.rviz"
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash"
    "$REPO_ROOT/humble_ws/install/setup.bash"
)

while [ $# -gt 0 ]; do
    case "$1" in
        --world-only)
            WORLD_ONLY=true
            ;;
        *)
            EXTRA_ARGS+=("$1")
            ;;
    esac
    shift
done

if [ "$WORLD_ONLY" = true ]; then
    RVIZ_CFG="$REPO_ROOT/config/dlio/dlio_world_only.rviz"
fi

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
    echo "Error: no D-LIO workspace setup was found." >&2
    echo "Looked for:" >&2
    for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
        echo "  - $candidate" >&2
    done
    echo "Create the desktop devcontainer or build the workspace first, then rerun this script." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1; then
    echo "Error: direct_lidar_inertial_odometry is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    if [ -f "$REPO_ROOT/.devcontainer/postCreate.sh" ]; then
        echo "In the desktop devcontainer, rerun: bash .devcontainer/postCreate.sh" >&2
    fi
    exit 1
fi

cleanup() {
    echo "Stopping shared raw D-LIO reconstruction..."
    for pid in "${BAG_PID:-}" "${DLIO_PID:-}"; do
        if [ -n "${pid:-}" ]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${BAG_PID:-}" "${DLIO_PID:-}"; do
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT INT TERM

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "D-LIO setup: $WS_SETUP"
echo "Mode: replay shared raw topics, run D-LIO offline, visualize generated outputs"
echo ""

ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=false \
    use_sim_time:=true \
    pointcloud_topic:=points_raw \
    imu_topic:=go2w/imu &
DLIO_PID=$!

sleep 3
if ! kill -0 "$DLIO_PID" 2>/dev/null; then
    wait "$DLIO_PID"
fi

ros2 bag play "$BAG" --clock "${EXTRA_ARGS[@]}" &
BAG_PID=$!

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
