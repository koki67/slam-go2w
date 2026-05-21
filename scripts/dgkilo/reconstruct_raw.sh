#!/bin/bash
# Reconstruct DG-KILO outputs from a raw legged sensor bag and open RViz2.
#
# Usage (from anywhere inside the repository):
#   bash scripts/dgkilo/reconstruct_raw.sh <bag_directory> [ros2 bag play args...]
#
# The input bag MUST contain /points_raw, /go2w/imu, AND /lowstate.
# Use catmux/record_raw_legged.yaml to capture a compatible bag.
# Plain raw bags (record_raw.yaml, no /lowstate) will be rejected.
#
# Examples:
#   bash scripts/dgkilo/reconstruct_raw.sh humble_ws/bags/raw_legged_20260521_120000
#   bash scripts/dgkilo/reconstruct_raw.sh humble_ws/bags/raw_legged_20260521_120000 --rate 2.0

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ_CFG="$REPO_ROOT/config/dgkilo/dg_kilo.rviz"
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/.devcontainer/offline_dgkilo/install/setup.bash"
    "$REPO_ROOT/humble_ws/install/setup.bash"
)

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then BAG="$REPO_ROOT/$BAG"; fi

if [ ! -d "$BAG" ]; then
    echo "Error: bag directory not found: $BAG" >&2; exit 1
fi

if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Error: metadata.yaml not found in bag directory: $BAG" >&2; exit 1
fi

# Parse topic list for validation
bag_topics=$(python3 -c "
import yaml, sys
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
info = d.get('rosbag2_bagfile_information', {})
for t in info.get('topics_with_message_count', []):
    print(t['topic_metadata']['name'])
" "$BAG/metadata.yaml" 2>/dev/null) \
    || { echo "Error: failed to parse $BAG/metadata.yaml" >&2; exit 1; }

# Validate required topics (DG-KILO requires /lowstate in addition to LiDAR+IMU)
for required in /points_raw /go2w/imu /lowstate; do
    if ! grep -qxF "$required" <<< "$bag_topics"; then
        echo "Error: bag is missing required topic: $required" >&2
        if [ "$required" = "/lowstate" ]; then
            echo "DG-KILO requires /lowstate for leg kinematics." >&2
            echo "Record a new bag with: catmux_create_session /external/catmux/record_raw_legged.yaml" >&2
        fi
        echo "Topics found in bag:" >&2
        while IFS= read -r t; do echo "  $t" >&2; done <<< "$bag_topics"
        exit 1
    fi
done

# Add --clock only when not already in bag
CLOCK_ARG=(--clock)
if grep -qxF "/clock" <<< "$bag_topics"; then CLOCK_ARG=(); fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2; exit 1
fi

for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ -f "$candidate" ]; then WS_SETUP="$candidate"; break; fi
done

if [ -z "$WS_SETUP" ]; then
    echo "Error: no DG-KILO workspace setup was found." >&2
    echo "Looked for:" >&2
    for c in "${WS_SETUP_CANDIDATES[@]}"; do echo "  - $c" >&2; done
    echo "Create the offline_dgkilo devcontainer or build the workspace first." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix dg_kilo >/dev/null 2>&1; then
    echo "Error: dg_kilo is not available after sourcing $WS_SETUP" >&2
    if [ -f "$REPO_ROOT/.devcontainer/postCreate.sh" ]; then
        echo "In the offline_dgkilo devcontainer, rerun: bash .devcontainer/postCreate.sh" >&2
    fi
    exit 1
fi

cleanup() {
    echo "Stopping DG-KILO reconstruction..."
    for pid in "${BAG_PID:-}" "${DGKILO_PID:-}"; do
        if [ -n "${pid:-}" ]; then kill -INT "$pid" 2>/dev/null || true; fi
    done
    for _ in 1 2 3 4 5; do
        local any_alive=0
        for pid in "${BAG_PID:-}" "${DGKILO_PID:-}"; do
            if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then any_alive=1; break; fi
        done
        [ "$any_alive" -eq 0 ] && break; sleep 1
    done
    for pid in "${BAG_PID:-}" "${DGKILO_PID:-}"; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
        if [ -n "${pid:-}" ]; then wait "$pid" 2>/dev/null || true; fi
    done
    pkill -KILL -x dg_kilo_node 2>/dev/null || true
}

trap cleanup EXIT INT TERM

if pgrep -x dg_kilo_node >/dev/null 2>&1; then
    echo "Found leftover DG-KILO nodes from a previous run; terminating them..."
    pkill -INT -x dg_kilo_node 2>/dev/null || true
    for _ in 1 2 3; do
        if ! pgrep -x dg_kilo_node >/dev/null 2>&1; then break; fi
        sleep 1
    done
    pkill -KILL -x dg_kilo_node 2>/dev/null || true
fi

echo "Bag:     $BAG"
echo "RViz:    $RVIZ_CFG"
echo "Setup:   $WS_SETUP"
echo "Mode:    replay raw legged topics, run DG-KILO offline, visualize outputs"
echo "Diags:   /tmp/dg_kilo_diag.csv"
echo ""

ros2 launch dg_kilo dg_kilo.launch.py \
    rviz:=false \
    use_sim_time:=true \
    config_path:="$REPO_ROOT/config/dgkilo" \
    config_file:=go2w.yaml \
    pointcloud_topic:=/points_raw \
    imu_topic:=/go2w/imu \
    lowstate_topic:=lowstate &
DGKILO_PID=$!

sleep 3
if ! kill -0 "$DGKILO_PID" 2>/dev/null; then
    echo "Error: DG-KILO exited during startup." >&2
    wait "$DGKILO_PID" || true
    exit 1
fi

ros2 bag play "$BAG" "${CLOCK_ARG[@]}" "${EXTRA_ARGS[@]}" &
BAG_PID=$!

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then wait "$BAG_PID"; fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
