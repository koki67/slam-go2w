#!/bin/bash
# Reconstruct wheel-legged odometry outputs from a raw lowstate bag and open RViz2.
#
# Usage:
#   bash scripts/wheel_legged_odometry/reconstruct_raw.sh <bag_directory> [ros2 bag play args...]
#
# The input bag must contain /lowstate. /points_raw is optional and is used only
# for visualization when present.

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO_ROOT/config/wheel_legged_odometry/wheel_legged_odometry.rviz"

source_setup_safely() {
    local setup_script="$1"
    local restore_nounset=0
    local rc=0
    case $- in *u*) restore_nounset=1; set +u ;; esac
    export COLCON_TRACE="${COLCON_TRACE-}"
    # shellcheck source=/dev/null
    source "$setup_script" || rc=$?
    if [ "$restore_nounset" -eq 1 ]; then set -u; fi
    return "$rc"
}

setup_matches_ros_distro() {
    local setup_script="$1"
    if grep -q "/opt/ros/" "$setup_script" 2>/dev/null && ! grep -q "/opt/ros/${ros_distro}" "$setup_script"; then
        return 1
    fi
    return 0
}

build_wheel_odom_overlay() {
    if [ ! -d "$WHEEL_ODOM_SRC" ]; then
        echo "Error: wheel_legged_odometry source not found: $WHEEL_ODOM_SRC" >&2
        return 1
    fi

    echo "Building wheel_legged_odometry Humble overlay: $WHEEL_ODOM_INSTALL"
    mkdir -p "$WHEEL_ODOM_WS_ROOT"
    rm -rf "$WHEEL_ODOM_WS_ROOT/build" "$WHEEL_ODOM_INSTALL" "$WHEEL_ODOM_WS_ROOT/log"

    colcon --log-base "$WHEEL_ODOM_WS_ROOT/log" build \
        --symlink-install \
        --base-paths "$REPO_ROOT/humble_ws/src" \
        --build-base "$WHEEL_ODOM_WS_ROOT/build" \
        --install-base "$WHEEL_ODOM_INSTALL" \
        --packages-up-to wheel_legged_odometry \
        --cmake-args -DBUILD_TESTING=OFF
}
WS_SETUP=""
WHEEL_ODOM_WS_ROOT="$REPO_ROOT/.devcontainer/offline_wheel_legged_odometry"
WHEEL_ODOM_INSTALL="$WHEEL_ODOM_WS_ROOT/install"
WHEEL_ODOM_SRC="$REPO_ROOT/humble_ws/src/wheel_legged_odometry"
WS_SETUP_CANDIDATES=(
    "$WHEEL_ODOM_INSTALL/setup.bash"
    "$REPO_ROOT/humble_ws/install/setup.bash"
)

# Resolve ROS setup path from ROS_DISTRO (or fall back to humble).
ros_distro="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ros_distro}/setup.bash"

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then BAG="$REPO_ROOT/$BAG"; fi
if [ ! -d "$BAG" ]; then echo "Error: bag directory not found: $BAG" >&2; exit 1; fi
if [ ! -f "$BAG/metadata.yaml" ]; then echo "Error: metadata.yaml not found in bag: $BAG" >&2; exit 1; fi

bag_topics=$(python3 -c "
import yaml, sys
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
info = d.get('rosbag2_bagfile_information', {})
for t in info.get('topics_with_message_count', []):
    print(t['topic_metadata']['name'])
" "$BAG/metadata.yaml" 2>/dev/null) \
    || { echo "Error: failed to parse $BAG/metadata.yaml" >&2; exit 1; }

if ! grep -qxF "/lowstate" <<< "$bag_topics"; then
    echo "Error: bag is missing required topic: /lowstate" >&2
    echo "Topics found in bag:" >&2
    while IFS= read -r t; do echo "  $t" >&2; done <<< "$bag_topics"
    exit 1
fi

CLOCK_ARG=(--clock)
if grep -qxF "/clock" <<< "$bag_topics"; then CLOCK_ARG=(); fi

if [ ! -f "$ROS_SETUP" ]; then echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2; exit 1; fi

source_setup_safely "$ROS_SETUP"

for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ ! -f "$candidate" ]; then
        continue
    fi
    if ! setup_matches_ros_distro "$candidate"; then
        echo "Skipping stale workspace setup for another ROS distro: $candidate" >&2
        continue
    fi
    if source_setup_safely "$candidate" && ros2 pkg prefix wheel_legged_odometry >/dev/null 2>&1; then
        WS_SETUP="$candidate"
        break
    fi
done

if [ -z "$WS_SETUP" ]; then
    build_wheel_odom_overlay
    WS_SETUP="$WHEEL_ODOM_INSTALL/setup.bash"
    source_setup_safely "$WS_SETUP"
fi

if ! ros2 pkg prefix wheel_legged_odometry >/dev/null 2>&1; then
    echo "Error: wheel_legged_odometry is not available after sourcing $WS_SETUP" >&2
    exit 1
fi

cleanup() {
    echo "Stopping wheel-legged odometry reconstruction..."
    for pid in "${BAG_PID:-}" "${ODOM_PID:-}"; do
        if [ -n "${pid:-}" ]; then kill -INT "$pid" 2>/dev/null || true; fi
    done
    for pid in "${BAG_PID:-}" "${ODOM_PID:-}"; do
        if [ -n "${pid:-}" ]; then wait "$pid" 2>/dev/null || true; fi
    done
}
trap cleanup EXIT INT TERM

echo "Bag:   $BAG"
echo "RViz:  $RVIZ_CFG"
echo "Setup: $WS_SETUP"

ros2 launch wheel_legged_odometry wheel_legged_odometry.launch.py \
    rviz:=false \
    use_sim_time:=true \
    config_path:="$REPO_ROOT/config/wheel_legged_odometry" \
    config_file:=go2w.yaml \
    lowstate_topic:=/lowstate &
ODOM_PID=$!

sleep 2
ros2 bag play "$BAG" "${CLOCK_ARG[@]}" "${EXTRA_ARGS[@]}" &
BAG_PID=$!

sleep 2
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
