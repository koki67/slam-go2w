#!/bin/bash
# Play a recorded wheel-legged odometry output bag and open RViz2.
#
# Usage:
#   bash scripts/wheel_legged_odometry/playback.sh <bag_directory> [ros2 bag play args...]

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO_ROOT/config/wheel_legged_odometry/wheel_legged_odometry.rviz"
ROS_SETUP="/opt/ros/humble/setup.bash"

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

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then BAG="$REPO_ROOT/$BAG"; fi
if [ ! -d "$BAG" ]; then echo "Error: bag directory not found: $BAG" >&2; exit 1; fi
if [ ! -f "$BAG/metadata.yaml" ]; then echo "Error: metadata.yaml not found: $BAG" >&2; exit 1; fi
if [ ! -f "$ROS_SETUP" ]; then echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2; exit 1; fi

source_setup_safely "$ROS_SETUP"
for setup in "$REPO_ROOT/humble_ws/install/setup.bash" \
             "$REPO_ROOT/.devcontainer/offline_dgkilo/install/setup.bash"; do
    if [ -f "$setup" ]; then source_setup_safely "$setup"; fi
done

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"

ros2 bag play "$BAG" --clock "${EXTRA_ARGS[@]}" &
BAG_PID=$!
trap "echo 'Stopping bag player...'; kill $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null" EXIT

sleep 2
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
