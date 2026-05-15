#!/bin/bash
# Play a recorded LIO-SAM bag and open RViz2 in one command.

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --loop)
            echo "Error: --loop is not supported in this playback helper." >&2
            exit 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            ;;
    esac
    shift
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO_ROOT/config/liosam/liosam.rviz"
ROS_SETUP="/opt/ros/humble/setup.bash"
DESKTOP_SETUP="$REPO_ROOT/.devcontainer/offline_liosam/install/setup.bash"

source_setup_safely() {
    local setup_script="$1"
    local restore_nounset=0
    local rc=0

    case $- in
        *u*)
            restore_nounset=1
            set +u
            ;;
    esac

    export COLCON_TRACE="${COLCON_TRACE-}"
    source "$setup_script" || rc=$?

    if [ "$restore_nounset" -eq 1 ]; then
        set -u
    fi

    return "$rc"
}

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

if ! command -v ros2 >/dev/null 2>&1 || ! command -v rviz2 >/dev/null 2>&1; then
    source_setup_safely "$ROS_SETUP"
fi

if [ -f "$DESKTOP_SETUP" ]; then
    source_setup_safely "$DESKTOP_SETUP"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found in PATH after sourcing $ROS_SETUP" >&2
    exit 1
fi

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "Error: rviz2 not found in PATH after sourcing $ROS_SETUP" >&2
    exit 1
fi

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo ""

PLAY_ARGS=(--clock)
PLAY_ARGS+=("${EXTRA_ARGS[@]}")
ros2 bag play "$BAG" "${PLAY_ARGS[@]}" &
BAG_PID=$!

trap "echo 'Stopping bag player...'; kill $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null" EXIT

sleep 2
rviz2 -d "$RVIZ_CFG"
