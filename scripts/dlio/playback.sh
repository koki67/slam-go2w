#!/bin/bash
# Play a recorded SLAM bag and open RViz2 in one command.
#
# Usage (from anywhere inside the workspace):
#   bash scripts/dlio/playback.sh <bag_directory> [--rate <speed>]
#
# Examples:
#   bash scripts/dlio/playback.sh humble_ws/bags/slam_20250301_143022
#   bash scripts/dlio/playback.sh humble_ws/bags/slam_20250301_143022 --rate 2.0

set -e

BAG="${1:?Error: bag path required.  Usage: $0 <bag_directory> [--rate <speed>]}"
shift
EXTRA_ARGS="$@"   # any additional flags, e.g. --rate 2.0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO_ROOT/config/dlio/dlio.rviz"

# Source ROS 2 if not already sourced.
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo ""

# Start bag player in the background.
ros2 bag play "$BAG" --clock --loop $EXTRA_ARGS &
BAG_PID=$!

# When this script exits (rviz2 closed, Ctrl+C, or any error),
# kill the bag player so it does not linger in the background.
trap "echo 'Stopping bag player...'; kill $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null" EXIT

# Give the bag player a moment to start publishing before RViz2 opens.
sleep 2

# Run RViz2 in the foreground; the script ends when the window is closed.
rviz2 -d "$RVIZ_CFG"
