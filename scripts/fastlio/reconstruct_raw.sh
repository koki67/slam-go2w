#!/bin/bash
# Reconstruct FAST-LIO outputs from a raw sensor bag and open RViz2.
#
# Usage (from anywhere inside the repository):
#   bash scripts/fastlio/reconstruct_raw.sh <bag_directory> [ros2 bag play args...]
#
# Examples:
#   bash scripts/fastlio/reconstruct_raw.sh humble_ws/bags/raw_20260312_024403
#   bash scripts/fastlio/reconstruct_raw.sh humble_ws/bags/raw_20260312_024403 --rate 2.0

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ_CFG="$REPO_ROOT/config/fastlio/fastlio.rviz"
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/.devcontainer/offline_fastlio/install/setup.bash"
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

# Parse topic list once; used for validation and --clock detection below.
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

# Add --clock only when the bag does not already contain /clock messages.
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
    echo "Error: no FAST-LIO workspace setup was found." >&2
    echo "Looked for:" >&2
    for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
        echo "  - $candidate" >&2
    done
    echo "Create the desktop devcontainer or build the workspace first, then rerun this script." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix fast_lio >/dev/null 2>&1; then
    echo "Error: fast_lio is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    if [ -f "$REPO_ROOT/.devcontainer/postCreate.sh" ]; then
        echo "In the desktop devcontainer, rerun: bash .devcontainer/postCreate.sh" >&2
    fi
    exit 1
fi

cleanup() {
    echo "Stopping raw-bag FAST-LIO reconstruction..."
    # SIGINT first so ros2 launch can propagate a graceful shutdown to its children.
    for pid in "${BAG_PID:-}" "${FASTLIO_PID:-}"; do
        if [ -n "${pid:-}" ]; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    # Wait up to 5 seconds for graceful exit.
    for _ in 1 2 3 4 5; do
        local any_alive=0
        for pid in "${BAG_PID:-}" "${FASTLIO_PID:-}"; do
            if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        [ "$any_alive" -eq 0 ] && break
        sleep 1
    done
    # Force-kill any direct child still alive.
    for pid in "${BAG_PID:-}" "${FASTLIO_PID:-}"; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
    # ros2 launch sometimes orphans its node children on SIGTERM/quick exit;
    # without this, leftover fastlio_mapping processes keep publishing the previous
    # run's accumulated map and corrupt the next reconstruction.
    pkill -KILL -x fastlio_mapping 2>/dev/null || true
}

trap cleanup EXIT INT TERM

# Defensive sweep: if a previous invocation orphaned FAST-LIO nodes (e.g., the shell
# was killed before its EXIT trap could run), clear them out before launching.
if pgrep -x fastlio_mapping >/dev/null 2>&1; then
    echo "Found leftover FAST-LIO nodes from a previous run; terminating them..."
    pkill -INT -x fastlio_mapping 2>/dev/null || true
    for _ in 1 2 3; do
        if ! pgrep -x fastlio_mapping >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done
    pkill -KILL -x fastlio_mapping 2>/dev/null || true
fi

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "FAST-LIO setup: $WS_SETUP"
echo "Mode: replay raw topics, run FAST-LIO offline, visualize generated outputs"
echo ""

ros2 launch fast_lio mapping.launch.py \
    rviz:=false \
    use_sim_time:=true \
    config_file:=go2w.yaml \
    pointcloud_topic:=points_raw \
    imu_topic:=go2w/imu &
FASTLIO_PID=$!

sleep 3
if ! kill -0 "$FASTLIO_PID" 2>/dev/null; then
    echo "Error: FAST-LIO exited during startup." >&2
    wait "$FASTLIO_PID" || true
    exit 1
fi

# Reset FAST-LIO state before playback to clear any leftover data from previous runs.
# --timeout prevents hanging indefinitely if a node didn't come up cleanly.
echo "Resetting FAST-LIO node state..."
ros2 service call /laserMapping/reset std_srvs/srv/Trigger --timeout 5 || true

ros2 bag play "$BAG" "${CLOCK_ARG[@]}" "${EXTRA_ARGS[@]}" &
BAG_PID=$!

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
