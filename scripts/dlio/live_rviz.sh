#!/bin/bash
# Launch RViz2 for live D-LIO visualization from a desktop-side Humble container.
#
# Usage:
#   bash scripts/dlio/live_rviz.sh [--iface <network_interface>] [--config <rviz_config>]
#
# Example:
#   bash scripts/dlio/live_rviz.sh --iface wlan0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG_DEFAULT="$REPO_ROOT/config/dlio/dlio.rviz"
IFACE_DEFAULT="${CYCLONEDDS_IFACE:-wlan0}"

iface="$IFACE_DEFAULT"
rviz_cfg="$RVIZ_CFG_DEFAULT"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --iface)
            iface="${2:?Error: --iface requires a value}"
            shift 2
            ;;
        --config)
            rviz_cfg="${2:?Error: --config requires a value}"
            shift 2
            ;;
        -h|--help)
            sed -n '2,8p' "$0"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/humble/setup.bash
fi

desktop_setup="$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash"
if [ -f "$desktop_setup" ]; then
    # The postCreate step builds D-LIO once in the desktop container.
    source "$desktop_setup"
fi

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "Error: rviz2 not found in PATH" >&2
    exit 1
fi

if [ ! -f "$rviz_cfg" ]; then
    echo "Error: RViz config not found: $rviz_cfg" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "Error: DISPLAY is not set. Start this from a desktop session or fix X11 forwarding." >&2
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"$iface\" priority=\"2\" multicast=\"true\" />
</Interfaces></General></Domain></CycloneDDS>"

echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "CycloneDDS interface: $iface"
echo "RViz config: $rviz_cfg"

rviz2 -d "$rviz_cfg"
