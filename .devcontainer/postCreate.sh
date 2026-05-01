#!/bin/bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
DLIO_SRC="$REPO_ROOT/humble_ws/src/direct_lidar_inertial_odometry"
DESKTOP_WS_ROOT="$REPO_ROOT/.devcontainer/offline_dlio"
DESKTOP_INSTALL="$DESKTOP_WS_ROOT/install"
FASTLIO_SRC="$REPO_ROOT/humble_ws/src/fast_lio_ros2"
DESKTOP_FASTLIO_WS_ROOT="$REPO_ROOT/.devcontainer/offline_fastlio"
DESKTOP_FASTLIO_INSTALL="$DESKTOP_FASTLIO_WS_ROOT/install"

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -d "$DLIO_SRC" ]; then
    echo "Error: D-LIO source not found: $DLIO_SRC" >&2
    echo "Did you clone submodules? Run: git submodule update --init --recursive" >&2
    exit 1
fi

bash "$REPO_ROOT/.devcontainer/configure_git_safe_directory.sh"

source "$ROS_SETUP"

mkdir -p "$DESKTOP_WS_ROOT"
rm -rf "$DESKTOP_WS_ROOT/build" "$DESKTOP_WS_ROOT/install" "$DESKTOP_WS_ROOT/log"

colcon --log-base "$DESKTOP_WS_ROOT/log" build \
    --symlink-install \
    --base-paths "$DLIO_SRC" \
    --build-base "$DESKTOP_WS_ROOT/build" \
    --install-base "$DESKTOP_INSTALL" \
    --packages-select direct_lidar_inertial_odometry

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -qxF "source $DESKTOP_INSTALL/setup.bash" ~/.bashrc || \
    echo "source $DESKTOP_INSTALL/setup.bash" >> ~/.bashrc

echo "Desktop offline D-LIO environment is ready."
echo "Installed setup: $DESKTOP_INSTALL/setup.bash"

# FAST-LIO is optional: build only when the submodule has been pulled in.
if [ -d "$FASTLIO_SRC" ]; then
    mkdir -p "$DESKTOP_FASTLIO_WS_ROOT"
    rm -rf "$DESKTOP_FASTLIO_WS_ROOT/build" "$DESKTOP_FASTLIO_INSTALL" "$DESKTOP_FASTLIO_WS_ROOT/log"

    colcon --log-base "$DESKTOP_FASTLIO_WS_ROOT/log" build \
        --symlink-install \
        --base-paths "$FASTLIO_SRC" \
        --build-base "$DESKTOP_FASTLIO_WS_ROOT/build" \
        --install-base "$DESKTOP_FASTLIO_INSTALL" \
        --packages-select fast_lio

    grep -qxF "source $DESKTOP_FASTLIO_INSTALL/setup.bash" ~/.bashrc || \
        echo "source $DESKTOP_FASTLIO_INSTALL/setup.bash" >> ~/.bashrc

    echo "Desktop offline FAST-LIO environment is ready."
    echo "Installed setup: $DESKTOP_FASTLIO_INSTALL/setup.bash"
else
    echo "Skipping FAST-LIO build: submodule not present at $FASTLIO_SRC."
    echo "  Run 'git submodule update --init --recursive' once the submodule is added, then rerun this script."
fi
