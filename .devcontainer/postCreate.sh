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
DGKILO_SRC="$REPO_ROOT/humble_ws/src/dg_kilo"
DESKTOP_DGKILO_WS_ROOT="$REPO_ROOT/.devcontainer/offline_dgkilo"
DESKTOP_DGKILO_INSTALL="$DESKTOP_DGKILO_WS_ROOT/install"

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
    --packages-select direct_lidar_inertial_odometry \
    --cmake-args -DBUILD_TESTING=OFF

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
        --packages-select fast_lio \
        --cmake-args -DBUILD_TESTING=OFF

    grep -qxF "source $DESKTOP_FASTLIO_INSTALL/setup.bash" ~/.bashrc || \
        echo "source $DESKTOP_FASTLIO_INSTALL/setup.bash" >> ~/.bashrc

    echo "Desktop offline FAST-LIO environment is ready."
    echo "Installed setup: $DESKTOP_FASTLIO_INSTALL/setup.bash"
else
    echo "Skipping FAST-LIO build: submodule not present at $FASTLIO_SRC."
    echo "  Run 'git submodule update --init --recursive' once the submodule is added, then rerun this script."
fi

# DG-KILO is optional: build only when the source directory is present.
# Requires GTSAM 4.2 (built from source in Dockerfile) — disk budget: +~300 MB.
if [ -d "$DGKILO_SRC" ]; then
    mkdir -p "$DESKTOP_DGKILO_WS_ROOT"
    rm -rf "$DESKTOP_DGKILO_WS_ROOT/build" "$DESKTOP_DGKILO_INSTALL" "$DESKTOP_DGKILO_WS_ROOT/log"

    colcon --log-base "$DESKTOP_DGKILO_WS_ROOT/log" build \
        --symlink-install \
        --base-paths "$REPO_ROOT/humble_ws/src" \
        --build-base "$DESKTOP_DGKILO_WS_ROOT/build" \
        --install-base "$DESKTOP_DGKILO_INSTALL" \
        --packages-up-to dg_kilo \
        --cmake-args -DBUILD_TESTING=OFF

    grep -qxF "source $DESKTOP_DGKILO_INSTALL/setup.bash" ~/.bashrc || \
        echo "source $DESKTOP_DGKILO_INSTALL/setup.bash" >> ~/.bashrc

    echo "Desktop offline DG-KILO environment is ready."
    echo "Installed setup: $DESKTOP_DGKILO_INSTALL/setup.bash"
else
    echo "Skipping DG-KILO build: source not present at $DGKILO_SRC."
fi
