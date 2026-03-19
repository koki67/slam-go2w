#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: $0 <bag_path> <config_path> <results_dir> [extra_ros_args...]" >&2
  exit 2
fi

bag_path=$1
config_path=$2
results_dir=$3
shift 3

source_ros_setup() {
  local candidate=

  source_candidate() {
    local path=$1
    set +u
    # shellcheck disable=SC1090
    . "$path"
    set -u
  }

  if [ -n "${ROS_SETUP_PATH:-}" ] && [ -f "${ROS_SETUP_PATH}" ]; then
    source_candidate "${ROS_SETUP_PATH}"
    return
  fi

  for candidate in \
    /opt/glim_ros2/install/setup.sh \
    /opt/glim_ros2/install/setup.bash \
    /root/ros2_ws/install/setup.sh \
    /root/ros2_ws/install/setup.bash \
    /opt/ros/*/setup.sh \
    /opt/ros/*/setup.bash; do
    if [ -f "$candidate" ]; then
      source_candidate "$candidate"
      return
    fi
  done

  echo "could not find a ROS setup script inside the container" >&2
  exit 2
}

if [ ! -e "$bag_path" ]; then
  echo "bag path does not exist: $bag_path" >&2
  exit 2
fi

if [ ! -d "$config_path" ]; then
  echo "config path does not exist: $config_path" >&2
  exit 2
fi

mkdir -p "$results_dir"
dump_dir=$results_dir/dump
rm -rf "$dump_dir"
mkdir -p "$dump_dir"
source_ros_setup

set +e
ros2 run glim_ros glim_rosbag "$bag_path" --ros-args \
  -p config_path:="$config_path" \
  -p auto_quit:=true \
  -p dump_path:="$dump_dir" \
  "$@"
status=$?
set -e

exit "$status"
