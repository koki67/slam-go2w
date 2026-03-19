#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: $0 <base_override_dir> <config_mode:gpu|cpu> <output_dir> [overlay_dir ...]" >&2
  exit 2
fi

base_override_dir=$1
config_mode=$2
output_dir=$3
shift 3
overlay_dirs=("$@")

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

resolve_glim_share_dir() {
  python3 - <<'PY'
from __future__ import annotations

import subprocess

try:
    from ament_index_python.packages import get_package_share_directory

    print(get_package_share_directory("glim"))
except Exception:
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", "glim"], text=True).strip()
    print(f"{prefix}/share/glim")
PY
}

source_ros_setup

for required in config_ros.json config_sensors.json config_preprocess.json; do
  if [ ! -f "$base_override_dir/$required" ]; then
    echo "missing override file: $base_override_dir/$required" >&2
    exit 2
  fi
done

case "$config_mode" in
  gpu)
    config_entry=config_gpu.json
    ;;
  cpu)
    config_entry=config_cpu.json
    ;;
  *)
    echo "unsupported config mode: $config_mode" >&2
    exit 2
    ;;
esac

if [ ! -f "$base_override_dir/$config_entry" ]; then
  echo "missing config entrypoint: $base_override_dir/$config_entry" >&2
  exit 2
fi

glim_share_dir=$(resolve_glim_share_dir)

mkdir -p "$output_dir"
find "$output_dir" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
cp -a "$glim_share_dir/config/." "$output_dir/"

apply_override_dir() {
  local override_dir=$1
  local file=

  for file in "$override_dir"/*.json; do
    [ -e "$file" ] || continue
    cp "$file" "$output_dir/$(basename "$file")"
  done
}

apply_override_dir "$base_override_dir"
cp "$output_dir/$config_entry" "$output_dir/config.json"

if [ "${#overlay_dirs[@]}" -gt 0 ]; then
  for overlay_dir in "${overlay_dirs[@]}"; do
    if [ ! -d "$overlay_dir" ]; then
      echo "overlay directory does not exist: $overlay_dir" >&2
      exit 2
    fi
    apply_override_dir "$overlay_dir"
    if [ -f "$output_dir/$config_entry" ]; then
      cp "$output_dir/$config_entry" "$output_dir/config.json"
    fi
  done
fi

cat > "$output_dir/RESOLVED_CONFIG.txt" <<EOF
ros_setup_path=${ROS_SETUP_PATH:-auto}
glim_share_dir=$glim_share_dir
base_override_dir=$base_override_dir
config_mode=$config_mode
EOF

if [ "${#overlay_dirs[@]}" -gt 0 ]; then
  printf 'overlay_dirs=%s\n' "$(IFS=:; echo "${overlay_dirs[*]}")" >> "$output_dir/RESOLVED_CONFIG.txt"
fi
