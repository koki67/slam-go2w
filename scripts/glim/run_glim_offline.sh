#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
bag_path=
image=${GLIM_OFFLINE_IMAGE_TAG:-go2w-glim-offline:jazzy_cuda13.1}
config_dir=$repo_root/config/glim/offline/go2w
config_mode=gpu
run_name=
overlay_config_dirs=()
skip_checker=0
use_gpu=1
checker_config=
dlio_repo=
imu_topic=
lidar_topic=
imu_frame=
lidar_frame=
docker_bin=${DOCKER_BIN:-docker}
glim_args=()
report_path=
checker_log_path=
prepare_log_path=
glim_log_path=
manifest_path=

usage() {
  cat <<'EOF'
usage: run_glim_offline.sh --bag PATH [options] [-- extra_glim_rosbag_args...]

Options:
  --bag PATH             Rosbag directory to process.
  --image IMAGE          Docker image tag to run.
  --config-dir PATH      GO2-W override config directory.
  --overlay-config-dir PATH
                        Additional config overlay directory. Can be repeated.
  --config-mode MODE     Config mode to resolve: gpu or cpu.
  --run-name NAME        Output run name. Defaults to bag name + UTC timestamp.
  --skip-checker         Skip the bag compatibility checker.
  --checker-config PATH  YAML overrides for tools/check_glim_bag.py.
  --dlio-repo PATH       Explicit D-LIO repo path for checker defaults.
  --imu-topic TOPIC      Checker override for expected IMU topic.
  --lidar-topic TOPIC    Checker override for expected LiDAR topic.
  --imu-frame FRAME      Checker override for expected IMU frame.
  --lidar-frame FRAME    Checker override for expected LiDAR frame.
  --no-gpu               Omit --gpus all when launching Docker.
  -h, --help             Show this help text.

Examples:
  scripts/glim/run_glim_offline.sh --bag /data/go2w_raw_bag
  scripts/glim/run_glim_offline.sh --bag /data/go2w_raw_bag --config-mode cpu --no-gpu
EOF
}

log() {
  printf '[offline-glim] %s\n' "$*"
}

fail_stage() {
  local stage=$1
  local status=$2
  local message=$3
  log "ERROR stage=${stage}: ${message}"
  printf 'stage.%s=%s\n' "$stage" "$status" >> "$manifest_path"
  printf 'final_stage=%s\nfinal_status=failed\n' "$stage" >> "$manifest_path"
  exit "$status"
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --bag)
      bag_path=$2
      shift 2
      ;;
    --image)
      image=$2
      shift 2
      ;;
    --config-dir)
      config_dir=$2
      shift 2
      ;;
    --config-mode)
      config_mode=$2
      shift 2
      ;;
    --overlay-config-dir)
      overlay_config_dirs+=("$2")
      shift 2
      ;;
    --run-name)
      run_name=$2
      shift 2
      ;;
    --skip-checker)
      skip_checker=1
      shift
      ;;
    --checker-config)
      checker_config=$2
      shift 2
      ;;
    --dlio-repo)
      dlio_repo=$2
      shift 2
      ;;
    --imu-topic)
      imu_topic=$2
      shift 2
      ;;
    --lidar-topic)
      lidar_topic=$2
      shift 2
      ;;
    --imu-frame)
      imu_frame=$2
      shift 2
      ;;
    --lidar-frame)
      lidar_frame=$2
      shift 2
      ;;
    --no-gpu)
      use_gpu=0
      shift
      ;;
    --)
      shift
      glim_args=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$bag_path" ]; then
  echo "--bag is required" >&2
  usage >&2
  exit 2
fi

bag_path=$(realpath "$bag_path")
config_dir=$(realpath "$config_dir")

if [ ! -e "$bag_path" ]; then
  echo "bag path does not exist: $bag_path" >&2
  exit 2
fi

if [ ! -d "$config_dir" ]; then
  echo "config directory does not exist: $config_dir" >&2
  exit 2
fi

if [ "${#overlay_config_dirs[@]}" -gt 0 ]; then
  for i in "${!overlay_config_dirs[@]}"; do
    overlay_config_dirs[$i]=$(realpath "${overlay_config_dirs[$i]}")
    if [ ! -d "${overlay_config_dirs[$i]}" ]; then
      echo "overlay config directory does not exist: ${overlay_config_dirs[$i]}" >&2
      exit 2
    fi
  done
fi

if [ -z "$run_name" ]; then
  bag_name=$(basename "$bag_path")
  run_name="${bag_name}_$(date -u +%Y%m%d_%H%M%S)"
fi

log_dir=$repo_root/output/logs/$run_name
result_dir=$repo_root/output/results/$run_name
report_dir=$repo_root/output/reports/$run_name
runtime_config_dir=$result_dir/resolved_config
dump_dir=$result_dir/dump
report_path=$report_dir/bag_checker.json
checker_log_path=$log_dir/bag_checker.stdout.log
prepare_log_path=$log_dir/config_prepare.stdout.log
glim_log_path=$log_dir/glim_rosbag.stdout.log
manifest_path=$report_dir/run_manifest.txt

mkdir -p "$log_dir" "$result_dir" "$report_dir" "$runtime_config_dir"

cat > "$manifest_path" <<EOF
run_name=$run_name
bag_path=$bag_path
image=$image
config_dir=$config_dir
config_mode=$config_mode
use_gpu=$use_gpu
runtime_config_dir=$runtime_config_dir
dump_dir=$dump_dir
overlay_config_dirs=$(IFS=:; echo "${overlay_config_dirs[*]}")
checker_log=$checker_log_path
prepare_log=$prepare_log_path
glim_log=$glim_log_path
bag_checker_report=$report_path
EOF

if [ "$skip_checker" -eq 0 ]; then
  log "Stage preflight: validating bag compatibility"
  checker_cmd=(
    python3
    "$repo_root/tools/check_glim_bag.py"
    "$bag_path"
    --report-path "$report_path"
  )

  if [ -n "$checker_config" ]; then
    checker_cmd+=(--config "$checker_config")
  fi
  if [ -n "$dlio_repo" ]; then
    checker_cmd+=(--dlio-repo "$dlio_repo")
  fi
  if [ -n "$imu_topic" ]; then
    checker_cmd+=(--imu-topic "$imu_topic")
  fi
  if [ -n "$lidar_topic" ]; then
    checker_cmd+=(--lidar-topic "$lidar_topic")
  fi
  if [ -n "$imu_frame" ]; then
    checker_cmd+=(--imu-frame "$imu_frame")
  fi
  if [ -n "$lidar_frame" ]; then
    checker_cmd+=(--lidar-frame "$lidar_frame")
  fi

  set +e
  "${checker_cmd[@]}" 2>&1 | tee "$checker_log_path"
  checker_status=${PIPESTATUS[0]}
  set -e
  if [ "$checker_status" -ne 0 ]; then
    fail_stage "preflight" "$checker_status" "bag validation failed; see $checker_log_path"
  fi
  printf 'stage.preflight=passed\n' >> "$manifest_path"
else
  log "Stage preflight: skipped"
  printf 'stage.preflight=skipped\n' >> "$manifest_path"
fi

log "Stage runtime_setup: checking Docker image $image"
if ! "$docker_bin" image inspect "$image" >/dev/null 2>&1; then
  fail_stage "runtime_setup" 1 "docker image not found locally: $image"
fi
printf 'stage.runtime_setup=passed\n' >> "$manifest_path"

prepare_cmd=(
  "$docker_bin" run --rm
  --user "$(id -u):$(id -g)"
  -e HOME=/tmp/go2w-glim
  -e ROS_SETUP_PATH=/opt/glim_ros2/install/setup.bash
  -v "$config_dir:/work/config_override:ro"
  -v "$runtime_config_dir:/work/runtime_config"
)

overlay_container_dirs=()
if [ "${#overlay_config_dirs[@]}" -gt 0 ]; then
  for i in "${!overlay_config_dirs[@]}"; do
    overlay_mount="/work/config_overlay_${i}"
    prepare_cmd+=(-v "${overlay_config_dirs[$i]}:${overlay_mount}:ro")
    overlay_container_dirs+=("$overlay_mount")
  done
fi

prepare_cmd+=(
  "$image"
  /opt/go2w/bin/prepare_glim_config.sh
  /work/config_override
  "$config_mode"
  /work/runtime_config
)

if [ "${#overlay_container_dirs[@]}" -gt 0 ]; then
  prepare_cmd+=("${overlay_container_dirs[@]}")
fi

log "Stage config_prepare: resolving upstream GLIM config and GO2-W overrides"
set +e
"${prepare_cmd[@]}" 2>&1 | tee "$prepare_log_path"
prepare_status=${PIPESTATUS[0]}
set -e
if [ "$prepare_status" -ne 0 ]; then
  fail_stage "config_prepare" "$prepare_status" "config preparation failed; see $prepare_log_path"
fi
printf 'stage.config_prepare=passed\n' >> "$manifest_path"

docker_cmd=(
  "$docker_bin" run --rm
  --user "$(id -u):$(id -g)"
  -e HOME=/tmp/go2w-glim
  -e ROS_SETUP_PATH=/opt/glim_ros2/install/setup.bash
  -v "$bag_path:/work/input/bag:ro"
  -v "$runtime_config_dir:/work/runtime_config:ro"
  -v "$log_dir:/work/output/logs"
  -v "$result_dir:/work/output/results"
)

if [ "$use_gpu" -eq 1 ]; then
  docker_cmd+=(--gpus all)
fi

docker_cmd+=(
  "$image"
  /opt/go2w/bin/run_glim_rosbag.sh
  /work/input/bag
  /work/runtime_config
  /work/output/results
)

if [ "${#glim_args[@]}" -gt 0 ]; then
  docker_cmd+=("${glim_args[@]}")
fi

log "Stage glim_execution: running glim_rosbag"
set +e
"${docker_cmd[@]}" 2>&1 | tee "$glim_log_path"
status=${PIPESTATUS[0]}
set -e

if [ "$status" -ne 0 ]; then
  fail_stage "glim_execution" "$status" "GLIM execution failed; see $glim_log_path"
fi

printf 'stage.glim_execution=passed\nfinal_stage=glim_execution\nfinal_status=passed\n' >> "$manifest_path"
log "Completed successfully: logs=$log_dir reports=$report_dir results=$result_dir"
exit "$status"
