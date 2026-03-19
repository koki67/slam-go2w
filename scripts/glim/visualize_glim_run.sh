#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
input_path=
run_name=
run_dir=
select_latest=0
summary_only=0
print_latest=0
docker_bin=${DOCKER_BIN:-docker}
image=${GLIM_VIS_IMAGE_TAG:-${GLIM_OFFLINE_IMAGE_TAG:-}}
optimization_mode=${GO2W_GLIM_VIEWER_OPTIMIZATION:-no}
gpu_mode=${GO2W_GLIM_VIEWER_GPU:-auto}
manifest_path=
viewer_log_path=
report_dir=
log_dir=
run_manifest_path=

usage() {
  cat <<'EOF'
usage: visualize_glim_run.sh <run_path> [options]
       visualize_glim_run.sh [--latest | --run-name NAME | --run-dir PATH] [options]

Inspect or launch the upstream GLIM offline viewer against a completed dump.

Selectors:
  <run_path>            Path to a completed run directory. If it points to a
                        dump directory or dump/graph.bin, the parent run
                        directory is inferred automatically.
  --latest               Use the most recently modified run under output/results/.
  --run-name NAME        Use output/results/NAME.
  --run-dir PATH         Use an explicit run directory path.
  --print-latest         Print the latest run path and exit.

Options:
  --summary-only         Print artifact summary without launching the viewer.
  --image IMAGE          Docker image tag to run. If omitted, the script first
                         reuses the run image from output/reports/<run_name>/
                         run_manifest.txt when available, then tries
                         go2w-glim-offline:jazzy_cuda13.1 and
                         go2w-glim-offline:jazzy.
  --gpu MODE             Viewer GPU mode: auto, yes, or no. Default: auto.
                         auto reuses use_gpu= from the recorded run manifest
                         when available, then falls back to inspecting the
                         resolved dump/config for GPU plugin references.
  --optimization MODE    Viewer load mode for the upstream "Do optimization?"
                         prompt: no, yes, or prompt. Default: no.
  -h, --help             Show this help text.

Examples:
  scripts/glim/visualize_glim_run.sh output/results/run_name
  scripts/glim/visualize_glim_run.sh /abs/path/to/output/results/run_name
  scripts/glim/visualize_glim_run.sh /abs/path/to/output/results/run_name/dump
  scripts/glim/visualize_glim_run.sh --run-name glim_raw_20260311_045358_e2e_cpu_v10
  scripts/glim/visualize_glim_run.sh --latest
  scripts/glim/visualize_glim_run.sh /abs/path/to/output/results/run_name --summary-only
  scripts/glim/visualize_glim_run.sh /abs/path/to/output/results/run_name --gpu yes
  scripts/glim/visualize_glim_run.sh /abs/path/to/output/results/run_name --optimization prompt
EOF
}

log() {
  printf '[offline-glim-view] %s\n' "$*"
}

fail() {
  log "ERROR: $*" >&2
  exit 1
}

fail_stage() {
  local stage=$1
  shift
  log "ERROR stage=${stage}: $*" >&2
  if [ -n "${manifest_path:-}" ]; then
    printf 'stage.%s=failed\nfinal_stage=%s\nfinal_status=failed\n' "$stage" "$stage" >> "$manifest_path"
  fi
  exit 1
}

latest_run_dir() {
  local results_root=$repo_root/output/results

  if [ ! -d "$results_root" ]; then
    return 1
  fi

  find "$results_root" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
    | sort -n \
    | tail -n 1 \
    | cut -d' ' -f2-
}

resolve_input_run_dir() {
  local candidate=$1
  local resolved_path=
  local parent_dir=

  if [ ! -e "$candidate" ]; then
    return 1
  fi

  resolved_path=$(realpath "$candidate")

  if [ -d "$resolved_path" ] && [ -d "$resolved_path/dump" ]; then
    printf '%s\n' "$resolved_path"
    return 0
  fi

  if [ -d "$resolved_path" ] && [ -f "$resolved_path/graph.bin" ] && [ -f "$resolved_path/traj_lidar.txt" ]; then
    printf '%s\n' "$(dirname "$resolved_path")"
    return 0
  fi

  if [ -f "$resolved_path" ] && [ "$(basename "$resolved_path")" = graph.bin ]; then
    parent_dir=$(dirname "$resolved_path")
    if [ -f "$parent_dir/traj_lidar.txt" ]; then
      printf '%s\n' "$(dirname "$parent_dir")"
      return 0
    fi
  fi

  return 1
}

resolve_default_image() {
  local recorded_image=
  local candidate=

  if [ -n "$image" ]; then
    printf '%s\n' "$image"
    return 0
  fi

  if [ -n "${run_manifest_path:-}" ] && [ -f "$run_manifest_path" ]; then
    recorded_image=$(sed -n 's/^image=//p' "$run_manifest_path" | head -n 1)
    if [ -n "$recorded_image" ] && "$docker_bin" image inspect "$recorded_image" >/dev/null 2>&1; then
      printf '%s\n' "$recorded_image"
      return 0
    fi
  fi

  for candidate in go2w-glim-offline:jazzy_cuda13.1 go2w-glim-offline:jazzy; do
    if "$docker_bin" image inspect "$candidate" >/dev/null 2>&1; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

manifest_value() {
  local path=$1
  local key=$2

  if [ -f "$path" ]; then
    sed -n "s/^${key}=//p" "$path" | head -n 1
  fi
}

config_uses_gpu() {
  local config_dir=$1
  local config_json=$config_dir/config.json

  if [ ! -f "$config_json" ]; then
    return 1
  fi

  if grep -Eq '"config_(odometry|sub_mapping|global_mapping)"[[:space:]]*:[[:space:]]*"[^"]*_gpu\.json"' "$config_json"; then
    return 0
  fi

  return 1
}

resolve_viewer_use_gpu() {
  local mode=$1
  local recorded_use_gpu=
  local config_dir=

  case "$mode" in
    yes)
      printf '1\n'
      return 0
      ;;
    no)
      printf '0\n'
      return 0
      ;;
    auto)
      ;;
    *)
      return 1
      ;;
  esac

  if [ -n "${run_manifest_path:-}" ] && [ -f "$run_manifest_path" ]; then
    recorded_use_gpu=$(manifest_value "$run_manifest_path" use_gpu)
    case "$recorded_use_gpu" in
      1)
        printf '1\n'
        return 0
        ;;
      0)
        printf '0\n'
        return 0
        ;;
    esac
  fi

  for config_dir in "$dump_dir/config" "$resolved_config"; do
    if config_uses_gpu "$config_dir"; then
      printf '1\n'
      return 0
    fi
  done

  printf '0\n'
}

resolve_xauthority() {
  local candidate=

  for candidate in \
    "${XAUTHORITY:-}" \
    "$HOME/.Xauthority" \
    "/run/user/$(id -u)/gdm/Xauthority"; do
    [ -n "$candidate" ] || continue
    if [ -r "$candidate" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

count_submaps() {
  local dump_dir=$1
  local count=0
  local path=
  local base=

  while IFS= read -r path; do
    base=$(basename "$path")
    if [[ "$base" =~ ^[0-9]+$ ]]; then
      count=$((count + 1))
    fi
  done < <(find "$dump_dir" -mindepth 1 -maxdepth 1 -type d | sort)

  printf '%s\n' "$count"
}

line_count_or_zero() {
  local path=$1
  if [ -f "$path" ]; then
    wc -l < "$path"
  else
    printf '0\n'
  fi
}

print_summary() {
  local resolved_run_name=$1
  local run_dir_path=$2
  local dump_dir=$3
  local graph_txt=$4
  local traj_lidar=$5
  local traj_imu=$6
  local resolved_config=$7
  local submap_count=$8
  local lidar_rows=$9
  local imu_rows=${10}

  printf 'Run name: %s\n' "$resolved_run_name"
  printf 'Run dir: %s\n' "$run_dir_path"
  printf 'Dump dir: %s\n' "$dump_dir"
  printf 'Submaps: %s\n' "$submap_count"
  printf 'traj_lidar.txt: %s (%s poses)\n' "$traj_lidar" "$lidar_rows"

  if [ -f "$traj_imu" ]; then
    printf 'traj_imu.txt: %s (%s poses)\n' "$traj_imu" "$imu_rows"
  else
    printf 'traj_imu.txt: missing\n'
  fi

  if [ -d "$resolved_config" ]; then
    printf 'resolved_config: %s\n' "$resolved_config"
  else
    printf 'resolved_config: missing (viewer will use upstream defaults)\n'
  fi

  if [ -f "$graph_txt" ]; then
    printf 'Graph stats:\n'
    grep -E '^num_' "$graph_txt" || true
  else
    printf 'Graph stats: graph.txt missing\n'
  fi
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --latest)
      select_latest=1
      shift
      ;;
    --run-name)
      run_name=$2
      shift 2
      ;;
    --run-dir)
      run_dir=$2
      shift 2
      ;;
    --summary-only)
      summary_only=1
      shift
      ;;
    --print-latest)
      print_latest=1
      shift
      ;;
    --image)
      image=$2
      shift 2
      ;;
    --gpu)
      gpu_mode=$2
      shift 2
      ;;
    --optimization)
      optimization_mode=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ "$1" == -* ]]; then
        echo "unknown argument: $1" >&2
        usage >&2
        exit 2
      fi
      if [ -n "$input_path" ]; then
        echo "multiple run paths provided: $input_path and $1" >&2
        usage >&2
        exit 2
      fi
      input_path=$1
      shift
      ;;
  esac
done

selector_count=0
[ -n "$input_path" ] && selector_count=$((selector_count + 1))
[ "$select_latest" -eq 1 ] && selector_count=$((selector_count + 1))
[ -n "$run_name" ] && selector_count=$((selector_count + 1))
[ -n "$run_dir" ] && selector_count=$((selector_count + 1))

if [ "$print_latest" -eq 1 ] && [ "$selector_count" -gt 0 ]; then
  fail "--print-latest cannot be combined with another run selector"
fi

if [ "$selector_count" -gt 1 ]; then
  fail "choose only one of <run_path>, --latest, --run-name, or --run-dir"
fi

case "$optimization_mode" in
  yes|no|prompt)
    ;;
  *)
    fail "unsupported --optimization value: $optimization_mode (expected yes, no, or prompt)"
    ;;
esac

case "$gpu_mode" in
  auto|yes|no)
    ;;
  *)
    fail "unsupported --gpu value: $gpu_mode (expected auto, yes, or no)"
    ;;
esac

if [ "$print_latest" -eq 1 ]; then
  latest_path=$(latest_run_dir) || fail "no run directories found under $repo_root/output/results"
  printf '%s\n' "$latest_path"
  exit 0
fi

if [ -n "$input_path" ]; then
  run_dir=$(resolve_input_run_dir "$input_path") \
    || fail "run path must point to a run directory, dump directory, or dump/graph.bin: $input_path"
elif [ -n "$run_dir" ]; then
  :
elif [ -n "$run_name" ]; then
  run_dir=$repo_root/output/results/$run_name
elif [ "$select_latest" -eq 1 ]; then
  run_dir=$(latest_run_dir) || fail "no run directories found under $repo_root/output/results"
else
  fail "a run path is required (or use --latest, --run-name, --run-dir, or --print-latest)"
fi

if [ ! -d "$run_dir" ]; then
  fail "run directory does not exist: $run_dir"
fi

run_dir=$(realpath "$run_dir")
resolved_run_name=$(basename "$run_dir")
dump_dir=$run_dir/dump
graph_bin=$dump_dir/graph.bin
graph_txt=$dump_dir/graph.txt
traj_lidar=$dump_dir/traj_lidar.txt
traj_imu=$dump_dir/traj_imu.txt
resolved_config=$run_dir/resolved_config
submap_count=0

log_dir=$repo_root/output/logs/$resolved_run_name
report_dir=$repo_root/output/reports/$resolved_run_name
viewer_log_path=$log_dir/offline_viewer.stdout.log
manifest_path=$report_dir/visualization_manifest.txt
if [[ "$run_dir" == "$repo_root"/output/results/* ]]; then
  run_manifest_path=$report_dir/run_manifest.txt
fi

if [ ! -d "$dump_dir" ]; then
  fail "dump directory does not exist: $dump_dir"
fi

if [ ! -f "$graph_bin" ]; then
  fail "required artifact missing: $graph_bin"
fi

if [ ! -f "$traj_lidar" ]; then
  fail "required artifact missing: $traj_lidar"
fi

submap_count=$(count_submaps "$dump_dir")
if [ "$submap_count" -eq 0 ]; then
  fail "no numeric submap directories found under $dump_dir"
fi

lidar_rows=$(line_count_or_zero "$traj_lidar")
imu_rows=$(line_count_or_zero "$traj_imu")

mkdir -p "$log_dir" "$report_dir"

cat > "$manifest_path" <<EOF
run_name=$resolved_run_name
run_dir=$run_dir
dump_dir=$dump_dir
image=${image:-auto}
gpu_mode=$gpu_mode
optimization_mode=$optimization_mode
viewer_log=$viewer_log_path
EOF

print_summary \
  "$resolved_run_name" \
  "$run_dir" \
  "$dump_dir" \
  "$graph_txt" \
  "$traj_lidar" \
  "$traj_imu" \
  "$resolved_config" \
  "$submap_count" \
  "$lidar_rows" \
  "$imu_rows"

printf 'stage.artifact_check=passed\n' >> "$manifest_path"

if [ "$summary_only" -eq 1 ]; then
  printf 'final_stage=artifact_check\nfinal_status=summary_only\n' >> "$manifest_path"
  exit 0
fi

if ! command -v "$docker_bin" >/dev/null 2>&1; then
  fail_stage "runtime_setup" "docker binary not found: $docker_bin"
fi

resolved_image=$(resolve_default_image) \
  || fail_stage "runtime_setup" "no Docker image found; build or specify one with --image"
printf 'image=%s\n' "$resolved_image" >> "$manifest_path"

if ! "$docker_bin" image inspect "$resolved_image" >/dev/null 2>&1; then
  fail_stage "runtime_setup" "docker image not found locally: $resolved_image"
fi

if ! "$docker_bin" run --rm --entrypoint /bin/bash "$resolved_image" -lc \
  'test -x /opt/glim_ros2/install/glim_ros/lib/glim_ros/offline_viewer || test -x /root/ros2_ws/install/glim_ros/lib/glim_ros/offline_viewer'; then
  fail_stage "runtime_setup" "offline_viewer is not present in Docker image: $resolved_image"
fi
printf 'stage.runtime_setup=passed\n' >> "$manifest_path"

if [ -n "${DISPLAY:-}" ]; then
  if [ ! -d /tmp/.X11-unix ]; then
    fail_stage "display_check" "DISPLAY is set to ${DISPLAY}, but /tmp/.X11-unix is missing"
  fi
elif [ -n "${WAYLAND_DISPLAY:-}" ]; then
  fail_stage "display_check" "WAYLAND_DISPLAY is set, but DISPLAY is empty. This wrapper currently expects X11/Xwayland forwarding. Use --summary-only or start an X11-capable session."
else
  fail_stage "display_check" "no DISPLAY or WAYLAND_DISPLAY found. Use --summary-only or run from a desktop session."
fi

xauth_file=$(resolve_xauthority) \
  || fail_stage "display_check" "DISPLAY is set, but no readable XAUTHORITY file was found. Export XAUTHORITY or use --summary-only."

printf 'display=%s\nxauthority=%s\n' "$DISPLAY" "$xauth_file" >> "$manifest_path"
printf 'stage.display_check=passed\n' >> "$manifest_path"

viewer_use_gpu=$(resolve_viewer_use_gpu "$gpu_mode") \
  || fail_stage "runtime_setup" "failed to resolve viewer GPU mode from --gpu=$gpu_mode"
printf 'viewer_use_gpu=%s\n' "$viewer_use_gpu" >> "$manifest_path"

viewer_cmd='set -eo pipefail; set +u; . /opt/glim_ros2/install/setup.bash; set -u; ros2 run glim_ros offline_viewer /work/run/dump'
if [ -d "$resolved_config" ]; then
  viewer_cmd+=' --config_path /work/run/resolved_config'
fi
viewer_cmd='export PATH=/opt/go2w/bin:$PATH; '"$viewer_cmd"

docker_cmd=(
  "$docker_bin" run --rm
  --entrypoint /bin/bash
  --user "$(id -u):$(id -g)"
  -e HOME=/tmp/go2w-glim
  -e DISPLAY="$DISPLAY"
  -e XAUTHORITY=/tmp/.Xauthority
  -e GO2W_GLIM_VIEWER_OPTIMIZATION="$optimization_mode"
  -e QT_X11_NO_MITSHM=1
  -v "$xauth_file:/tmp/.Xauthority:ro"
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro
  -v "$log_dir:/work/output/logs"
  -v "$repo_root/docker/desktop/glim/bin/zenity_auto_answer.sh:/opt/go2w/bin/zenity:ro"
  -v "$run_dir:/work/run:ro"
)

if [ "$viewer_use_gpu" -eq 1 ]; then
  docker_cmd+=(--gpus all)
fi

docker_cmd+=(
  "$resolved_image"
  -lc "$viewer_cmd"
)

log "Launching offline_viewer for run=$resolved_run_name"
log "Viewer log: $viewer_log_path"
log "Viewer GPU access: $( [ "$viewer_use_gpu" -eq 1 ] && printf 'enabled' || printf 'disabled' )"

set +e
"${docker_cmd[@]}" 2>&1 | tee "$viewer_log_path"
status=${PIPESTATUS[0]}
set -e

if [ "$status" -eq 130 ] || [ "$status" -eq 137 ] || [ "$status" -eq 143 ]; then
  printf 'stage.viewer_launch=interrupted\nfinal_stage=viewer_launch\nfinal_status=interrupted\n' >> "$manifest_path"
  log "Viewer interrupted"
  exit "$status"
fi

if [ "$status" -ne 0 ]; then
  fail_stage "viewer_launch" "offline_viewer failed; see $viewer_log_path"
fi

printf 'stage.viewer_launch=passed\nfinal_stage=viewer_launch\nfinal_status=passed\n' >> "$manifest_path"
log "Viewer exited successfully"
