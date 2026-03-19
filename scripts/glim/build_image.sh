#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
base_image=${GLIM_BASE_IMAGE:-koide3/glim_ros2:jazzy_cuda13.1}
tag=${GLIM_OFFLINE_IMAGE_TAG:-go2w-glim-offline:jazzy_cuda13.1}
no_cache=0

usage() {
  cat <<'EOF'
usage: build_image.sh [--base-image IMAGE] [--tag TAG] [--no-cache]

Build the desktop-side offline GLIM wrapper image.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --base-image)
      base_image=$2
      shift 2
      ;;
    --tag)
      tag=$2
      shift 2
      ;;
    --no-cache)
      no_cache=1
      shift
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

cmd=(
  docker build
  -f "$repo_root/docker/desktop/glim/Dockerfile"
  --build-arg "GLIM_BASE_IMAGE=$base_image"
  -t "$tag"
)

if [ "$no_cache" -eq 1 ]; then
  cmd+=(--no-cache)
fi

cmd+=("$repo_root")

printf 'Building image %s from %s\n' "$tag" "$base_image"
"${cmd[@]}"
