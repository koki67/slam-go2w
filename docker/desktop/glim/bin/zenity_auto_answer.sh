#!/usr/bin/env bash
set -euo pipefail

real_zenity=/usr/bin/zenity
mode=${GO2W_GLIM_VIEWER_OPTIMIZATION:-no}
title=
text=
question=0
original_args=("$@")

if [ ! -x "$real_zenity" ]; then
  echo "[go2w-zenity] real zenity binary not found: $real_zenity" >&2
  exit 127
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    --question)
      question=1
      shift
      ;;
    --title)
      title=${2:-}
      shift 2
      ;;
    --text)
      text=${2:-}
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

if [ "$question" -eq 1 ] && [ "$title" = "Confirm" ] && [ "$text" = "Do optimization?" ] && [ "$mode" != "prompt" ]; then
  case "$mode" in
    yes)
      echo "[go2w-zenity] auto-answering Yes to offline_viewer optimization prompt" >&2
      printf 'Yes\n'
      exit 0
      ;;
    no)
      echo "[go2w-zenity] auto-answering No to offline_viewer optimization prompt" >&2
      printf 'No\n'
      exit 0
      ;;
    *)
      echo "[go2w-zenity] unsupported GO2W_GLIM_VIEWER_OPTIMIZATION=$mode" >&2
      exit 2
      ;;
  esac
fi

exec "$real_zenity" "${original_args[@]}"
