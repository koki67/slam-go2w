#!/bin/bash
# Record the visible RViz2 window over X11/Xwayland with ffmpeg.
#
# Usage:
#   bash scripts/dlio/record_rviz_window.sh [--output <file.mp4>] [--framerate <fps>] [--show-cursor] [--timeout <seconds>] [--title <pattern>]
#
# Examples:
#   bash scripts/dlio/record_rviz_window.sh
#   bash scripts/dlio/record_rviz_window.sh --output /tmp/rviz.mp4 --show-cursor
#   bash scripts/dlio/record_rviz_window.sh --title RViz

set -euo pipefail

output_path="$PWD/rviz_$(date +%Y%m%d_%H%M%S).mp4"
framerate=60
draw_mouse=0
timeout_seconds=0
window_title_pattern="RViz"

usage() {
    sed -n '2,11p' "$0"
}

require_command() {
    local cmd="$1"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Error: required command not found in PATH: $cmd" >&2
        exit 1
    fi
}

find_rviz_window_id() {
    local raw_id=""

    if command -v xdotool >/dev/null 2>&1; then
        raw_id="$(xdotool search --onlyvisible --class rviz2 2>/dev/null | tail -n 1 || true)"
        if [ -z "$raw_id" ]; then
            raw_id="$(xdotool search --onlyvisible --name "$window_title_pattern" 2>/dev/null | tail -n 1 || true)"
        fi
    fi

    if [ -z "$raw_id" ] && command -v xwininfo >/dev/null 2>&1; then
        raw_id="$(
            xwininfo -root -tree 2>/dev/null \
                | awk -v pattern="$window_title_pattern" '
                    BEGIN { pattern = tolower(pattern) }
                    tolower($0) ~ pattern && $1 ~ /^0x[0-9a-f]+$/ { print $1 }
                ' \
                | tail -n 1 || true
        )"
    fi

    if [ -n "$raw_id" ]; then
        printf '%d\n' "$((raw_id))"
    fi
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --output)
            output_path="${2:?Error: --output requires a value}"
            shift 2
            ;;
        --framerate)
            framerate="${2:?Error: --framerate requires a value}"
            shift 2
            ;;
        --show-cursor)
            draw_mouse=1
            shift
            ;;
        --timeout)
            timeout_seconds="${2:?Error: --timeout requires a value}"
            shift 2
            ;;
        --title)
            window_title_pattern="${2:?Error: --title requires a value}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [ -z "${DISPLAY:-}" ]; then
    echo "Error: DISPLAY is not set. Run this from the same X11/Xwayland session as RViz." >&2
    exit 1
fi

require_command ffmpeg
if ! command -v xdotool >/dev/null 2>&1 && ! command -v xwininfo >/dev/null 2>&1; then
    echo "Error: install xdotool or xwininfo so the script can discover the RViz window." >&2
    exit 1
fi

if ! [[ "$framerate" =~ ^[0-9]+$ ]] || [ "$framerate" -le 0 ]; then
    echo "Error: --framerate must be a positive integer" >&2
    exit 1
fi

if ! [[ "$timeout_seconds" =~ ^[0-9]+$ ]]; then
    echo "Error: --timeout must be a non-negative integer" >&2
    exit 1
fi

mkdir -p "$(dirname "$output_path")"

echo "Waiting for a visible RViz window on DISPLAY=$DISPLAY"
echo "Window match pattern: $window_title_pattern"

window_id=""
start_time="$(date +%s)"
while [ -z "$window_id" ]; do
    window_id="$(find_rviz_window_id || true)"
    if [ -n "$window_id" ]; then
        break
    fi

    if [ "$timeout_seconds" -gt 0 ]; then
        now="$(date +%s)"
        if [ $((now - start_time)) -ge "$timeout_seconds" ]; then
            echo "Error: timed out waiting for an RViz window after ${timeout_seconds}s" >&2
            exit 1
        fi
    fi

    sleep 1
done

echo "Recording RViz window id: $window_id"
echo "Output: $output_path"
echo "Frame rate: ${framerate} fps"
if [ "$draw_mouse" -eq 1 ]; then
    echo "Mouse cursor: visible"
else
    echo "Mouse cursor: hidden"
fi
echo "Stop recording with q in this terminal or Ctrl+C."

exec ffmpeg -y \
    -f x11grab \
    -framerate "$framerate" \
    -draw_mouse "$draw_mouse" \
    -window_id "$window_id" \
    -i "$DISPLAY" \
    -c:v libx264 \
    -preset veryfast \
    -crf 18 \
    -pix_fmt yuv420p \
    "$output_path"
