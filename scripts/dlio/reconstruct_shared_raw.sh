#!/bin/bash
# Backward-compatible wrapper for the renamed raw-bag reconstruction helper.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/reconstruct_raw.sh" "$@"
