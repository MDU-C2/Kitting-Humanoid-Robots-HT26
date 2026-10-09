#!/usr/bin/env bash
# Enter from any directory, from within the ROS 2 Humble Docker container.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$ROOT/g1_pipeline.py" "$@"
