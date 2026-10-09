#!/usr/bin/env bash
# No cd, PYTHONPATH manipulation, or Micromamba activation required.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON=/opt/mamba/envs/xr/bin/python
if [[ ! -x "$PYTHON" ]]; then
  echo "Missing XR environment: $PYTHON" >&2
  exit 1
fi
cd "$ROOT"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH "$PYTHON" "$ROOT/xr_inprocess_offline_harness.py"
