#!/usr/bin/env bash
# One launcher: XR Python selected automatically. Always sim-only.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XR_PYTHON=/opt/mamba/envs/xr/bin/python
if [[ ! -x "${XR_PYTHON}" ]]; then
  echo 'Missing /opt/mamba/envs/xr/bin/python; build Docker image first' >&2
  exit 1
fi
cd "${ROOT}"
if [[ ${1:-} == --verify ]]; then
  exec env -u PYTHONPATH -u LD_LIBRARY_PATH "${XR_PYTHON}" "${ROOT}/xr_integrated_sim_overlay.py" --verify
fi
# Explicit gated simulation invocation, never a physical bridge launcher.
exec env -u PYTHONPATH -u LD_LIBRARY_PATH "${XR_PYTHON}" "${ROOT}/xr_integrated_sim_overlay.py" "$@"
