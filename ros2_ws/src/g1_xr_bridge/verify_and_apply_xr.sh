#!/usr/bin/env bash
# Safe, offline XR dependency and patch setup. Never runs a motor publisher.
set -Eeuo pipefail
WS="${G1_WORKSPACE:-/workspace}"
BR="$WS/src/g1_xr_bridge"
XR="$WS/src/xr_teleoperate"
SDK="$WS/src/unitree_sdk2_python"
REVS="$BR/patches/dependency_revisions.txt"
PATCH="$BR/patches/xr_headless_moveit.patch"
[[ -f "$REVS" && -f "$PATCH" ]] || { echo "Missing committed revisions or XR patch under $BR/patches" >&2; exit 1; }
getrev() { sed -n "s/^$1=//p" "$REVS" | head -1; }
for entry in "xr_teleoperate:$XR" "unitree_sdk2_python:$SDK"; do
  name="${entry%%:*}"; dir="${entry#*:}"
  [[ -d "$dir/.git" ]] || { echo "Missing clone: $dir. Import via your existing docker/import_dependencies.sh on the host." >&2; exit 1; }
  expected="$(getrev "$name")"; actual="$(git -C "$dir" rev-parse HEAD)"
  [[ -n "$expected" && "$actual" == "$expected" ]] || { echo "REVISION MISMATCH: $name expected=$expected actual=$actual. No automatic checkout." >&2; exit 1; }
  echo "OK: $name $actual"
done
if git -C "$XR" apply --reverse --check "$PATCH" >/dev/null 2>&1; then
  echo 'OK: XR integration patch already applied exactly.'
elif git -C "$XR" apply --check "$PATCH" >/dev/null 2>&1; then
  if [[ "${1:-}" == '--apply' ]]; then
    git -C "$XR" apply "$PATCH"
    echo 'APPLIED: XR integration patch.'
  else
    echo 'READY: patch cleanly applicable. Run with --apply to install.'
  fi
else
  echo 'ERROR: XR patch neither cleanly applicable nor identical to installed changes; inspect before proceeding.' >&2; exit 1
fi
[[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'ERROR: XR Micromamba Python missing; build Docker image using existing docker/install_xr_micromamba.sh.' >&2; exit 1; }
[[ -f "$BR/xr_headless_moveit.py" && -f "$BR/ros2_headless_moveit_action.py" ]] || { echo 'ERROR: bridge code missing.' >&2; exit 1; }
python3 -m py_compile "$BR/xr_headless_moveit.py" "$BR/ros2_headless_moveit_action.py"
echo 'PASS: offline headless XR prerequisites and Python syntax.'
