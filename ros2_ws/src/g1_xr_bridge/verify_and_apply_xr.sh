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
# G1_MOVEIT_DIRECT_PATCH_CHECK_V1: installed XR may be at pinned upstream,
# original integration-patched, or fully overlay-patched state. Apply in order.
DIRECT="$BR/patches/xr_moveit_direct_commands.patch"
[[ -f "$DIRECT" ]] || { echo 'Missing automatic MoveIt direct-command overlay' >&2; exit 1; }
if git -C "$XR" apply --reverse --check "$DIRECT" >/dev/null 2>&1; then
  echo 'OK: XR MoveIt direct-command overlay already applied.'
else
  if git -C "$XR" apply --reverse --check "$PATCH" >/dev/null 2>&1; then
    echo 'OK: original XR integration patch already applied.'
  elif git -C "$XR" apply --check "$PATCH" >/dev/null 2>&1; then
    if [[ "${1:-}" != '--apply' ]]; then
      echo 'XR integration not installed. Run install.sh while offline.' >&2
      exit 1
    fi
    git -C "$XR" apply "$PATCH"
    echo 'APPLIED: original XR headless integration patch.'
  else
    echo 'ERROR: pinned XR source differs from both expected patch states.' >&2
    exit 1
  fi
  if ! git -C "$XR" apply --check "$DIRECT" >/dev/null 2>&1; then
    echo 'ERROR: direct-command overlay does not apply cleanly; refusing launch.' >&2
    exit 1
  fi
  if [[ "${1:-}" != '--apply' ]]; then
    echo 'XR direct-command overlay not installed. Run install.sh while offline.' >&2
    exit 1
  fi
  git -C "$XR" apply "$DIRECT"
  echo 'APPLIED: XR MoveIt direct-command overlay.'
fi
# Fail closed if the opt-in path does not match the expected implementation.
XR_ARM="$XR/teleop/robot_control/robot_arm.py"
grep -Fq 'G1_MOVEIT_DIRECT_COMMANDS_V1' "$XR_ARM" || exit 1
if grep -Fq 'def _headless_slew_step(' "$XR_ARM"; then
  echo 'ERROR: old XR headless slew limiter is still installed.' >&2
  exit 1
fi
[[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'ERROR: XR Micromamba Python missing; build Docker image using existing docker/install_xr_micromamba.sh.' >&2; exit 1; }
[[ -f "$BR/xr_headless_moveit.py" && -f "$BR/ros2_headless_moveit_action.py" ]] || { echo 'ERROR: bridge code missing.' >&2; exit 1; }
python3 -m py_compile "$BR/xr_headless_moveit.py" "$BR/ros2_headless_moveit_action.py"
echo 'PASS: offline headless XR prerequisites and Python syntax.'
