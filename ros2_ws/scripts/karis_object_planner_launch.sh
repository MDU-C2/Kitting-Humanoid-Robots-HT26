#!/usr/bin/env bash
# Launch the existing KARIS -> MoveIt interactive planner with the same
# CycloneDDS configuration as the *running* MoveIt server.
# This launcher does not start G1 control or request trajectory execution.
# ROS 2 Humble's generated setup.bash references optional variables without
# defining them first; enable errexit/pipefail but NOT nounset while sourcing.
# The Python launcher below explicitly validates the required values.
set -eo pipefail

cd /workspace

# Load ROS packages / KARIS interfaces. This script may install a different
# CYCLONEDDS_URI, so the Python block below restores the active MoveIt one.
if [[ -r /workspace/karis_ros_env.sh ]]; then
  source /workspace/karis_ros_env.sh
else
  echo 'ERROR: /workspace/karis_ros_env.sh missing; source the correct ROS overlay.' >&2
  exit 1
fi

python3 - "$@" <<'PY'
import os
import pathlib
import sys

# Prefer the explicitly deployed /tmp version while testing; fall back to
# the persistent workspace copy. Print the selected path for verification.
candidates = (
    '/workspace/scripts/karis_object_review_execute.py',
    '/tmp/karis_object_review_execute.py',
)
planner = next((p for p in candidates if os.path.isfile(p)), None)
if planner is None:
    raise SystemExit('ERROR: Planner not found. Copy karis_object_review_execute_v2.py to '
                     '/workspace/scripts/karis_object_review_execute.py or '
                     '/tmp/karis_object_review_execute.py first.')

running = []
for proc in pathlib.Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        cmd = (proc / 'cmdline').read_bytes().split(b'\0')
        if not cmd or not cmd[0]:
            continue  # includes zombie processes
        exe = os.fsdecode(cmd[0])
        if os.path.basename(exe) != 'move_group':
            continue
        state = (proc / 'stat').read_text().split(') ', 1)[1].split()[0]
        if state == 'Z':
            continue
        entries = (proc / 'environ').read_bytes().split(b'\0')
        env = {}
        for entry in entries:
            if b'=' not in entry:
                continue
            key, value = entry.split(b'=', 1)
            env[os.fsdecode(key)] = os.fsdecode(value)
        if 'CYCLONEDDS_URI' not in env:
            continue
        running.append((int(proc.name), env))
    except (PermissionError, FileNotFoundError, ProcessLookupError, OSError):
        continue

if len(running) != 1:
    raise SystemExit(f'ERROR: Expected exactly one live MoveIt move_group with '
                     f'accessible DDS environment; found {len(running)}. '
                     'Do not launch with guessed DDS settings.')

pid, moveit_env = running[0]
env = os.environ.copy()
keys = (
    'CYCLONEDDS_URI', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID',
    'ROS_LOCALHOST_ONLY', 'G1_INTERFACE', 'G1_ROBOT_IP',
    'G1_ROS_WIFI_INTERFACE', 'G1_ROS_PEERS',
)
for key in keys:
    if key in moveit_env:
        env[key] = moveit_env[key]
    else:
        env.pop(key, None)

if not env.get('CYCLONEDDS_URI'):
    raise SystemExit('ERROR: Active MoveIt has no readable CYCLONEDDS_URI')

print(f'Using active MoveIt PID {pid} ROS/DDS settings (not a hard-coded PID).', flush=True)
print(f"ROS_DOMAIN_ID={env.get('ROS_DOMAIN_ID', '<unset>')}  "
      f"RMW_IMPLEMENTATION={env.get('RMW_IMPLEMENTATION', '<unset>')}", flush=True)
print(f"G1: {env.get('G1_INTERFACE', '<unset>')} -> "
      f"{env.get('G1_ROBOT_IP', '<unset>')}; "
      f"Kim ROS peer: {env.get('G1_ROS_WIFI_INTERFACE', '<unset>')} -> "
      f"{env.get('G1_ROS_PEERS', '<unset>')}", flush=True)
print(f'Running {planner}. PLAN remains separate from user-confirmed EXECUTE.', flush=True)

argv = [sys.executable, planner, *sys.argv[1:]]  # Planner resolves per-arm defaults.
# The wrapper is evaluated from a heredoc, so fd 0 still points at that
# already-consumed heredoc. Restore stdin to the calling interactive TTY
# BEFORE replacing this Python interpreter with the prompt-driven planner.
try:
    tty_fd = os.open('/dev/tty', os.O_RDONLY)
except OSError as exc:
    raise SystemExit(f'ERROR: Interactive terminal stdin unavailable: {exc}')
try:
    os.dup2(tty_fd, 0)
finally:
    os.close(tty_fd)
os.execvpe(sys.executable, argv, env)
PY
