#!/usr/bin/env bash
# Writes docker/.env for this computer, so nobody has to edit it by hand. Run in WSL / on the
# host, from anywhere in the repo:
#
#     bash docker/setup_env.sh                          # cable to the robot (or no robot)
#     bash docker/setup_env.sh --robot 192.168.123.213  # WiFi: the robot's WiFi address (ip -4 -o addr | grep wlan0 on the G1)
#     bash docker/setup_env.sh --cable                  # back to cable (no peers)
#     bash docker/setup_env.sh --domain 42              # ROS_DOMAIN_ID (0 for the robot)
#
# Paths (KARIS_REPO, KARIS_DIR = <repo>/local_data) come from where this repo is. Other lines
# already in docker/.env are kept. Afterwards recreate the containers:
#     docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml up -d --force-recreate perception
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
ENV="$HERE/.env"
[ -f "$ENV" ] || cp "$HERE/.env.example" "$ENV"

set_key() {   # set_key KEY VALUE: replace the line KEY=... or append it
    if grep -q "^$1=" "$ENV"; then
        sed -i "s|^$1=.*|$1=$2|" "$ENV"
    else
        echo "$1=$2" >> "$ENV"
    fi
}

while [ $# -gt 0 ]; do
    case "$1" in
        --robot)  set_key KARIS_DDS_PEERS "$2"; shift 2 ;;
        --cable)  set_key KARIS_DDS_PEERS ""; shift ;;
        --domain) set_key ROS_DOMAIN_ID "$2"; shift 2 ;;
        --if)     set_key KARIS_NET_IF "$2"; shift 2 ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "unknown option $1 (see --help)"; exit 1 ;;
    esac
done
set_key KARIS_REPO "$REPO"
set_key KARIS_DIR "$REPO/local_data"
mkdir -p "$REPO/local_data/models" "$REPO/local_data/data" "$REPO/local_data/videos" "$REPO/local_data/bags"

echo "docker/.env:"
grep -E '^(KARIS_REPO|KARIS_DIR|ROS_DOMAIN_ID|KARIS_NET_IF|KARIS_DDS_PEERS)=' "$ENV" | sed 's/^/  /'
echo "Now (re)create the containers:"
echo "  docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml up -d --force-recreate perception"
echo "  docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d --force-recreate isaac_fp"
