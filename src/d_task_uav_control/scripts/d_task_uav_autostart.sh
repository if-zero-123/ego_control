#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="${D_TASK_UAV_WORKSPACE:-/home/orangepi/catkin_ws}"
mission_script="$workspace_dir/src/ego_control/src/d_task_uav_control/scripts/start_d_task_uav.sh"

[[ -x "$mission_script" ]] || {
  echo "missing D-task UAV startup script: $mission_script" >&2
  exit 2
}

if [[ "${1:-}" == "--check" ]]; then
  exec "$mission_script" --dry-run
fi

exec "$mission_script" --mqtt-host "${D_TASK_MQTT_HOST:-192.168.0.198}"
