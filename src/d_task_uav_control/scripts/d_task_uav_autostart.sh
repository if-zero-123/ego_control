#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="${D_TASK_UAV_WORKSPACE:-/home/orangepi/catkin_ws}"
mission_script="$workspace_dir/src/ego_control/src/d_task_uav_control/scripts/start_d_task_uav.sh"
broker_host="${D_TASK_MQTT_HOST:-192.168.0.198}"
broker_port="${D_TASK_MQTT_PORT:-1883}"

[[ -x "$mission_script" ]] || {
  echo "missing D-task UAV startup script: $mission_script" >&2
  exit 2
}

if [[ "${1:-}" == "--check" ]]; then
  exec "$mission_script" --dry-run
fi

wait_for_mqtt_broker() {
  until timeout 2 bash -c '>/dev/tcp/"$1"/"$2"' _ \
    "$broker_host" "$broker_port" 2>/dev/null; do
    echo "waiting for MQTT broker $broker_host:$broker_port" >&2
    sleep 2
  done
}

wait_for_mqtt_broker
export D_TASK_AUTOSTART_NONINTERACTIVE=true
exec "$mission_script" --mqtt-host "$broker_host" --mqtt-port "$broker_port"
