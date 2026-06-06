#!/usr/bin/env bash
set -Eeuo pipefail

CONFIG_FILE="${CONFIG_FILE:-/etc/default/craic-autostart}"
if [[ -r "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/orangepi/catkin_ws}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_RUN_USER="${ROS_RUN_USER:-orangepi}"
ROS_USER_HOME="${ROS_USER_HOME:-}"
TARGET_WIFI_SSID="${TARGET_WIFI_SSID:-316}"
TARGET_WIFI_PASSWORD="${TARGET_WIFI_PASSWORD:-}"
WIFI_IFACE="${WIFI_IFACE:-}"
WIFI_SCAN_TIMEOUT="${WIFI_SCAN_TIMEOUT:-20}"
WIFI_CONNECT_TIMEOUT="${WIFI_CONNECT_TIMEOUT:-30}"
HOTSPOT_ENABLE="${HOTSPOT_ENABLE:-true}"
HOTSPOT_SSID="${HOTSPOT_SSID:-CRAIC_MID360}"
HOTSPOT_PASSWORD="${HOTSPOT_PASSWORD:-12345678}"
HOTSPOT_CHANNEL="${HOTSPOT_CHANNEL:-6}"
HOTSPOT_IP="${HOTSPOT_IP:-192.168.66.1}"
WEB_PORT="${WEB_PORT:-8088}"
WEB_HOST="${WEB_HOST:-}"
WEB_HOST_MODE="${WEB_HOST_MODE:-ip}"
LAUNCH_GAP_SEC="${LAUNCH_GAP_SEC:-3}"
PX4_FIRST_GAP_SEC="${PX4_FIRST_GAP_SEC:-5}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP_AUTO="${ROS_IP_AUTO:-true}"
ROS_IP="${ROS_IP:-}"
LIVOX_PACKAGE="${LIVOX_PACKAGE:-livox_ros_driver2}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360s.launch}"
MAVROS_PACKAGE="${MAVROS_PACKAGE:-mavros}"
MAVROS_LAUNCH="${MAVROS_LAUNCH:-px4.launch}"
LIDAR_TO_MAVROS_PACKAGE="${LIDAR_TO_MAVROS_PACKAGE:-lidar_to_mavros}"
LIDAR_TO_MAVROS_LAUNCH="${LIDAR_TO_MAVROS_LAUNCH:-fastlio_to_px4_mid360_direct.launch}"
WEB_PACKAGE="${WEB_PACKAGE:-ego_api}"
WEB_LAUNCH="${WEB_LAUNCH:-craic_web_control.launch}"

if [[ -n "$ROS_RUN_USER" && -z "$ROS_USER_HOME" ]]; then
  ROS_USER_HOME="$(getent passwd "$ROS_RUN_USER" | awk -F: '{print $6}')"
fi
if [[ -z "$ROS_USER_HOME" ]]; then
  ROS_USER_HOME="${HOME:-/home/orangepi}"
fi

LOG_ROOT="${LOG_ROOT:-$ROS_USER_HOME/.ros/craic_autostart}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/logs/$RUN_ID"
ROS_LOG_DIR="$LOG_DIR/ros"
mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"

if [[ "$EUID" -eq 0 && -n "$ROS_RUN_USER" ]] && id "$ROS_RUN_USER" >/dev/null 2>&1; then
  chown -R "$ROS_RUN_USER:$ROS_RUN_USER" "$LOG_ROOT" 2>/dev/null || true
fi

exec > >(awk '{ print strftime("[%F %T]"), $0; fflush(); }' | tee -a "$LOG_DIR/autostart.log") 2>&1

PIDS=()
NAMES=()
AP_BACKEND=""
AP_STARTED=false
STOP_REQUESTED=false

log() {
  echo "[craic_autostart] $*" >&2
}

have() {
  command -v "$1" >/dev/null 2>&1
}

run_root() {
  if [[ "$EUID" -eq 0 ]]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

ros_env_cmd() {
  local quoted_args=""
  printf -v quoted_args "%q " "$@"
  printf "cd %q && source %q && source %q && export ROS_MASTER_URI=%q && export ROS_LOG_DIR=%q" \
    "$WORKSPACE_DIR" "/opt/ros/$ROS_DISTRO/setup.bash" "$WORKSPACE_DIR/devel/setup.bash" "$ROS_MASTER_URI" "$ROS_LOG_DIR"
  if [[ -n "${ROS_IP:-}" ]]; then
    printf " && export ROS_IP=%q && unset ROS_HOSTNAME" "$ROS_IP"
  fi
  printf " && exec %s" "$quoted_args"
}

run_ros_bg() {
  local name="$1"
  shift
  local logfile="$LOG_DIR/$name.log"
  local cmd
  cmd="$(ros_env_cmd "$@")"
  log "start $name: $*"
  if [[ "$EUID" -eq 0 && -n "$ROS_RUN_USER" ]] && id "$ROS_RUN_USER" >/dev/null 2>&1; then
    setsid runuser -u "$ROS_RUN_USER" -- env \
      "HOME=$ROS_USER_HOME" "USER=$ROS_RUN_USER" "LOGNAME=$ROS_RUN_USER" \
      bash -lc "$cmd" >>"$logfile" 2>&1 &
  else
    setsid bash -lc "$cmd" >>"$logfile" 2>&1 &
  fi
  PIDS+=("$!")
  NAMES+=("$name")
  sleep 0.5
}

run_ros_fg() {
  local cmd
  cmd="$(ros_env_cmd "$@")"
  if [[ "$EUID" -eq 0 && -n "$ROS_RUN_USER" ]] && id "$ROS_RUN_USER" >/dev/null 2>&1; then
    runuser -u "$ROS_RUN_USER" -- env \
      "HOME=$ROS_USER_HOME" "USER=$ROS_RUN_USER" "LOGNAME=$ROS_RUN_USER" \
      bash -lc "$cmd"
  else
    bash -lc "$cmd"
  fi
}

cleanup() {
  local rc=$?
  if [[ "$STOP_REQUESTED" == "true" ]]; then
    rc=0
  fi
  trap - EXIT INT TERM
  log "stopping, rc=$rc"
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    local pid="${PIDS[$i]}"
    local name="${NAMES[$i]}"
    if kill -0 "$pid" >/dev/null 2>&1; then
      log "SIGINT $name pid=$pid"
      kill -INT "-$pid" >/dev/null 2>&1 || kill -INT "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 8
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    local pid="${PIDS[$i]}"
    local name="${NAMES[$i]}"
    if kill -0 "$pid" >/dev/null 2>&1; then
      log "SIGTERM $name pid=$pid"
      kill -TERM "-$pid" >/dev/null 2>&1 || kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 3
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    local pid="${PIDS[$i]}"
    local name="${NAMES[$i]}"
    if kill -0 "$pid" >/dev/null 2>&1; then
      log "SIGKILL $name pid=$pid"
      kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done
  if [[ "$AP_STARTED" == "true" && "$AP_BACKEND" == "create_ap" && -n "$WIFI_IFACE" ]]; then
    log "stop hotspot on $WIFI_IFACE"
    run_root create_ap --stop "$WIFI_IFACE" >/dev/null 2>&1 || true
  fi
  exit "$rc"
}
trap cleanup EXIT
trap 'STOP_REQUESTED=true; cleanup' INT TERM

get_wifi_iface() {
  if [[ -n "$WIFI_IFACE" ]]; then
    echo "$WIFI_IFACE"
    return 0
  fi
  if have nmcli; then
    nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" && $1 != "--" {print $1; exit}'
    return 0
  fi
  if have iw; then
    iw dev | awk '$1 == "Interface" {print $2; exit}'
  fi
}

get_ipv4() {
  local iface="$1"
  ip -4 -o addr show dev "$iface" scope global 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}'
}

get_first_ipv4() {
  ip -4 -o addr show scope global 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}'
}

wait_for_ip() {
  local iface="$1"
  local timeout="$2"
  local ip_addr=""
  for ((i=0; i<timeout; i++)); do
    ip_addr="$(get_ipv4 "$iface")"
    if [[ -n "$ip_addr" ]]; then
      echo "$ip_addr"
      return 0
    fi
    sleep 1
  done
  return 1
}

ssid_visible() {
  local iface="$1"
  local ssid="$2"
  nmcli -t -f SSID device wifi list ifname "$iface" 2>/dev/null | sed 's/\\:/:/g' | grep -Fxq "$ssid"
}

connect_target_wifi() {
  local iface="$1"
  log "scan wifi ssid=$TARGET_WIFI_SSID iface=$iface"
  run_root nmcli radio wifi on || true
  for ((i=0; i<WIFI_SCAN_TIMEOUT; i+=3)); do
    run_root nmcli device wifi rescan ifname "$iface" >/dev/null 2>&1 || true
    sleep 1
    if ssid_visible "$iface" "$TARGET_WIFI_SSID"; then
      log "found wifi ssid=$TARGET_WIFI_SSID"
      if run_root nmcli connection up id "$TARGET_WIFI_SSID" ifname "$iface" >&2; then
        wait_for_ip "$iface" "$WIFI_CONNECT_TIMEOUT"
        return $?
      fi
      if [[ -n "$TARGET_WIFI_PASSWORD" ]]; then
        run_root nmcli device wifi connect "$TARGET_WIFI_SSID" password "$TARGET_WIFI_PASSWORD" ifname "$iface" >&2
      else
        run_root nmcli device wifi connect "$TARGET_WIFI_SSID" ifname "$iface" >&2
      fi
      wait_for_ip "$iface" "$WIFI_CONNECT_TIMEOUT"
      return $?
    fi
    sleep 2
  done
  return 1
}

start_hotspot() {
  local iface="$1"
  if [[ "$HOTSPOT_ENABLE" != "true" ]]; then
    return 1
  fi
  log "start hotspot ssid=$HOTSPOT_SSID iface=$iface ip=$HOTSPOT_IP"
  run_root nmcli radio wifi on || true
  run_root nmcli device disconnect "$iface" >/dev/null 2>&1 || true
  sleep 2
  if have create_ap; then
    if [[ ${#HOTSPOT_PASSWORD} -ge 8 ]]; then
      run_root create_ap --daemon -n -g "$HOTSPOT_IP" -c "$HOTSPOT_CHANNEL" "$iface" "$HOTSPOT_SSID" "$HOTSPOT_PASSWORD" >&2
    else
      log "HOTSPOT_PASSWORD shorter than 8 chars; create open hotspot"
      run_root create_ap --daemon -n -g "$HOTSPOT_IP" -c "$HOTSPOT_CHANNEL" "$iface" "$HOTSPOT_SSID" >&2
    fi
    AP_BACKEND="create_ap"
    AP_STARTED=true
    wait_for_ip "$iface" 15 || true
    echo "$HOTSPOT_IP"
    return 0
  fi
  if have nmcli && [[ ${#HOTSPOT_PASSWORD} -ge 8 ]]; then
    run_root nmcli device wifi hotspot ifname "$iface" ssid "$HOTSPOT_SSID" password "$HOTSPOT_PASSWORD" >&2
    AP_BACKEND="nmcli"
    AP_STARTED=true
    wait_for_ip "$iface" 15
    return $?
  fi
  return 1
}

port_in_use() {
  local port="$1"
  if have ss; then
    ss -ltn | awk '{print $4}' | grep -Eq "[:.]${port}$"
  else
    return 1
  fi
}

pick_port() {
  local base="$1"
  local port
  for ((port=base; port<base+30; port++)); do
    if ! port_in_use "$port"; then
      echo "$port"
      return 0
    fi
  done
  echo "$base"
}

wait_for_ros_master() {
  local timeout="${1:-30}"
  for ((i=0; i<timeout; i++)); do
    if run_ros_fg rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_launch() {
  local name="$1"
  local pkg="$2"
  local launch_file="$3"
  local gap_sec="$4"
  shift 4
  local resolved_launch="$launch_file"
  if [[ "$launch_file" != /* ]]; then
    local pkg_dir=""
    pkg_dir="$(run_ros_fg rospack find "$pkg" 2>/dev/null || true)"
    if [[ -n "$pkg_dir" ]]; then
      if [[ -f "$pkg_dir/launch/$launch_file" ]]; then
        resolved_launch="$launch_file"
      elif [[ -f "$pkg_dir/launch_ROS1/$launch_file" ]]; then
        resolved_launch="$pkg_dir/launch_ROS1/$launch_file"
      elif [[ -f "$pkg_dir/$launch_file" ]]; then
        resolved_launch="$pkg_dir/$launch_file"
      fi
    fi
  fi
  if [[ "$resolved_launch" == /* ]]; then
    run_ros_bg "$name" roslaunch --wait "$resolved_launch" "$@"
  else
    run_ros_bg "$name" roslaunch --wait "$pkg" "$resolved_launch" "$@"
  fi
  sleep "$gap_sec"
  local pid="${PIDS[-1]}"
  if ! kill -0 "$pid" >/dev/null 2>&1; then
    log "$name exited early; see $LOG_DIR/$name.log"
    return 1
  fi
}

resolve_launch() {
  local pkg="$1"
  local launch_file="$2"
  local pkg_dir=""
  local resolved="$launch_file"
  pkg_dir="$(run_ros_fg rospack find "$pkg" 2>/dev/null || true)"
  if [[ -z "$pkg_dir" ]]; then
    echo "MISSING_PACKAGE"
    return 1
  fi
  if [[ "$launch_file" == /* ]]; then
    [[ -f "$launch_file" ]] || return 1
    echo "$launch_file"
    return 0
  fi
  if [[ -f "$pkg_dir/launch/$launch_file" ]]; then
    resolved="$launch_file"
  elif [[ -f "$pkg_dir/launch_ROS1/$launch_file" ]]; then
    resolved="$pkg_dir/launch_ROS1/$launch_file"
  elif [[ -f "$pkg_dir/$launch_file" ]]; then
    resolved="$pkg_dir/$launch_file"
  else
    echo "MISSING_LAUNCH"
    return 1
  fi
  echo "$resolved"
}

check_config() {
  local failed=0
  log "check workspace: $WORKSPACE_DIR"
  [[ -r "/opt/ros/$ROS_DISTRO/setup.bash" ]] || { log "missing /opt/ros/$ROS_DISTRO/setup.bash"; failed=1; }
  [[ -r "$WORKSPACE_DIR/devel/setup.bash" ]] || { log "missing $WORKSPACE_DIR/devel/setup.bash"; failed=1; }
  local iface
  iface="$(get_wifi_iface || true)"
  log "wifi iface: ${iface:-none}"
  have nmcli && log "nmcli: $(command -v nmcli)" || log "nmcli: missing"
  have create_ap && log "create_ap: $(command -v create_ap)" || log "create_ap: missing, will try nmcli hotspot fallback"
  local spec name pkg launch_file resolved
  while read -r name pkg launch_file; do
    [[ -n "$name" ]] || continue
    if resolved="$(resolve_launch "$pkg" "$launch_file")"; then
      log "launch $name: $pkg $launch_file -> $resolved"
    else
      log "launch $name: $pkg $launch_file -> $resolved"
      failed=1
    fi
  done <<EOF
livox_mid360 $LIVOX_PACKAGE $LIVOX_LAUNCH
mavros_px4 $MAVROS_PACKAGE $MAVROS_LAUNCH
fastlio_to_px4 $LIDAR_TO_MAVROS_PACKAGE $LIDAR_TO_MAVROS_LAUNCH
craic_web_control $WEB_PACKAGE $WEB_LAUNCH
EOF
  return "$failed"
}

main() {
  if [[ "${1:-}" == "--check" ]]; then
    check_config
    exit $?
  fi

  log "log dir: $LOG_DIR"
  log "workspace: $WORKSPACE_DIR"

  if [[ ! -r "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    log "missing /opt/ros/$ROS_DISTRO/setup.bash"
    exit 2
  fi
  if [[ ! -r "$WORKSPACE_DIR/devel/setup.bash" ]]; then
    log "missing $WORKSPACE_DIR/devel/setup.bash"
    exit 2
  fi

  WIFI_IFACE="$(get_wifi_iface)"
  local access_ip=""
  local network_mode="none"
  if [[ -n "$WIFI_IFACE" ]] && have nmcli; then
    if access_ip="$(connect_target_wifi "$WIFI_IFACE")"; then
      network_mode="wifi:$TARGET_WIFI_SSID"
    else
      log "wifi ssid=$TARGET_WIFI_SSID unavailable or connect failed"
      if access_ip="$(start_hotspot "$WIFI_IFACE")"; then
        network_mode="hotspot:$HOTSPOT_SSID"
      fi
    fi
  else
    log "no wifi interface or nmcli unavailable"
  fi

  if [[ -z "$access_ip" ]]; then
    access_ip="$(get_first_ipv4 || true)"
  fi

  if [[ "$ROS_IP_AUTO" == "true" && -z "${ROS_IP:-}" && -n "$access_ip" ]]; then
    ROS_IP="$access_ip"
  fi

  local selected_port
  selected_port="$(pick_port "$WEB_PORT")"
  local web_host="$WEB_HOST"
  if [[ -z "$web_host" ]]; then
    if [[ "$WEB_HOST_MODE" == "all" ]]; then
      web_host="0.0.0.0"
    else
      web_host="${access_ip:-0.0.0.0}"
    fi
  fi

  log "network mode: $network_mode"
  log "wifi iface: ${WIFI_IFACE:-none}"
  log "access url: http://${access_ip:-$web_host}:$selected_port"
  log "web bind: $web_host:$selected_port"
  log "ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=${ROS_IP:-unset}"

  if run_ros_fg rostopic list >/dev/null 2>&1; then
    log "ROS master already running"
  else
    run_ros_bg roscore roscore
    if ! wait_for_ros_master 30; then
      log "roscore did not become ready; see $LOG_DIR/roscore.log"
      exit 3
    fi
  fi

  start_launch mavros_px4 "$MAVROS_PACKAGE" "$MAVROS_LAUNCH" "$PX4_FIRST_GAP_SEC"
  start_launch livox_mid360 "$LIVOX_PACKAGE" "$LIVOX_LAUNCH" "$LAUNCH_GAP_SEC"
  start_launch fastlio_to_px4 "$LIDAR_TO_MAVROS_PACKAGE" "$LIDAR_TO_MAVROS_LAUNCH" "$LAUNCH_GAP_SEC"
  start_launch craic_web_control "$WEB_PACKAGE" "$WEB_LAUNCH" "$LAUNCH_GAP_SEC" "host:=$web_host" "port:=$selected_port"

  log "all launches started"
  while true; do
    for ((i=0; i<${#PIDS[@]}; i++)); do
      if ! kill -0 "${PIDS[$i]}" >/dev/null 2>&1; then
        log "${NAMES[$i]} exited; service will stop and systemd can restart it"
        exit 4
      fi
    done
    sleep 5
  done
}

main "$@"
