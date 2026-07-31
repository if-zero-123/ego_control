#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
用法：start_d_task_uav.sh [选项]

依次启动 ROS Master、MAVROS、MID360 驱动和 D 题无人机后端。
脚本不会发布起飞消息；后端启动后等待小车合法指令。

选项：
  --mqtt-host HOST        MQTT Broker 地址（默认：192.168.0.198）
  --mqtt-port PORT        MQTT Broker 端口（默认：1883）
  --video-device PATH     下视相机设备路径
  --mavros-fcu-url URL    传给 mavros px4.launch 的 fcu_url
  --dry-run               仅显示将执行的启动命令
  -h, --help              显示帮助
EOF
}

die() {
  echo "错误：$*" >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  [[ -n "$value" ]] || die "$option 缺少参数"
}

print_command() {
  printf '[dry-run]'
  printf ' %q' "$@"
  printf '\n'
}

mqtt_host="192.168.0.198"
mqtt_port="1883"
video_device="/dev/v4l/by-id/usb-DCX-250107-ZW_DECXIN-video-index0"
mavros_fcu_url=""
dry_run=false

while (($# > 0)); do
  case "$1" in
    --mqtt-host)
      require_value "$1" "${2:-}"
      mqtt_host="$2"
      shift 2
      ;;
    --mqtt-port)
      require_value "$1" "${2:-}"
      mqtt_port="$2"
      shift 2
      ;;
    --video-device)
      require_value "$1" "${2:-}"
      video_device="$2"
      shift 2
      ;;
    --mavros-fcu-url)
      require_value "$1" "${2:-}"
      mavros_fcu_url="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项：$1"
      ;;
  esac
done

mavros_command=(roslaunch mavros px4.launch)
if [[ -n "$mavros_fcu_url" ]]; then
  mavros_command+=("fcu_url:=$mavros_fcu_url")
fi
backend_command=(
  roslaunch d_task_uav_control d_task_uav.launch
  "mqtt_host:=$mqtt_host"
  "mqtt_port:=$mqtt_port"
  "video_device:=$video_device"
)

if [[ "$dry_run" == true ]]; then
  print_command roscore
  print_command "${mavros_command[@]}"
  print_command roslaunch livox_ros_driver2 msg_MID360s.launch
  print_command "${backend_command[@]}"
  exit 0
fi

ros_setup="${D_TASK_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
positioning_setup="${D_TASK_POSITIONING_SETUP:-/home/orangepi/ros_ws/devel/setup.bash}"
catkin_setup="${D_TASK_CATKIN_SETUP:-/home/orangepi/catkin_ws/devel/setup.bash}"
log_root="${D_TASK_LOG_ROOT:-${HOME}/.ros/d_task_uav}"
startup_timeout_s="${D_TASK_STARTUP_TIMEOUT_S:-30}"
skip_mqtt_check="${D_TASK_SKIP_MQTT_CHECK:-false}"

[[ "$mqtt_port" =~ ^[0-9]+$ ]] || die "MQTT 端口必须是整数：$mqtt_port"
[[ "$startup_timeout_s" =~ ^[1-9][0-9]*$ ]] \
  || die "D_TASK_STARTUP_TIMEOUT_S 必须是正整数"
[[ -e "$video_device" ]] || die "找不到相机设备：$video_device"

for setup_file in "$ros_setup" "$positioning_setup" "$catkin_setup"; do
  [[ -r "$setup_file" ]] || die "找不到环境文件：$setup_file"
done
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$positioning_setup"
# 保留定位工作空间，使 livox_ros_driver2 与当前任务包同时可见。
# shellcheck disable=SC1090
source "$catkin_setup" --extend

for required_command in roscore roslaunch rosnode rostopic setsid timeout; do
  command -v "$required_command" >/dev/null 2>&1 \
    || die "找不到命令：$required_command"
done

run_id="$(date +%Y%m%d_%H%M%S)_$$"
log_dir="$log_root/$run_id"
mkdir -p "$log_dir"

pids=()
process_names=()
cleanup_started=false

log() {
  printf '[d_task_start] %s\n' "$*"
}

cleanup() {
  local index pid watchdog_pid
  if [[ "$cleanup_started" == true ]]; then
    return
  fi
  cleanup_started=true
  trap - INT TERM

  if ((${#pids[@]} == 0)); then
    return
  fi

  log "正在停止本脚本启动的 ROS 进程..."
  for ((index = ${#pids[@]} - 1; index >= 0; --index)); do
    pid="${pids[$index]}"
    kill -TERM -- "-$pid" 2>/dev/null || true
  done

  (
    sleep 3
    for pid in "${pids[@]}"; do
      kill -KILL -- "-$pid" 2>/dev/null || true
    done
  ) </dev/null >/dev/null 2>&1 &
  watchdog_pid=$!

  for ((index = ${#pids[@]} - 1; index >= 0; --index)); do
    pid="${pids[$index]}"
    wait "$pid" 2>/dev/null || true
  done
  kill "$watchdog_pid" 2>/dev/null || true
  wait "$watchdog_pid" 2>/dev/null || true
  log "已完成清理"
}

handle_signal() {
  exit 0
}

trap handle_signal INT TERM
trap cleanup EXIT

start_component() {
  local name="$1"
  shift
  local log_file="$log_dir/$name.log"
  log "启动 $name，日志：$log_file"
  setsid "$@" >"$log_file" 2>&1 &
  pids+=("$!")
  process_names+=("$name")
}

node_exists() {
  local expected="$1"
  rosnode list 2>/dev/null | awk -v node="$expected" '$0 == node { found=1 } END { exit !found }'
}

wait_for_master() {
  local attempt
  for ((attempt = 0; attempt < startup_timeout_s * 10; ++attempt)); do
    if rosnode list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

wait_for_node() {
  local node="$1"
  local attempt response
  for ((attempt = 0; attempt < startup_timeout_s * 10; ++attempt)); do
    response="$(rosnode ping -c 1 "$node" 2>&1 || true)"
    if grep -q 'xmlrpc reply from' <<<"$response"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

wait_for_mavros_connection() {
  local attempt state
  for ((attempt = 0; attempt < startup_timeout_s; ++attempt)); do
    state="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
    if grep -Eq '^connected: (True|true)$' <<<"$state"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if rosnode list >/dev/null 2>&1; then
  log "复用已有 ROS Master"
else
  start_component roscore roscore
  wait_for_master || die "ROS Master 在 ${startup_timeout_s}s 内未就绪"
fi

for task_node in /d_task_mission /uav_protocol_gateway /fastlio_supervisor; do
  node_exists "$task_node" && die "检测到已有任务节点 $task_node，请先停止旧任务后端"
done

if node_exists /mavros; then
  log "复用已有 MAVROS"
else
  start_component mavros "${mavros_command[@]}"
  wait_for_node /mavros || die "MAVROS 节点在 ${startup_timeout_s}s 内未就绪"
fi
wait_for_mavros_connection \
  || die "MAVROS 未在 ${startup_timeout_s}s 内连接飞控"

if node_exists /livox_lidar_publisher2; then
  log "复用已有 MID360 驱动"
else
  start_component mid360 roslaunch livox_ros_driver2 msg_MID360s.launch
  wait_for_node /livox_lidar_publisher2 \
    || die "MID360 驱动在 ${startup_timeout_s}s 内未就绪"
fi

if [[ "$skip_mqtt_check" != true ]]; then
  if timeout 2 bash -c 'exec 3<>"/dev/tcp/$1/$2"' _ "$mqtt_host" "$mqtt_port" \
      >/dev/null 2>&1; then
    log "MQTT Broker 可连接：$mqtt_host:$mqtt_port"
  else
    log "警告：暂时无法连接 MQTT Broker $mqtt_host:$mqtt_port，网关将继续重连"
  fi
fi

start_component d_task_uav "${backend_command[@]}"
wait_for_node /d_task_mission \
  || die "D 题任务节点在 ${startup_timeout_s}s 内未就绪"

log "全部软件已启动，当前不会自动起飞"
log "请通过小车发送合法 mission_config，定位就绪后等待小车 mission_start"
log "状态检查：rostopic echo /d_task/positioning/status"
log "任务检查：rostopic echo /uav_protocol/task_state"
log "全部日志：$log_dir"
log "按 Ctrl+C 停止本脚本启动的进程"

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

exited_name="某个受管 ROS 进程"
for index in "${!pids[@]}"; do
  if ! kill -0 "${pids[$index]}" 2>/dev/null; then
    exited_name="${process_names[$index]}"
    break
  fi
done

if ((status == 0)); then
  die "$exited_name 意外退出"
else
  die "$exited_name 异常退出，状态码：$status"
fi
