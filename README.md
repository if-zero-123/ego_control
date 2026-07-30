# D题陆空协同无人机后端

本仓库中的 D 题后端由 `uav_protocol_gateway`、`d_task_uav_control`、
`ego_api` 和 `ego_bridge` 组成。无人机只运行 ROS1 控制和 MQTT Agent；
MQTT Broker 运行在小车 `192.168.0.198`，图形地面站运行在本机
`192.168.0.133`。

## 启动顺序

1. 启动 PX4、MAVROS，并确认 `/mavros/state`、
   `/mavros/local_position/odom` 正常。
2. MID360 驱动只启动一次并保持常驻：

   ```bash
   source /home/orangepi/ros_ws/devel/setup.bash
   roslaunch livox_ros_driver2 msg_MID360s.launch
   ```

3. 编译并加载当前工作空间：

   ```bash
   cd /home/orangepi/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. 启动无人机完整后端，连接小车上的 Broker：

   ```bash
   roslaunch d_task_uav_control d_task_uav.launch \
     mqtt_host:=192.168.0.198
   ```

   更换模型或摄像头设备时可附加：

   ```bash
   model_path:=/absolute/path/platform_target.rknn \
   video_device:=/dev/v4l/by-id/your-camera
   ```

也可以使用一键启动脚本。它会依次启动或复用 ROS Master、MAVROS 和
MID360 驱动，然后启动 D 题无人机完整后端：

```bash
/home/orangepi/catkin_ws/src/ego_control/src/d_task_uav_control/scripts/start_d_task_uav.sh
```

需要覆盖现场连接参数时使用：

```bash
/home/orangepi/catkin_ws/src/ego_control/src/d_task_uav_control/scripts/start_d_task_uav.sh \
  --mqtt-host 192.168.0.198 \
  --video-device /dev/v4l/by-id/your-camera
```

飞控不是 `mavros/px4.launch` 默认连接方式时，再按实际串口或 UDP 配置附加
`--mavros-fcu-url URL`，不要直接套用未经确认的设备名和波特率。

先用 `--dry-run` 可查看启动顺序而不启动进程。脚本前台常驻，日志写入
`~/.ros/d_task_uav/<启动时间>_<PID>/`；按 `Ctrl+C` 时只停止脚本自己启动的
进程，不停止启动前已经存在的 ROS Master、MAVROS 或 MID360 驱动。脚本不会
发布起飞消息，后端仍会停在准备流程并等待小车合法指令。注意合法
`mission/start` 到达后，当前 `ego_bridge` 配置会自动切换 OFFBOARD 并解锁，
投放任务也会按 `payload.enabled` 配置驱动真实 GPIO。

`d_task_uav.launch` 不启动 MID360 驱动，也不重复直接启动 mapping launch。
实体短按或地面站 `SELECT` 都先交给小车，由小车生成 mission ID 并发送
`mission_config(sender=car)`。无人机收到该配置后，`fastlio_supervisor`
才会先停止自己管理的旧进程，再启动：

```bash
roslaunch lidar_to_mavros fastlio_to_px4_mid360_direct.launch \
  rviz:=false zero_origin:=true
```

该 direct launch 已包含 `mapping_mid360_px4imu.launch`，不要再单独启动一份。
`fastlio_supervisor` 默认通过 `/home/orangepi/ros_ws/devel/env.sh` 启动该命令，
因此不依赖启动无人机后端的终端是否 source 过定位工作空间；路径可在
`positioning.workspace_env` 中修改。
当 `/Odometry`、`/mavros/vision_pose/pose` 和 PX4 odom 连续稳定 3 秒后，
系统记录 H 点并进入 `WAIT_START`。正式起飞只接受 `sender=car` 的
`mission/start`，且 `start_reason` 必须为实体长按的 `car_button` 或小车通过完整
安全门后代网页发布的 `ground_web`。地面站不能直接向无人机发起起飞。
UAV health 同时发布 `positioning_ready=true`，小车不会再把“旧 odom 仍新鲜”
误判为本次任务已经准备完成。
地面站 `START_CAR_ONLY` 只在小车侧打开调试任务，不会向无人机发布 start。
无人机内置协议与车端共同实现版本均为 `1.2.0`。UAV state 同时回传定位坐标系、
位姿有效性和数据年龄，地面站必须将过期位置显示为失效而不是继续外推。

## 任务流程

`DROP`：起飞至相对 H 点 1.50m，悬停 3 秒，按小车位姿接近，YOLO
锁定标靶并伴飞，稳定后立即下降到默认 0.80m 投放高度并触发投放流程，
随后爬升、返回 H 点和普通降落。`distance_to_d_m` 仅作 D 点截止保护，
不会等到 D 点附近才开始投放。

`DYNAMIC_LANDING`：起飞、搜索和伴飞后分 0.80m、0.30m 两段下降，
移动平台接触阶段继续发送平台位置与速度前馈。桥接层确认下压误差和低垂速
持续成立后锁桨，随车停留 5.20 秒，再预发 setpoint、切 OFFBOARD、解锁，
恢复 1.50m 高度并返航。接触超时最多自动取消并重试一次。

小车位姿超过 300ms 未更新时，网关关闭动态下降门。高空伴飞在视觉短时丢失、
但小车位姿仍新鲜时按小车世界坐标继续接近；进入投放下降、动态下降或平台接触后，
视觉超过 `pixel_servo.source_timeout_s` 未更新会立即保持当前高度和横向位置，不会继续下降。

## 现场参数

无人机任务常调参数集中在
`src/d_task_uav_control/config/d_task_uav.yaml`：

- `tracking.car_frame_offset_x_m/y_m`：小车 ROS 坐标起点到无人机坐标起点的平移。
- `tracking.use_visual_projection`：默认 `false`，避免未标定视觉投影影响小车世界坐标；仅保留旧方案时才开启。
- `tracking.platform_height_m`：平台中心在无人机世界系中的高度。
- `pixel_servo.*`：像素滤波、死区、最大横移速度及图像轴到机体系轴的映射；必须先完成台架四方向确认。
- `mission.cruise_height_m`、`drop_height_m`、两段下降高度和速度。
- 对中误差、相对速度、稳定时间、超时、5.20 秒停留和重试次数。
- `payload.*`：投放执行器预留配置。

参数在启动时加载，飞行中不热更新。调整坐标偏移、像素轴映射和平台高度后，
必须先做低高度、禁用自动解锁的实机验证。

YOLO 使用单类别平台标靶模型 `model_2_yolo_target_yolov8n_640_rk3588_i8.rknn`，标靶固定在
降落平台中心。检测节点每帧发布结构化检测，即使未发现目标也发布
`found=false`。小车世界坐标仍由四状态卡尔曼滤波输出平台位置和速度；视觉锁定后的
伴飞与下降使用像素伺服，不依赖相机内参。

仅验证相机和模型时运行：

```bash
roslaunch metal_ball_rknn platform_target_test.launch
```

在地面站运行 `rqt_image_view` 并订阅 `/metal_ball_rknn/debug_image` 查看带框画面；
`/d_task/vision/platform_detection` 输出结构化检测结果。

确认下视相机的图像轴与机体系方向时，拆桨、禁止解锁后运行：

```bash
roslaunch d_task_uav_control pixel_servo_debug.launch
```

该入口只启动相机、RKNN 检测和调试节点，不启动任务状态机、`ego_bridge` 或飞控控制。
观察 `/d_task/vision/pixel_servo_debug`、`/d_task/vision/pixel_velocity_body_debug`
和 `/d_task/vision/pixel_velocity_world_debug`；分别将标靶向机体前后左右移动，确认建议
速度方向后再修改 `pixel_servo.body_*` 参数。视觉超过 `pixel_servo.source_timeout_s` 未更新时，
调试与任务控制都会输出零像素修正，下降阶段会立即冻结。

投放执行器由 Orange Pi 5 Ultra 的 40Pin 排针物理 16 脚控制，即
`GPIO1_A3`（WiringOP 编号 `9`）。本机默认配置为 `payload.backend: gpio`、
`payload.gpio_active_high: true` 和 `payload.pulse_duration_s: 5.0`：状态机仅在
`RELEASE` 阶段把该脚输出高电平 5 秒，之后自动恢复低电平；节点启动、任务复位和
节点退出时也会强制输出低电平。`payload.enabled: false` 时保留完整状态机，但只发布
`PAYLOAD_RELEASE_DRY_RUN` 事件，不会输出高电平。

物理 16 脚只连接投放驱动模块的 `IN`，物理 14 脚可连接驱动模块 `GND`；驱动模块
电源必须按执行器额定电压单独供电，并与 Orange Pi 共地。严禁将电磁锁、继电器线圈或
电机直接接到 Orange Pi GPIO。需要改用 PX4 辅助输出时，将 `payload.backend` 改为
`mavros`，并继续使用已有的 `topic`、`group_mix`、`channel`、`release_value` 和
`neutral_value` 参数。

仅台架确认投放驱动输入时，可单独启动 GPIO 测试节点；它不启动任务状态机、桥接层、
飞控或视觉节点，启动和退出时均输出低电平：

```bash
roslaunch d_task_uav_control payload_gpio_test.launch
rostopic pub -1 /payload_gpio_test/set std_msgs/Bool "data: true"
rostopic pub -1 /payload_gpio_test/set std_msgs/Bool "data: false"
```

默认仍是物理 16 脚（`wiringpi_pin:=9`）和高电平有效；引脚或极性不同可通过
`wiringpi_pin:=<n>`、`active_high:=false` 覆盖。完成台架测试后退出该节点，再启动完整
任务后端，避免两个节点同时控制同一 GPIO。

## 关键状态与检查

地面站应依次看到 `POSITIONING_INIT -> WAIT_START -> TAKEOFF`，之后根据模式
进入投放或动态降落阶段，最终为 `COMPLETE` 或 `ABORT`。无人机 MQTT Agent
发布 ACK、state、tracking、health、heartbeat、event 和 safety/fault；任务状态
由任务节点发布，底层 bridge 状态不会覆盖正在运行的任务阶段。

首次实机运行前至少确认：MQTT 心跳稳定、三路定位 READY、相机健康、标靶世界
像素建议速度方向正确、车机坐标偏移正确、300ms 断链能停止下降、平台锁桨条件不会在
空中误触发，以及投放执行器仍处于 dry-run。
