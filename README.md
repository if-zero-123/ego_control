# D题陆空协同无人机后端

本仓库中的 D 题后端由 `uav_protocol_gateway`、`d_task_uav_control`、
`ego_api` 和 `ego_bridge` 组成。无人机只运行 ROS1 控制和 MQTT Agent，
地面站网页与 MQTT Broker 均运行在地面站设备。

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

4. 启动无人机完整后端，将 Broker 地址改为地面站地址：

   ```bash
   roslaunch d_task_uav_control d_task_uav.launch \
     mqtt_host:=192.168.50.10
   ```

   更换模型或摄像头设备时可附加：

   ```bash
   model_path:=/absolute/path/platform_target.rknn \
   video_device:=/dev/v4l/by-id/your-camera
   ```

`d_task_uav.launch` 不启动 MID360 驱动，也不重复直接启动 mapping launch。
地面站选择任务并发送 `mission/config` 后，`fastlio_supervisor` 才会先停止
自己管理的旧进程，再启动：

```bash
roslaunch lidar_to_mavros fastlio_to_px4_mid360_direct.launch \
  rviz:=false zero_origin:=true
```

该 direct launch 已包含 `mapping_mid360_px4imu.launch`，不要再单独启动一份。
`fastlio_supervisor` 默认通过 `/home/orangepi/ros_ws/devel/env.sh` 启动该命令，
因此不依赖启动无人机后端的终端是否 source 过定位工作空间；路径可在
`positioning.workspace_env` 中修改。
当 `/Odometry`、`/mavros/vision_pose/pose` 和 PX4 odom 连续稳定 3 秒后，
系统记录 H 点并进入 `WAIT_START`。正式起飞只接受小车实体按键产生的
`mission/start`，且 `start_reason` 必须为 `car_button`。

## 任务流程

`DROP`：起飞至相对 H 点 1.50m，悬停 3 秒，按小车位姿接近，YOLO
锁定标靶并伴飞，稳定后立即下降到默认 0.80m 投放高度并触发投放流程，
随后爬升、返回 H 点和普通降落。`distance_to_d_m` 仅作 D 点截止保护，
不会等到 D 点附近才开始投放。

`DYNAMIC_LANDING`：起飞、搜索和伴飞后分 0.80m、0.30m 两段下降，
移动平台接触阶段继续发送平台位置与速度前馈。桥接层确认下压误差和低垂速
持续成立后锁桨，随车停留 5.20 秒，再预发 setpoint、切 OFFBOARD、解锁，
恢复 1.50m 高度并返航。接触超时最多自动取消并重试一次。

小车位姿超过 300ms 未更新时，网关关闭动态下降门。状态机立即保持当前
高度，只继续水平伴飞；不会用过期位姿继续向下。近距离标靶离开相机视野后，
跟踪器使用已锁定的“标靶相对小车偏移 + 小车实时位姿”短时预测。

## 现场参数

无人机任务常调参数集中在
`src/d_task_uav_control/config/d_task_uav.yaml`：

- `tracking.car_frame_offset_x_m/y_m`：小车 ROS 坐标起点到无人机坐标起点的平移。
- `camera.*`：无畸变相机内参和相机系到机体系外参。
- `tracking.platform_height_m`：平台中心在无人机世界系中的高度。
- `mission.cruise_height_m`、`drop_height_m`、两段下降高度和速度。
- 对中误差、相对速度、稳定时间、超时、5.20 秒停留和重试次数。
- `payload.*`：投放执行器预留配置。

参数在启动时加载，飞行中不热更新。调整坐标偏移、相机外参和平台高度后，
必须先做低高度、禁用自动解锁的实机验证。

YOLO 使用单类别平台标靶模型 `yolo_target_yolov8n_640_rk3588_i8.rknn`，标靶固定在
降落平台中心。检测节点每帧发布结构化检测，即使未发现目标也发布
`found=false`，融合节点使用四状态卡尔曼滤波输出平台位置和速度。

仅验证相机和模型时运行：

```bash
roslaunch metal_ball_rknn platform_target_test.launch
```

在地面站运行 `rqt_image_view` 并订阅 `/metal_ball_rknn/debug_image` 查看带框画面；
`/d_task/vision/platform_detection` 输出结构化检测结果。

投放硬件尚未定型，因此 `payload.enabled` 默认为 `false`。此时完整任务流程会
发布 `PAYLOAD_RELEASE_DRY_RUN` 事件，但不会向 PX4 输出通道发命令。确定接线、
PX4 mixer/control allocation 和安全通道后，再配置 MAVROS
`/mavros/actuator_control` 的 group、channel、释放值与脉冲时间。

## 关键状态与检查

地面站应依次看到 `POSITIONING_INIT -> WAIT_START -> TAKEOFF`，之后根据模式
进入投放或动态降落阶段，最终为 `COMPLETE` 或 `ABORT`。无人机 MQTT Agent
发布 ACK、state、tracking、health、heartbeat、event 和 safety/fault；任务状态
由任务节点发布，底层 bridge 状态不会覆盖正在运行的任务阶段。

首次实机运行前至少确认：MQTT 心跳稳定、三路定位 READY、相机健康、标靶世界
坐标方向正确、车机坐标偏移正确、300ms 断链能停止下降、平台锁桨条件不会在
空中误触发，以及投放执行器仍处于 dry-run。
