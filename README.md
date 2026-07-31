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

   更换摄像头设备时可附加：

   ```bash
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

先用 `--dry-run` 可查看启动顺序而不启动进程。脚本前台常驻，每次启动的记录写入
`~/.ros/d_task_uav/<启动时间>_<PID>/`。默认只保留各进程日志，不启动任务话题录包。
需要现场复盘时显式附加 `--record-bag`，任务话题包会按 256MB 分片写入同一目录，
且不包含相机图像、AprilTag 调试图和点云。脚本启动完成时会打印本次目录。

```bash
/home/orangepi/catkin_ws/src/ego_control/src/d_task_uav_control/scripts/start_d_task_uav.sh \
  --mqtt-host 192.168.0.198 \
  --record-bag
```

启用录包后，rosbag 异常退出只会输出醒目警告，不会停止飞行任务主流程。按 `Ctrl+C` 时只停止
脚本自己启动的进程，不停止启动前已经存在的 ROS Master、MAVROS 或 MID360 驱动。脚本不会
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

`DROP`：定位就绪时锁存 H 点和起飞航向，起飞至相对 H 点 1.50m。进入悬停后，先按
起飞航向向右前方 30° 移动 0.50m，再沿锁定的起飞航向向前搜索 1.00m。搜索途中一旦
真实识别 ID 0，立即进入现有图像中心 PID 伴飞；到达搜索终点仍未识别则直接返回 H、
降落并以 `ABORT` 结束。

伴飞没有固定时长。ID 0 在巡航高度持续对中 1.00s 后，状态进入 `RELEASE`，无人机保持
当前水平位置和 1.50m 高度，物理16脚输出5.0s投递脉冲并自动恢复低电平。投递完成后才
进入 `RETURN_HOME`；此状态通过 UAV `follow_established` 协议锁存通知小车提速。无人机
同时直接返回 H 点，稳定到位后进入 `LAND_HOME`，落地后发布 `COMPLETE` 和 mission reset。
mission reset 会释放 Orange 网关的当前任务上下文，使下一次地面站选任务能够重新触发
Fast-LIO。地面站对同一 mission 执行“重新建立定位”时会使用新的配置 command_id，网关
也会重新下发该配置，而不是当成传输重复包。

任务视觉源按状态严格分段。`SEARCH_CAR/LOCK_CAR` 在 1.50m 巡航高度以 ID 1-4
的 120mm 外围码为主；进入 `FOLLOW_CAR` 后如果 ID 0 中心码同时可见，也允许与外围码
一起联合解算，但没有中心码仍可正常伴飞和投递。任意一个大码都可按已知 195mm 偏移
反推出平台中心，多个可见码使用全部角点联合解算。进入动态下降后继续允许
ID 0-4 联合解算，45mm 中心码负责低段精确对中。公共平台中心继续进入已有的
四状态 `[x,y,vx,vy]` 卡尔曼平台跟踪器，像素误差仅作为有界细调量。

高空水平伴飞采用外环 PD 速度控制：

```text
v_cmd = v_tag + Kp * (p_tag - p_uav) + Kd * (v_tag - v_uav) + vision_trim
```

其中 `p_tag/v_tag` 均来自 AprilTag 平台中心投影后的卡尔曼状态。速度指令以 30Hz 通过
`/ego_bridge/override_cmd` 发送，始终使用 `control_mode=1`；水平目标位置填入
无人机当前位置，避免桥接层位置环与外部 PD 重复控制，Z 方向仍使用任务目标高度。
全流程不接入 EGO-Planner，也不启用避障。水平控制只读取下视相机 ID 0 的去畸变中心，
小车上传的 X/Y/速度不进入无人机 PID。短时丢码时水平指令归零并保持固定高度；只有新鲜
真实标签重新稳定对中才允许投递。`DYNAMIC_LANDING` 不由当前简化生产节点执行；选择该
模式会被任务节点明确拒绝，避免把动态降落错误地当成 DROP 流程飞行。

小车位姿超过 300ms 未更新时，网关仍会关闭动态下降安全门，但小车 X/Y/速度不进入
水平控制。高空伴飞短时失视时只按最后 AprilTag 状态预测，最多 1.0s；进入动态下降后
要求新鲜真实标签，只有已经通过 ID 0 近距对中门的 `LAND_ON_PLATFORM` 可在最终视觉
盲区继续预测下降。预测期间安全门关闭、视觉状态失效或超过 1.0s 会取消降落并爬升重试。

### AprilTag 独立跟随（当前实现）

水平平台跟踪改为只使用 AprilTag：外围码或中心码经过平台布局反推得到平台中心，
再结合相机内参、相机到机体外参和无人机姿态投影到无人机世界坐标，视觉卡尔曼状态
`[x,y,vx,vy]` 只接受这些标签观测。`/uav_protocol/car/pose` 的位置和速度不得初始化、
校正或延续平台滤波，也不得改变搜索、锁定、伴飞、投递或降落的水平指令。

固定航迹中只有真实 AprilTag 检测才能进入 `LOCK_CAR`。`LOCK_CAR/FOLLOW_CAR`
使用标签世界位置/速度作为外环目标，并保留标签中心像素细调。标签短时丢失时，只允许
使用最后一次标签状态预测最多 `1.0s`；超过预算后水平悬停并退出锁定，搜索超时则安全
返航。动态下降要求实时标签，最终近距盲区同样只允许最多 `1.0s` 的纯视觉预测，超时
取消降落。小车消息仅保留 `distance_to_d_m`、下降许可和安全门等协议字段，不参与平台
X/Y/速度融合。

验收要求包括：小车坐标位于无人机后方或速度符号错误时，不得改变 AprilTag 产生的
跟随方向；从未识别标签时不得因小车位姿进入视觉锁定；标签丢失超过预测预算后不得继续
追踪小车坐标；DROP 和 DYNAMIC_LANDING 的投递、低段中心码门和安全返航行为保持有效。

## 现场参数

无人机任务常调参数集中在
`src/d_task_uav_control/config/d_task_uav.yaml`：

- `simple_follow.initial_offset_distance_m/initial_offset_clockwise_deg`：起飞后右前搜索段，
  当前为 `0.50m/30°`。
- `simple_follow.forward_search_distance_m`：沿锁定起飞航向继续搜索，当前为 `1.00m`。
- `simple_follow.pid.*`：ID 0 图像中心 PID；当前 `kp=0.65`，Z 始终保持任务固定高度。
- `simple_follow.drop_alignment_tolerance/drop_alignment_stable_s`：投递前图像归一化误差
  和连续稳定门，当前为 `0.08/1.00s`。
- `simple_follow.home_xy_tolerance_m/home_stable_s`：返航到 H 的到位门，当前为
  `0.20m/0.50s`。
- `tracking.car_frame_offset_x_m/y_m`、`car_frame_yaw_offset_rad`：旧融合方案兼容参数；
  `use_car_measurements=false` 时不生效。
- 现场把无人机中心放在小车坐标圆点时，无人机本地坐标测得
  `(0.7139167116, -0.3870449346)m`，因此当前平移参数已按原符号写入；
  两边均采用 X 向前、Y 向左，`car_frame_yaw_offset_rad` 保持 `0.0`。
- `tracking.use_visual_projection`：当前为 `true`，将 AprilTag 平台中心投影到无人机世界系。
- `tracking.platform_height_m`：平台中心在无人机世界系中的高度。
- `apriltag.track_filter_alpha/beta`：AprilTag 中心和框的 α-β 滤波权重。
- `apriltag.layout`：ID 0-4 的尺寸、相对平台中心偏移和朝向。
- `apriltag.full_frame_interval/roi_expand_scale`：全图重捕获周期和跟踪 ROI 扩展比例。
- `apriltag.clahe_*`：灰度检测失败或低对比度时的 CLAHE 回退参数。
- `apriltag.range_filter_alpha/range_max_jump_m`：码面距离低通权重和跳变门限。
- `apriltag.prediction_timeout_s`：瞬时丢码预测窗口，默认 `0.18s`。
- `apriltag.max_velocity_px_s`：预测像素速度限幅，默认 `800px/s`。
- `apriltag.reacquire_distance_px`：重捕获跳变阈值，默认 `120px`；超过后直接重置滤波。
- `pixel_servo.*`：像素滤波、死区、最大横移速度及图像轴到机体系轴的映射；必须先完成台架四方向确认。
- `mission.cruise_height_m`、`drop_height_m`、两段下降高度和速度；
- `mission.search_start_x_m/y_m`：任务1固定搜索起点，当前为
  `(0.746, -0.379)m`；
- `mission.search_forward_distance_m`：到达搜索起点后沿世界坐标 `+X` 的搜索距离，
  当前为 `1.50m`；
- `mission.drop_apriltag_distance_m`：任务1的 ID 0 码面投递距离，当前为 `0.30m`；
- `mission.apriltag_range_timeout_s`：码测距新鲜度窗口，当前为 `0.35s`。
- `mission.follow_xy_kp/kd`：高空伴飞外环 PD 增益，默认 `0.80/0.25`。
- `mission.follow_position_deadband_m`：水平位置死区，默认 `0.04m`。
- `mission.follow_max_correction_mps`、`follow_max_total_speed_mps`：
  PD 修正和总水平速度上限，默认 `0.30m/s`、`0.50m/s`。
- `mission.follow_max_accel_mps2`：水平速度指令加速度上限，默认 `0.50m/s²`。
- `mission.vision_trim_max_speed_mps`：AprilTag 平台中心像素细调速度上限，默认 `0.12m/s`。
- `mission.landing_prediction_timeout_s`：最终接触阶段中心码盲区预测上限，默认 `1.0s`。
- `mission.landing_visual_handoff_s`：视觉修正切换到坐标预测的平滑时间，默认 `0.25s`。
- 对中误差、相对速度、稳定时间、超时、5.20 秒停留和重试次数。
- `payload.*`：投放执行器配置；当前启用 WiringOP 9、高电平有效、单次脉冲5.0s。

参数在启动时加载，飞行中不热更新。调整坐标偏移、像素轴映射和平台高度后，
必须先做低高度、禁用自动解锁的实机验证。

正式任务不再启动 RKNN YOLO 检测和检测源 mux，USB 相机图像直接进入 AprilTag 36h11
多码检测。贴装文件为
`/home/orangepi/catkin_ws/src/AprilTag_600mm_Car_Platform_Print_and_Placement.pdf`，
必须按 100%/实际大小打印并保持所有标签页面顶部朝向车头。当前布局为：

```text
ID 0: 45mm,  (x= 0.000, y= 0.000)m
ID 1: 120mm, (x=-0.195, y= 0.195)m
ID 2: 120mm, (x= 0.195, y= 0.195)m
ID 3: 120mm, (x= 0.195, y=-0.195)m
ID 4: 120mm, (x=-0.195, y=-0.195)m
```

检测节点将可见码全部角点放入统一平台坐标系求解，输出的是平台几何中心而不是某个
标签中心。高空状态过滤 ID 0，下降状态允许五码联合；无关 ID、过小码和高重投影误差
结果不会进入控制。检测话题为：

```text
/d_task/vision/platform_detection            # 融合后的平台中心
/d_task/vision/apriltag_range                 # AprilTag 三维测距与质量
/d_task/vision/apriltag_debug                 # AprilTag 调试图
```

相关参数在 `apriltag.*` 和 `mission.*`。短时预测窗口为 `0.18s`，检测与测距新鲜度
窗口均为 `0.35s`。
`PlatformDetection` 增加 `predicted` 和 `measurement_age_s`，用于明确区分真实检测与
预测中心；任务节点另行读取 `/d_task/vision/apriltag_range`，只把真实检测得到的有效
码面距离用于投递门控，不会另起一套水平控制器。

投递高度先用 AprilTag 测距实测，不直接猜值。拆桨后运行只读入口：

```bash
roslaunch d_task_uav_control apriltag_range_debug.launch
rostopic echo /d_task/vision/apriltag_range
rqt_image_view /d_task/vision/apriltag_debug
```

该 launch 只启动 USB 相机和 AprilTag 检测，不启动 `ego_bridge`、任务状态机、
MQTT 或投放执行器。`apriltag_range` 每帧发布：

- `detected/pose_valid/center_tag_visible`：平台是否解算成功、距离是否有效、ID 0 是否可见；
- `visible_tag_ids/used_tag_ids`：本帧检出的码和通过几何门限参与融合的码；
- `optical_axis_distance_m`：沿相机光轴的 Z 距离；
- `slant_range_m`：相机光心到码中心的空间直线距离；
- `plane_distance_m/raw_plane_distance_m`：滤波后和原始的相机到平台平面垂直距离；
- `mean_side_px`、`tag_tilt_deg`、`reprojection_error_px`：码大小、倾角和解算质量；
- `detection_region/preprocessing/processing_time_ms`：当前全图或 ROI、灰度或 CLAHE、耗时；
- `intrinsics_source`：`camera_info` 表示使用相机标定，`config_fallback` 表示使用
  YAML 中 `fx/fy/cx/cy` 的临时内参。

`/d_task/vision/apriltag_debug` 叠加每个 ID、融合中心、平台坐标轴、跟踪框、
`FULL/ROI`、`GRAY/CLAHE`、使用 ID、滤波/原始距离、重投影误差和单帧耗时。
短时预测使用橙色框，预测帧距离固定为 `N/A`。

DECXIN 相机当前固定使用 640×480。完整任务从 CameraInfo 读取内参和畸变参数，先将
检测到的 ID 0 中心去畸变，再相对标定主点和可调像素偏移计算伴飞误差。当前标定为：

```text
fx=854.64215684  fy=858.77973527
cx=322.79506911  cy=237.32768003
D=[0.1898368982,-0.9235811664,0.0042989883,0.0011198578,0.0]
```

标定文件为 `config/decxin_640x480.yaml`。相机固定偏角通过
`simple_follow.target_offset_u_px/v_px` 补偿，`u` 向右为正、`v` 向下为正。在无人机
物理位置正确时读取 `/d_task/simple_follow/debug`：偏移量应分别设为
`servo_observed_u-cx` 和 `servo_observed_v-cy`，调整后 `servo_observed` 应收敛到
`servo_target`。正常运行时 `camera_info_ready` 必须为 `true`。

自动标定偏移时先停止完整任务后端，拆桨并保持机体水平。要补偿相机相对机体的安装
位置，应把机体中心而不是相机本体放在 ID 0 正上方；相机到标签距离尽量接近实际
1.50m 伴飞高度。执行：

```bash
roslaunch d_task_uav_control tag0_offset_calibration.launch
```

该入口只启动 USB 相机和标定节点，不启动 MAVROS、`ego_bridge`、任务节点、MQTT 或
Fast-LIO。节点默认采集90帧去畸变中心，像素标准差不超过1.5时自动更新
`config/d_task_uav.yaml`，随后关闭相机并退出。原配置备份到
`~/.ros/d_task_uav/calibration_backups/`；超时或画面抖动时不会写配置。配置修改后需
重启正式任务后端才会生效。

拆桨、禁止解锁后，可观察识别与切换状态：

```bash
rostopic echo /uav_protocol/task_state
rostopic echo /d_task/vision/platform_detection
rostopic echo /d_task/vision/apriltag_range
rqt_image_view /d_task/vision/apriltag_debug
```

高空 `FOLLOW_CAR` 调试图应显示 `policy=OUTER_ONLY`；进入 `DROP_DESCEND`、
`DESCEND_HIGH` 等下降状态后应显示 `policy=CENTER+OUTER`。码瞬时丢失 `0.18s`
内发布带标志的预测中心，超过窗口后检测失效并由状态机执行停降或盲区安全策略。
实飞前分级验证：

1. 拆桨并设 `payload.enabled: false`，用完整贴码平台确认 ID 1-4 高空识别和融合中心稳定。
2. 遮挡部分外围码，确认单码/多码之间切换时平台中心没有明显跳变。
3. 安全进入 `DROP_DESCEND`，确认 ID 0 距离大于 0.30m 不投递，达到门槛并稳定 0.50s 才投递。
4. 动态降落在 0.30m 完成 ID 0 对中后遮挡中心码，确认继续随车下压；位姿过期时应取消重试。
5. 低高度系留验证通过后再装桨；首次完整任务仍保持投放 dry-run，最后才恢复 GPIO。

确认下视相机的图像轴与机体系方向时，拆桨、禁止解锁后运行：

```bash
roslaunch d_task_uav_control pixel_servo_debug.launch
```

该入口只启动相机、多码检测和调试节点，不启动任务状态机、`ego_bridge` 或飞控控制。
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

2026-07-31 任务1改动验证记录：完整 `catkin_make` 成功；`d_task_uav_control` 与
`uav_protocol_gateway` 测试结果汇总为 `116 tests, 0 errors, 0 failures`；新增
AprilTag 检测、检测源门控和协议伴飞锁存均有独立测试。
`roslaunch --nodes d_task_uav_control d_task_uav.launch` 已确认实际启动链包含
`metal_ball_detector`、`apriltag_detector`、`platform_detection_mux`、
`platform_tracking_fusion` 和 `d_task_mission`。这些验证未启动飞行或投放动作。

2026-07-31 坐标伴飞 PD 验证记录：完整 `catkin_make` 再次成功；
`d_task_uav_control` 单包汇总为 `88 tests, 0 errors, 0 failures`，
`d_task_uav_control` 与 `uav_protocol_gateway` 相关测试合计为
`107 tests, 0 errors, 0 failures`。新增测试覆盖坐标系旋转和平移、无 YOLO 时
按坐标接近、PD 总速度限幅、加速度限幅，以及投递前坐标/相对速度/像素三重对齐。
`roslaunch --nodes d_task_uav_control d_task_uav.launch` 已通过启动链解析。
验证仅包含编译、自动测试与 launch 解析，未启动飞行、解锁或投放硬件。

2026-07-31 AprilTag 测距调试验证记录：完整 `catkin_make` 成功；
`d_task_uav_control` 汇总为 `94 tests, 0 errors, 0 failures`。合成投影测试以
`80mm` 标签、已知 `0.600m` 光轴距离验证 PnP 回算，并覆盖无效内参拒绝。
`roslaunch --nodes d_task_uav_control apriltag_range_debug.launch` 仅列出 USB 相机和
测距节点，`rosmsg show d_task_uav_control/AprilTagRange` 字段生成正确。本次没有打开
相机、飞控、解锁或投放硬件，`drop_height_m` 仍保持原值，等待现场测距后再确定。

2026-07-31 DECXIN 内参实测记录：80mm 标签位于镜头平面到码面 0.500m 时，
120 帧水平/垂直边长中位数为 145.500px/145.507px，反标得到
`fx=909.375000`、`fy=909.418101`。实际启动确认 CameraInfo 正确发布该 K、全零 D，
测距消息使用 `intrinsics_source=camera_info`；完整任务与只读入口的 launch 参数导出
均指向同一标定文件。完整 `catkin_make` 成功，包测试为
`97 tests, 0 errors, 0 failures`。加载新 K 后复核时标签已离开当前画面，因此仍需在
码重新进入画面后用卷尺距离复核最终误差；投递高度尚未修改。

2026-07-31 AprilTag 图像位置叠加验证记录：完整 `catkin_make` 成功，包测试汇总为
`97 tests, 0 errors, 0 failures`。实际相机入口发布 640×480 调试图，当前镜头内无
AprilTag，因此正确显示 `APRILTAG SEARCH` 和 `K=camera_info`。另用隔离的合成图像
入口验证 ID 0 检出、中心位置 `u=379.5/v=259.5`、相机 `X/Y/Z`、`range/plane`
和三维坐标轴均叠加在码附近。`roslaunch --nodes` 仍只列出 USB 相机和测距节点；
验证后临时节点已精确停止，未启动飞控、任务状态机、MQTT 或投放执行器。

2026-07-31 0.40m 投递门控验证记录：任务1下降目标和平台跟踪释放参考高度改为
`0.40m`，任务节点新增 `/d_task/vision/apriltag_range` 订阅。只有新鲜有效的
`plane_distance_m <= 0.40m` 且像素、XY、相对速度均对准时才允许进入 `RELEASE`；
达到距离后先停止下降并完成 `0.50s` 稳定计时。新增测试覆盖测距丢失、`0.401m`
拒绝投递和门槛内停降，任务状态机 `21/21` 通过，全包汇总为
`103 tests, 0 errors, 0 failures`。完整构建和 launch 参数解析通过，本次未启动飞控、
解锁或投放硬件。

2026-07-31 AprilTag 平滑与短时预测验证记录：真实中心和检测框加入 α-β 滤波，
丢码后按限幅像素速度最多预测 `0.18s`，置信度线性衰减；大于 `120px` 的重捕获
跳变会直接重置滤波。`PlatformDetection` 使用 `predicted/measurement_age_s`
明确标出预测帧，任务像素伺服和世界坐标融合均不会把预测帧当作新测量。
AprilTag 距离仍只由当前真实四角 PnP 产生，预测帧距离为无效，因此不能触发
`0.40m` 投递。完整 `catkin_make` 成功，预测器 `6/6` 测试和
`d_task_uav_control` 全包 `116 tests, 0 errors, 0 failures` 通过；完整 launch
参数和节点链解析通过，未启动相机、飞控、解锁或投放硬件。

2026-07-31 车机坐标平移标定记录：小车圆点取小车坐标 `(0,0)`，无人机中心放在
圆点时本地坐标为 `x=0.7139167116197679m`、`y=-0.38704493464013184m`。
按实现 `p_uav=R(yaw_offset)·p_car+offset` 将这两个数原符号写入
`tracking.car_frame_offset_x_m/y_m`；两边坐标轴均为 X 向前、Y 向左，
故 `car_frame_yaw_offset_rad=0.0`。完整 launch 参数导出已确认融合节点加载精确值，
坐标变换专项 `4/4` 通过；未启动或重启飞行节点。

2026-07-31 Fast-LIO 任务触发修复记录：小车和无人机网关约定
`mission_config(sender=car)`，Fast-LIO supervisor 与无人机任务节点现已统一只接受
该 sender；旧的 `sender=ground` 会被拒绝。回归测试同时核对网关、supervisor 和
C++ 任务节点三处契约，防止后续再次出现本地二次校验不一致。完整 `catkin_make`
成功；`d_task_uav_control` 为 `119 tests, 0 errors, 0 failures`，
`uav_protocol_gateway` 为 `19 tests, 0 errors, 0 failures`，完整 launch 节点解析
通过。源码生效需要安全重启无人机任务后端，并由小车重新发布一个新的任务配置；
本次没有重启运行节点或解锁飞控。

2026-07-31 定位 READY 锁存修复记录：`startup_timeout_s` 现在只约束首次完成定位
初始化；三路定位连续稳定并进入 `WAIT_START` 后，READY 不会在任务配置 15 秒时被
错误反转为 `positioning_startup_timeout`。网关本地事件同时会消费并校验来源
`mission_id`，不再把它作为重复参数传给协议封装器；旧任务的本地事件会被拒绝。
新增测试覆盖 READY 跨过启动期限、匹配任务事件、无任务字段事件和旧任务事件拒绝。
完整 `catkin_make` 成功；`d_task_uav_control` 为
`120 tests, 0 errors, 0 failures`，`uav_protocol_gateway` 为
`22 tests, 0 errors, 0 failures`，完整 launch 节点解析通过。本次没有重启运行节点、
解锁飞控或修改 Kill Switch。

2026-07-31 任务1固定搜索航迹更新记录：起飞悬停后新增
`MOVE_TO_SEARCH_START` 和 `FORWARD_SEARCH`，先飞到世界坐标
`(0.746, -0.379)`，再沿 `+X` 搜索 1.50m；途中发现有效平台估计和真实 YOLO
检测后立即进入原有视觉锁定、伴飞和投递流程，到达 `(2.246, -0.379)` 仍未发现
平台则报错返航并最终以 `ABORT` 结束。动态起降任务继续使用原有 `SEARCH_CAR`。
完整 `catkin_make` 成功；`d_task_uav_control` 为
`130 tests, 0 errors, 0 failures`，`uav_protocol_gateway` 为
`23 tests, 0 errors, 0 failures`，Python 语法检查和完整 launch 参数导出通过。
本次未启动相机、飞控、任务状态机、自动解锁或投放 GPIO。

2026-07-31 五码 AprilTag 融合跟踪更新记录：正式任务链已关闭 YOLO 和
`platform_detection_mux`，改由 36h11 的 ID 1-4 外围码进行 SEARCH/LOCK/FOLLOW
平台中心融合；进入 FOLLOW_CAR 后中心 ID 0 如果可见也会加入联合 PnP，不能识别中心码时
仍使用外围码正常伴飞和投递；中心码仍是低段降落对齐的硬门。五码联合 PnP、外围码反推
平台中心、重投影误差剔除、任务阶段 ID 筛选均已加入。检测入口增加灰度检测、ROI 跟踪、
每 10 帧全图重捕获、ROI 丢失重捕获、低对比度 CLAHE 回退和亚像素角点；平台中心使用
alpha-beta 滤波，距离使用低通和跳变剔除。DROP 投递在 `1.50m` 伴飞高度进行，只要求
外围码融合的平台中心、像素/XY/相对速度对齐并稳定 `1.0s`；该段是历史实现记录，
其中小车位姿融合和 `5.0s` 盲区预测已被后续 AprilTag 独立跟随实现替代。
完整 `catkin_make` 成功；`catkin_make run_tests_d_task_uav_control` 与
`catkin_test_results build/test_results/d_task_uav_control` 汇总为
`162 tests, 0 errors, 0 failures, 0 skipped`；完整任务、只读测距和像素调试 launch
均已解析验证，未启动真实相机、飞控、解锁、投放或降落硬件。

2026-07-31 DROP 高空直接投递更新记录：DROP 在 `FOLLOW_CAR` 对齐稳定后保持
`cruiseZ=1.50m` 直接进入 `RELEASE`，不再下降到 `0.30m`，投递不再要求中心 ID 0
或近距测距；投递前仍要求外围码融合平台中心、像素/XY/相对速度对齐，并增加实际
巡航高度门。AprilTag 在 `FOLLOW_CAR` 允许中心 ID 0 可见时加入外围码联合 PnP，
中心码缺失仍可用外围码投递；DYNAMIC_LANDING 的 0.80m/0.30m 两段下降和 ID 0
低段准入保持不变。完整 `catkin_make` 成功；`catkin_test_results
build/test_results/d_task_uav_control` 汇总为 `164 tests, 0 errors, 0 failures,
0 skipped`，其中任务控制器 `31/31` 通过；未启动真实相机、飞控、解锁或投放硬件。

2026-07-31 实飞诊断记录更新：任务节点新增状态切换、故障和默认 1Hz 的结构化
诊断快照；一键启动脚本在任务后端就绪后自动录制关键任务话题，并随脚本退出统一
停止和收尾，单包按 256MB 分片且排除图像与点云。完整 `catkin_make` 成功，
`d_task_uav_control` 汇总为 `168 tests, 0 errors, 0 failures, 0 skipped`。测试覆盖
录包命令白名单、启动顺序、录包进程清理及诊断字段契约；未启动真实飞行、相机、
解锁或投放硬件。

2026-07-31 AprilTag-only 平台跟踪收敛记录：平台世界坐标和速度现在只接受
AprilTag 五码融合后的视觉投影，小车位姿默认不订阅、不初始化、不校正也不延续平台滤波；
高空搜索必须由真实 AprilTag 才能锁定，短时丢码只允许最后视觉状态预测 1.0s。
投递要求真实视觉对中，动态下降要求新鲜真实标签，最终中心码盲区预测上限为 1.0s，
超时立即悬停并取消降落。新增回归覆盖错误小车坐标、视觉预测超时、搜索锁定和动态下降
悬停；完整构建成功，`d_task_uav_control` 为 `182 tests, 0 errors, 0 failures, 0 skipped`，
launch 节点解析通过，未启动真实飞控、相机、解锁或投放硬件。
