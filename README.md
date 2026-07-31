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

`DROP`：起飞至相对 H 点 1.50m，悬停 3 秒，将小车位姿从车端坐标系转换到
无人机世界系，再按卡尔曼估计的位置和速度直接接近；YOLO 锁定标靶后参与精确
伴飞，稳定后立即下降到默认 0.80m 投放高度并触发投放流程，
随后爬升、返回 H 点和普通降落。`distance_to_d_m` 仅作 D 点截止保护，
不会等到 D 点附近才开始投放。

任务1的视觉源按状态严格分段。`SEARCH_CAR/LOCK_CAR/FOLLOW_CAR` 在 1.50m
巡航高度只使用 YOLO 检测，公共检测结果继续进入已有的四状态
`[x,y,vx,vy]` 卡尔曼平台跟踪器；即使高空能看到 AprilTag，也不会抢占 YOLO。
`SEARCH_CAR` 只要收到新鲜的小车坐标，就会先按坐标估计飞向小车，不会在原地等
YOLO 出框；YOLO 可见后，其像素误差仅作为限幅后的细调量叠加到坐标控制上。

高空水平伴飞采用外环 PD 速度控制：

```text
v_cmd = v_car + Kp * (p_car - p_uav) + Kd * (v_car - v_uav) + vision_trim
```

其中 `p_car/v_car` 均来自坐标转换后的卡尔曼平台状态。速度指令以 30Hz 通过
`/ego_bridge/override_cmd` 发送，始终使用 `control_mode=1`；水平目标位置填入
无人机当前位置，避免桥接层位置环与外部 PD 重复控制，Z 方向仍使用任务目标高度。
全流程不接入 EGO-Planner，也不启用避障。无人机像素对中连续稳定 1 秒并进入
`DROP_DESCEND` 后，网关把本任务的
`follow_established=true` 持续回传给小车，小车据此从低速切到正常速度。投放状态机
同时开放 AprilTag 检测；`DROP_DESCEND/RELEASE` 中优先采用新鲜且识别成功的
AprilTag。真实检测中心和框先经过轻量 α-β 滤波，瞬间丢码时最多按匀速预测
`0.18s`（30Hz 下约 5 帧），预测置信度随时间衰减；超过窗口后再回退到新鲜的
YOLO。离开这两个状态后自动回到仅 YOLO。
投递动作本身不使用 YOLO 估计高度：只有新鲜有效的 AprilTag
`plane_distance_m <= 0.40m`，同时像素、XY 和相对速度均已对准，才会进入释放。
达到 0.40m 门槛后立即停止下降，连续稳定 `0.50s` 再触发投递；测距失效时保持当前位置，
超过失视超时后中止返航，不会按世界坐标高度盲投。
预测帧只维持图像源和中心显示，不产生三维测距，也不会刷新控制层的真实观测或触发投递。

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

- `tracking.car_frame_offset_x_m/y_m`：小车 ROS 坐标系到无人机世界系的 X/Y 平移。
- `tracking.car_frame_yaw_offset_rad`：两个坐标系之间的平面航向旋转。
- 现场把无人机中心放在小车坐标圆点时，无人机本地坐标测得
  `(0.7139167116, -0.3870449346)m`，因此当前平移参数已按原符号写入；
  两边均采用 X 向前、Y 向左，`car_frame_yaw_offset_rad` 保持 `0.0`。
- `tracking.use_visual_projection`：默认 `false`，避免未标定视觉投影影响小车世界坐标；仅保留旧方案时才开启。
- `tracking.platform_height_m`：平台中心在无人机世界系中的高度。
- `apriltag.track_filter_alpha/beta`：AprilTag 中心和框的 α-β 滤波权重。
- `apriltag.prediction_timeout_s`：瞬时丢码预测窗口，默认 `0.18s`。
- `apriltag.max_velocity_px_s`：预测像素速度限幅，默认 `800px/s`。
- `apriltag.reacquire_distance_px`：重捕获跳变阈值，默认 `120px`；超过后直接重置滤波。
- `pixel_servo.*`：像素滤波、死区、最大横移速度及图像轴到机体系轴的映射；必须先完成台架四方向确认。
- `mission.cruise_height_m`、`drop_height_m`、两段下降高度和速度；
- `mission.drop_apriltag_distance_m`：任务1的 AprilTag 码面投递距离，当前为 `0.40m`；
- `mission.apriltag_range_timeout_s`：码测距新鲜度窗口，当前为 `0.35s`。
- `mission.follow_xy_kp/kd`：高空伴飞外环 PD 增益，默认 `0.80/0.25`。
- `mission.follow_position_deadband_m`：水平位置死区，默认 `0.04m`。
- `mission.follow_max_correction_mps`、`follow_max_total_speed_mps`：
  PD 修正和总水平速度上限，默认 `0.30m/s`、`0.50m/s`。
- `mission.follow_max_accel_mps2`：水平速度指令加速度上限，默认 `0.50m/s²`。
- `mission.vision_trim_max_speed_mps`：YOLO/AprilTag 像素细调速度上限，默认 `0.12m/s`。
- 对中误差、相对速度、稳定时间、超时、5.20 秒停留和重试次数。
- `payload.*`：投放执行器预留配置。

参数在启动时加载，飞行中不热更新。调整坐标偏移、像素轴映射和平台高度后，
必须先做低高度、禁用自动解锁的实机验证。

YOLO 使用单类别平台标靶模型 `model_2_yolo_target_yolov8n_640_rk3588_i8.rknn`，标靶固定在
降落平台中心。检测节点每帧发布结构化检测，即使未发现目标也发布
`found=false`。小车世界坐标由四状态卡尔曼滤波输出平台位置和速度；伴飞主控制使用
该状态做 XY 方向 PD，视觉只做有界细调。近距离下降时切换到 AprilTag，以处理 YOLO
无法完整框出平台的情况。

下降近距离跟踪使用平台中心的 AprilTag：

- 码族 `AprilTag 36h11`，ID `0`；
- 输入文件为
  `E:\电赛空地协同\output\pdf\AprilTag_36h11_ID0_80mm_print.pdf`；
- 必须按“实际大小/100%”打印，禁止适合页面或缩放；
- 黑色标签图形边长为 `80mm`，含完整白色静区的外框约 `100mm`；
- 平整、水平贴在平台中心，正面朝上对着下视相机，不能镜像、裁掉白边、覆膜强反光
  或折皱；
- 码在平台平面内旋转不影响 ID、中心位置和 `plane_distance_m`，没有强制箭头方向；
  为了让调试图的位姿坐标轴方向固定，建议把打印 PDF 的页面顶部朝向小车前进方向。

该 PDF 已渲染检查，并在 Orange Pi 的 OpenCV 4.2
`DICT_APRILTAG_36h11` 上实测识别为 ID 0。检测源话题为：

```text
/d_task/vision/platform_detection/yolo       # 高空 YOLO
/d_task/vision/platform_detection/apriltag   # 近距 AprilTag
/d_task/vision/platform_detection            # 状态门控后的公共输入
/d_task/vision/apriltag_range                 # AprilTag 三维测距与质量
/d_task/vision/apriltag_debug                 # AprilTag 调试图
```

相关参数在 `apriltag.*`、`detection_mux.*` 和 `mission.*`。默认码 ID 为 `0`、
物理边长 `0.080m`、短时预测窗口为 `0.18s`，检测与测距新鲜度窗口均为 `0.35s`。
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

- `detected/pose_valid`：是否看到码、是否成功解算位姿；
- `optical_axis_distance_m`：沿相机光轴的 Z 距离；
- `slant_range_m`：相机光心到码中心的空间直线距离；
- `plane_distance_m`：相机光心到码平面的垂直距离，下视相机选投递高度时重点观察；
- `mean_side_px`、`tag_tilt_deg`、`reprojection_error_px`：码大小、倾角和解算质量；
- `intrinsics_source`：`camera_info` 表示使用相机标定，`config_fallback` 表示使用
  YAML 中 `fx/fy/cx/cy` 的临时内参。

`/d_task/vision/apriltag_debug` 会直接在相机画面中叠加检测框、中心十字、像素位置
`u/v`、相机坐标 `X/Y/Z`、空间直线距离 `range`、到码面的垂直距离 `plane` 和位姿
坐标轴。这里使用相机光学坐标系：`X` 向图像右侧、`Y` 向图像下方、`Z` 沿镜头
向前；单位为米。未识别到码时左上角显示 `APRILTAG SEARCH`，看到码但位姿解算无效时，
码旁的三维位置与距离显示 `N/A`，不会把无效值伪装成零。
真实帧以绿色 `APRILTAG FILTER` 标记；短时丢码预测帧以橙色框和
`APRILTAG PREDICT` 标记，并显示预测年龄与衰减置信度。预测帧的距离固定显示 `N/A`。

DECXIN 相机当前固定使用 640×480、无畸变图像。实测时把黑色边长 80mm 的标签正对
镜头，镜头平面到码面为 0.500m；连续 120 帧得到水平/垂直边长中位数
145.500px/145.507px，据此反标：

```text
fx=909.375000  fy=909.418101
cx=319.500000  cy=239.500000
D=[0,0,0,0,0]
```

标定文件为 `config/decxin_640x480.yaml`，完整任务和只读测距入口都会给 `usb_cam`
加载该文件；正常情况下 Topic 应显示 `intrinsics_source: camera_info`。该结果是服务于
平台中心测距的单距离正视标定，不等同于多姿态棋盘格完整标定。仍应从高到低记录稳定
识别下限、`plane_distance_m` 与卷尺真值的误差和重投影误差，再确定留有安全裕量的
投递高度。

拆桨、禁止解锁后，可观察识别与切换状态：

```bash
rostopic echo /uav_protocol/task_state
rostopic echo /d_task/vision/platform_detection/yolo
rostopic echo /d_task/vision/platform_detection/apriltag
rostopic echo /d_task/vision/platform_detection
rostopic echo /d_task/vision/apriltag_range
rqt_image_view /d_task/vision/apriltag_debug
```

高空 `FOLLOW_CAR` 时公共结果应来自 YOLO；进入 `DROP_DESCEND` 后日志应出现
`apriltag enabled`，码识别成功时出现 `selected source=APRILTAG`。码瞬时丢失
`0.18s` 内仍保持 AprilTag 源并发布带标志的预测中心，超过窗口后才回退 YOLO；
识别源都超过 `0.35s` 未更新时 mux 不再发布，现有下降失视保护会保持位置并按原
超时策略处理。

临时回退近距码追踪时，在完整启动命令后加 `start_apriltag:=false`；mux 会在下降
阶段继续使用可用的 YOLO，公共 Topic 和任务状态机无需改回旧代码。实飞前分级验证：

1. 拆桨并设 `payload.enabled: false`，用打印码确认 AprilTag 私有 Topic 和调试图稳定。
2. 维持 `FOLLOW_CAR`，同时让 YOLO 与码都可见，确认公共输出仍选 YOLO。
3. 只模拟或安全进入 `DROP_DESCEND`，确认码成功时选择 AprilTag；短暂遮码时先出现
   橙色 `PREDICT`，超过 `0.18s` 后再回退 YOLO。
4. 同时遮挡两种目标超过 `0.35s`，确认下降冻结且不会触发投放。
5. 低高度系留验证通过后再装桨；首次完整任务仍保持投放 dry-run，最后才恢复 GPIO。

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
