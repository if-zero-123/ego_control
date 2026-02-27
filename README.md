# ego_bridge

EGO-Planner → PX4 桥接功能包。绕过 `px4ctrl` 的自定义控制器，将 EGO-Planner 的轨迹命令（position + velocity + acceleration + yaw）直接发送给 PX4 内置位置控制器，实现更简洁的控制链路。

```
┌──────────┐    PositionCommand     ┌─────────────┐    PositionTarget     ┌─────┐
│ EGO-     │ ────────────────────▶  │ ego_bridge  │ ────────────────────▶ │ PX4 │
│ Planner  │    /position_cmd       │   node      │  /mavros/setpoint_   │     │
│ traj_srv │                        │             │   raw/local          │     │
└──────────┘                        └──────┬──────┘                      └─────┘
                                           │ ▲
              override_cmd                 │ │
┌──────────┐ ────────────────────▶         │ │
│ 你的代码  │ set_control_mode=1           │ │
│ (视觉等) │ ◀──── reach_status ───────────┘ │
└──────────┘                                  │
                                              │
┌──────────┐   TakeoffLand                    │
│ RC遥控器 │ ──────────────────────────────────┘
│ /脚本    │   takeoff.sh / land.sh
└──────────┘
```

---

## 目录结构

```
ego_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── ego_bridge_param.yaml      # 所有参数（含详细注释）
├── launch/
│   └── ego_bridge.launch          # 启动文件（桥接 + 可选RC）
├── scripts/
│   ├── takeoff.sh                 # 脚本触发起飞
│   └── land.sh                    # 脚本触发降落
└── src/
    ├── ego_bridge_node.cpp        # 核心桥接节点（6状态FSM）
    └── rc_commander_node.cpp      # RC 遥控器节点（可选）
```

---

## 状态机

```
IDLE ──(起飞命令)──▶ PRE_OFFBOARD ──(OFFBOARD+ARM)──▶ TAKEOFF ──(到达高度)──▶ HOVER
                                                                               │  ▲
                                                               (EGO有新cmd)   │  │ (cmd超时 /
                                                                               ▼  │  OVERRIDE切回)
                                                                            TRACKING
                                                                               │
                                                               (降落命令)      │
                                                                               ▼
                                                                            LANDING ──(着地)──▶ IDLE
```

| 状态 | 说明 | 发给 PX4 |
|------|------|----------|
| **IDLE** | 地面待机，等待起飞命令 | 无 |
| **PRE_OFFBOARD** | 预发 setpoint，尝试切 OFFBOARD 并 ARM | position（当前位置） |
| **TAKEOFF** | 电机预热 → 匀速爬升 → 接近高度减速 | position + velocity |
| **HOVER** | 锁定位置悬停，等待 EGO 命令或降落 | position（锁定点） |
| **TRACKING** | 转发 EGO 轨迹命令（或 OVERRIDE 时转发你的命令） | position + velocity + acceleration + yaw |
| **LANDING** | 匀速下降 → 近地减速 → 着陆检测 → DISARM | position + velocity |

---

## 快速开始

### 1. 编译

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

### 2. 配置参数

编辑 `config/ego_bridge_param.yaml`，关键参数：

```yaml
ctrl_freq: 50.0           # 控制频率(Hz)
offboard:
  enable_auto_arm: true    # 自动解锁（调试时建议 false）
takeoff:
  height: 1.0              # 起飞高度(m)
  speed: 0.3               # 爬升速度(m/s)
landing:
  speed: 0.3               # 降落速度(m/s)
```

### 3. 配置 PX4 参数（QGC 中设置）

| PX4 参数 | 推荐值 | 说明 |
|----------|--------|------|
| `COM_OF_LOSS_T` | 1.0 | OFFBOARD 超时(s) |
| `COM_OBL_RC_ACT` | 0 | OFFBOARD 丢失 → Position 模式 |
| `NAV_RCL_ACT` | 2 | RC 丢失 → 自动降落 |
| `MPC_XY_VEL_MAX` | ≥ EGO max_vel | 水平最大速度 |
| `MPC_Z_VEL_MAX_UP` | ≥ 0.3 | 最大上升速度 |
| `MPC_Z_VEL_MAX_DN` | ≥ 0.3 | 最大下降速度 |
| `MPC_ACC_HOR` | ≥ EGO max_acc | 最大水平加速度 |

### 4. 启动

```bash
# 方式一：带 RC 遥控器
roslaunch ego_bridge ego_bridge.launch

# 方式二：不用 RC（用脚本触发起飞降落）
roslaunch ego_bridge ego_bridge.launch enable_rc:=false
```

### 5. 起飞

```bash
# 方式一：脚本
./src/src/realflight_modules/ego_bridge/scripts/takeoff.sh

# 方式二：手动 rostopic
rostopic pub -1 /ego_bridge/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"

# 方式三：RC 遥控器
# SWB(ch4) 从低拨到高，SWC(ch5) 保持低位
```

### 6. 降落

```bash
# 脚本
./src/src/realflight_modules/ego_bridge/scripts/land.sh

# RC：SWB 保持高位，SWC 从低拨到高
```

---

## 完整飞行流程

```
1. 启动 EGO-Planner:
   $ roslaunch ego_planner single_run_in_exp.launch

2. 启动 ego_bridge:
   $ roslaunch ego_bridge ego_bridge.launch

3. 起飞:
   $ ./scripts/takeoff.sh
   
   → PRE_OFFBOARD: 预发 setpoint 2秒 → 切 OFFBOARD → ARM
   → TAKEOFF:      电机预热 2秒 → 以 0.3m/s 爬升到 1.0m → 接近时减速
   → HOVER:        悬停 2秒 → 发 /traj_start_trigger → EGO 开始规划

4. EGO 规划完成后自动进入 TRACKING，跟踪轨迹飞行

5. 到达后降落:
   $ ./scripts/land.sh
   
   → LANDING: 以 0.3m/s 下降 → 近地(<0.3m)减半速 → 着地检测 → DISARM
```

---

## 控制权交接（OVERRIDE）

在 TRACKING 状态下，你可以随时接管控制，用自己的代码（视觉伺服、精准投递等）直接控制飞机。

### 接管

```bash
# 1. 切换到 OVERRIDE 模式
rostopic pub -1 /ego_bridge/set_control_mode std_msgs/UInt8 "data: 1"

# 2. 你的代码以 ≥50Hz 发布控制命令
#    话题: /ego_bridge/override_cmd
#    类型: quadrotor_msgs/PositionCommand
#    填写: position, velocity, acceleration, yaw
```

### 归还控制权

```bash
# 1. 切回 EGO 模式
rostopic pub -1 /ego_bridge/set_control_mode std_msgs/UInt8 "data: 0"

# → 飞机在当前位置悬停（进入 HOVER）
# → 忽略 EGO 的旧命令（等待新轨迹）

# 2. 给 EGO 发一个新目标点，让它从当前位置重新规划
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 0.0, z: 1.0}}}"

# → EGO 重规划 → traj_server 发出新轨迹（trajectory_id 变化）
# → ego_bridge 检测到新 trajectory_id → 恢复 TRACKING
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped

rospy.init_node('my_controller')

mode_pub = rospy.Publisher('/ego_bridge/set_control_mode', UInt8, queue_size=1)
cmd_pub  = rospy.Publisher('/ego_bridge/override_cmd', PositionCommand, queue_size=1)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

rospy.sleep(0.5)

# ── 接管控制 ──
mode_pub.publish(UInt8(data=1))
rospy.sleep(0.1)

# ── 发送你的控制命令（例：悬停在指定位置） ──
rate = rospy.Rate(50)
for i in range(500):  # 控制 10 秒
    cmd = PositionCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.position.x = 2.0
    cmd.position.y = 0.0
    cmd.position.z = 1.0
    cmd.velocity.x = 0.0
    cmd.velocity.y = 0.0
    cmd.velocity.z = 0.0
    cmd.acceleration.x = 0.0
    cmd.acceleration.y = 0.0
    cmd.acceleration.z = 0.0
    cmd.yaw = 0.0
    cmd_pub.publish(cmd)
    rate.sleep()

# ── 归还控制权 ──
mode_pub.publish(UInt8(data=0))
rospy.sleep(1.0)

# ── 给 EGO 发新目标 ──
goal = PoseStamped()
goal.header.frame_id = 'world'
goal.pose.position.x = 5.0
goal.pose.position.y = 0.0
goal.pose.position.z = 1.0
goal.pose.orientation.w = 1.0
goal_pub.publish(goal)
```

---

## 到达检测

外部代码可以通过 `~target_point` 设定目标点，桥接节点会在飞机到达后发布通知。

```bash
# 设定目标点
rostopic pub -1 /ego_bridge/target_point geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 0.0, z: 1.0}}}"

# 监听到达状态
rostopic echo /ego_bridge/reach_status
# data: 0  ← 飞行中
# data: 1  ← 已到达（位置误差 < 0.3m 且 EGO速度 ≈ 0 持续 1秒）
```

到达判定条件（均可在 YAML 中调整）：
- `reach_detect/pos_threshold`: 位置阈值(m)，默认 0.3
- `reach_detect/vel_threshold`: 速度阈值(m/s)，默认 0.1
- `reach_detect/hold_time`: 持续时间(s)，默认 1.0

---

## RC 遥控器操作

需要在 launch 中 `enable_rc:=true`（默认开启）。

| 操作 | 遥控器动作 | 说明 |
|------|-----------|------|
| **起飞** | SWB(ch4) 低→高，SWC(ch5) 保持低 | 拨上主开关 |
| **降落** | SWB 保持高，SWC 低→高 | 拨上模式开关 |
| **紧急停止** | SWB 高→低 | 拨下主开关 → 停发 setpoint → PX4 failsafe |

通道号和阈值可在 YAML 的 `rc:` 部分修改。

---

## 话题一览

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/mavros/state` | mavros_msgs/State | PX4 状态 |
| `/mavros/extended_state` | mavros_msgs/ExtendedState | 着陆检测 |
| `~odom` | nav_msgs/Odometry | 里程计 |
| `~cmd` | quadrotor_msgs/PositionCommand | EGO 轨迹命令 |
| `~takeoff_land` | quadrotor_msgs/TakeoffLand | 起飞/降落触发 |
| `~target_point` | geometry_msgs/PoseStamped | 到达检测目标点 |
| `~set_control_mode` | std_msgs/UInt8 | 控制模式切换 |
| `~override_cmd` | quadrotor_msgs/PositionCommand | 接管控制命令 |
| `~emergency_stop` | std_msgs/Empty | 紧急停止 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/mavros/setpoint_raw/local` | mavros_msgs/PositionTarget | 发给 PX4 的控制指令 |
| `/traj_start_trigger` | geometry_msgs/PoseStamped | 通知 EGO 开始规划 |
| `~reach_status` | std_msgs/UInt8 | 0=飞行中, 1=已到达 |
| `~flight_state` | std_msgs/String | 当前状态名 |
| `~control_mode` | std_msgs/UInt8 | 0=EGO, 1=OVERRIDE |
| `~debug` | std_msgs/Float64MultiArray | 调试信息（需配置启用） |

---

## 安全机制

| 保护 | 触发条件 | 行为 |
|------|---------|------|
| **里程计超时** | odom 超过 0.5s 无更新 | 停发 setpoint → PX4 自身 failsafe 降落 |
| **OFFBOARD 丢失** | PX4 被 RC 切出 OFFBOARD | 打 WARN，不自动重切 |
| **EGO 命令超时** | traj_server 超过 0.5s 无消息 | TRACKING → HOVER 悬停 |
| **OVERRIDE 命令超时** | 接管代码超过 0.5s 无消息 | 在当前位置悬停（不退出 OVERRIDE） |
| **紧急停止** | RC 或 emergency_stop 话题 | 立即停发所有 setpoint → PX4 failsafe |

---

## 与 px4ctrl 的区别

| 维度 | px4ctrl | ego_bridge |
|------|---------|------------|
| 控制方式 | 自己算姿态+推力（SE3控制器） | 利用 PX4 内置位置控制器 |
| 发给 PX4 | `AttitudeTarget`（姿态+推力） | `PositionTarget`（位置+速度+加速度） |
| 需要标定 | 推力模型、质量、PID增益 | 不需要（PX4 自己管） |
| 控制精度 | 高（尤其高速激进飞行） | 中高（中低速完全够用） |
| 复杂度 | 高 | 低 |
| 控制权交接 | 无 | 支持 OVERRIDE 模式 |
| 到达检测 | 无 | 内置 |
